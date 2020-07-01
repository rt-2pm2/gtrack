/**
 * \file devinterface.cpp
 * \author Luigi
 *
 */

#include <iostream>
#include "utils/opencv_conv.hpp"
#include "devinterface/devinterface.hpp"


DeviceInterface::DeviceInterface() {
	playback = false;
	active = false;

	dev_cfg.playback = playback;

	input_queue = new rs2::frame_queue(QUEUE_SIZE, false);
	filt_queue = new rs2::frame_queue(QUEUE_SIZE, false);

	init = true;
}

DeviceInterface::DeviceInterface(const DevConfig& cf) : dev_cfg(cf){
	active = false;
	playback = false;
	dev_cfg.playback = playback;

	input_queue = new rs2::frame_queue(QUEUE_SIZE, false);
	filt_queue = new rs2::frame_queue(QUEUE_SIZE, false);
	init = true;
}

DeviceInterface::~DeviceInterface() {
	active = false;

	if (input_thread.joinable()) {
		input_thread.join();
		std::cout << "Stopped input thread!" << std::endl;
	}
	
	if (conversion_thread.joinable()) {
		conversion_thread.join();
		std::cout << "Stopped conversion thread!" << std::endl;
	}

	std::cout << "Device Stopped!" << std::endl;
};


// ========================================================
bool DeviceInterface::startDevice(bool playback, std::string bag_name) {

	// Configure the pipeline
	_cfg.enable_stream(RS2_STREAM_DEPTH,
			dev_cfg.depth_width, dev_cfg.depth_height,
			dev_cfg.depth_format,
			dev_cfg.depth_framerate);

	_cfg.enable_stream(RS2_STREAM_COLOR,
			dev_cfg.rgb_width, dev_cfg.rgb_height,
			dev_cfg.rgb_format,
			dev_cfg.rgb_framerate);

	std::cout << "Starting Pipeline..." << std::endl;

	if (!init) { 
		std::cerr << "Device not configured..." << std::endl;
		return false;
	}

	if (bag_name.size() > 1) {
		if (!playback) {
			std::cout << "Recording to " << bag_name << std::endl;
			_cfg.enable_record_to_file(bag_name);
		} else {
			std::cout << "Streaming from " << bag_name << std::endl;
			_cfg.enable_device_from_file(bag_name);
		}
	}
	
	profile = _pipeline.start(_cfg);
	std::cout << "Pipeline started!" << std::endl;

	// Now that the device is started I can get information
	// about the sensors and finish the configuration.
	rs2::depth_sensor d_sensor = profile.get_device().first<rs2::depth_sensor>();
	if (d_sensor && d_sensor.is<rs2::depth_stereo_sensor>()) {
		if (!playback) {
			// Set Accuracy
			d_sensor.set_option(RS2_OPTION_VISUAL_PRESET,
					RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);

			// Set Depth Scale
			d_sensor.set_option(RS2_OPTION_DEPTH_UNITS,
					dev_cfg.depth_scale);
		}
	}

	// Get calibration data
	camera_intr = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
	
	active = true;

	std::cout << "Starting Pipeline Threads..." << std::endl;
	input_thread = std::thread(&DeviceInterface::inflow, this);
	conversion_thread = std::thread(&DeviceInterface::filtering, this);

	std::cout << "Pipeline Operative" << std::endl;
	return true;
}

void DeviceInterface::stopDevice(bool wait) {
	active = false;

	if (wait) {
		if (input_thread.joinable()) {
			input_thread.join();
			std::cout << "Stopped input thread!" << std::endl;
		}

		if (conversion_thread.joinable()) {
			conversion_thread.join();
			std::cout << "Stopped conversion thread!" << std::endl;
		}
	}
}

void DeviceInterface::playbackPause() {
	if (playback)
		profile.get_device().as<rs2::playback>().pause();
}

void DeviceInterface::playbackResume() {
	if (playback)
		profile.get_device().as<rs2::playback>().resume();
}

bool DeviceInterface::getCameraParam(cv::Mat& cameraMatrix, cv::Mat& dCoeff) {
	bool out = false;
	if (active) {
		cameraMatrix = cv::Mat::zeros(cv::Size(3,3), CV_64FC1);
		dCoeff = cv::Mat::zeros(cv::Size(1,5), CV_64FC1);
		// Fill the opencv
		for (int i = 0; i < 5; i++) {
			dCoeff.at<double>(0,i) = camera_intr.coeffs[i];
		}

		cameraMatrix.at<double>(0,0) = camera_intr.fx;
		cameraMatrix.at<double>(1,1) = camera_intr.fy;
		cameraMatrix.at<double>(0,2) = camera_intr.ppx;
		cameraMatrix.at<double>(1,2) = camera_intr.ppy;
		out = true;
	} else {
		out = false;
	}

	return out;
}

bool DeviceInterface::getCameraParam(rs2_intrinsics& intr) {
	bool out = false;
	if (active) {
		intr = camera_intr;
		out = true;
	} else {
		out = false;
	}

	return out;
}


void DeviceInterface::get_rgb(cv::Mat& m) {
	data.get_rgb(m);
}

void DeviceInterface::get_depth(cv::Mat& m) {
	data.get_depth(m);
}

void DeviceInterface::get_depth_col(cv::Mat& m) {
	data.get_depth_col(m);
}


bool DeviceInterface::synchronize() {
	std::unique_lock<std::mutex> lck(mx_synch);
	cv_synch.wait(lck);

	return true;
}



// ===========================================================
// ===========================================================
void DeviceInterface::signal_new() {

	cv_synch.notify_all();
}

void DeviceInterface::inflow() {
	unsigned int frame_counter = 0;
	while (active) {
		rs2::frameset data_raw;
		bool gotData = false;

		data_raw = _pipeline.wait_for_frames();
		if (data_raw) {
			frame_counter++;

			input_queue->enqueue(data_raw);

		} else {
			std::cout << "Input thread: TIMEOUT!" << std::endl;
		}
	}
}
 

void DeviceInterface::filtering() {

	rs2::align align_to_depth(RS2_STREAM_DEPTH);
	rs2::align align_to_rgb(RS2_STREAM_COLOR);

	rs2::decimation_filter dec;
	dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);

	rs2::disparity_transform depth2disparity;
	rs2::disparity_transform disparity2depth(false);

	rs2::spatial_filter spat;
	spat.set_option(RS2_OPTION_HOLES_FILL, 1);
	spat.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5);

	rs2::temporal_filter temp(0.4, 20, 7);
	temp.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.8);
	temp.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);

	rs2::colorizer color_map;

	rs2::frameset fs;
	rs2::frame rgb_rs;
	rs2::frame depth_rs;

	cv::Mat rgb_cv;
	cv::Mat depth_cv;
	cv::Mat depthc_cv;

	unsigned int frame_counter = 0;

	while (active) {
		rs2::frameset fs_raw;	
		rs2::frameset fs_filt;
		fs_raw = input_queue->wait_for_frame();

		if (fs_raw) {
			frame_counter++;
			double timestamp = fs_raw.get_timestamp();
			fs_filt = fs_raw;
			fs_filt = fs_filt.apply_filter(dec);
			fs_filt = fs_filt.apply_filter(depth2disparity);
			fs_filt = fs_filt.apply_filter(spat);
			fs_filt = fs_filt.apply_filter(temp);
			fs_filt = fs_filt.apply_filter(disparity2depth);
			fs_filt = fs_filt.apply_filter(align_to_rgb);

			rgb_rs = fs_filt.get_color_frame();
			depth_rs = fs_filt.get_depth_frame();

			frame2Mat(rgb_cv, rgb_rs, CV_8UC3, 1); 
			frame2Mat(depth_cv, depth_rs, CV_16UC1, 0);
			frame2Mat(depthc_cv, depth_rs.apply_filter(color_map), CV_8UC3, 1);
			data.set_rgb(rgb_cv);
			data.set_depth(depth_cv);
			data.set_depth_col(depthc_cv);
		} else {
			std::cout << "Filtering thread: Timeout! " << std::endl;
		}
		signal_new();
	}
}
