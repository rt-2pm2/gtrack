/**
 * \file devinterface.cpp
 * \author Luigi
 *
 */

#include <iostream>
#include "utils/opencv_conv.hpp"
#include "devinterface/devinterface.hpp"
#include "utils/timelib.hpp"
#include "utils/utils.hpp"

DeviceInterface::DeviceInterface() {
	_isplayback = false;
	active = false;

	dev_cfg.playback = _isplayback;

	input_queue = new rs2::frame_queue(QUEUE_SIZE, false);
	filt_queue = new rs2::frame_queue(QUEUE_SIZE, false);

	init = true;
}

DeviceInterface::DeviceInterface(const DevConfig& cf,
		rs2::context ctx, rs2::device dev) : dev_cfg(cf) {
	active = false;
	dev_cfg  = cf;
	_isplayback = dev_cfg.playback;

	_ctx = ctx;
	_dev = dev;

	_serial = _dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

	input_queue = new rs2::frame_queue(QUEUE_SIZE, false);
	filt_queue = new rs2::frame_queue(QUEUE_SIZE, false);
	init = true;
}

DeviceInterface::~DeviceInterface() {
	active = false;

	if (input_thread.joinable()) {
		input_thread.join();
		std::cout << "Stopped input thread [" << 
			_serial << "]!" << std::endl;
	}
	
	if (conversion_thread.joinable()) {
		conversion_thread.join();
		std::cout << "Stopped conversion thread! [" << 
			_serial << "]" << std::endl;
	}
	
	std::cout << "Device Stopped!" << std::endl;
};


DevConfig& DeviceInterface::getDevConfig() {
	return dev_cfg;
}

// ========================================================
bool DeviceInterface::startDevice(int operation, std::string bag_fname = "") {
	switch (operation) {
		case (RSTRK_RECORDING):	
			{
			_cfg.enable_device(_serial);

			// Bag File
			std::string bag_name = 
				_dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) +
				std::string(".bag");
			std::cout << "RECORDING mode " << std::endl;
			if (file_exist(bag_name)) {
				std::cerr << "File already exist!" << std::endl;
				return -1;
			}
			std::cout << "Recording to " << bag_name << std::endl;
			_cfg.enable_record_to_file(bag_name);
			_isplayback = false;

			// Configure the pipeline
			_cfg.enable_stream(RS2_STREAM_DEPTH,
					dev_cfg.depth_width, dev_cfg.depth_height,
					dev_cfg.depth_format,
					dev_cfg.depth_framerate);

			_cfg.enable_stream(RS2_STREAM_COLOR,
					dev_cfg.rgb_width, dev_cfg.rgb_height,
					dev_cfg.rgb_format,
					dev_cfg.rgb_framerate);

			std::cout << "Starting Pipeline[" << _serial << 
				"]..." << std::endl;

			if (!init) { 
				std::cerr << "Device not configured..." << std::endl;
				return false;
			}
			break;	
			}
		case (RSTRK_PLAYBACK):
			{
			std::cout << "PLAYBACK mode " << std::endl;
			if (file_exist(bag_fname)) {
				std::cout << "Streaming from " << bag_fname << std::endl;
				_cfg.enable_device_from_file(bag_fname);
			} else {
				std::cout << "Source file does not exists!" << std::endl;
				return -1;
			}
			_isplayback = true;
			break;
			}
		default:
			{
			std::cout << "ONLINE mode " << std::endl;
			_isplayback = false;

			_cfg.enable_device(_serial);

			// Configure the pipeline
			_cfg.enable_stream(RS2_STREAM_DEPTH,
					dev_cfg.depth_width, dev_cfg.depth_height,
					dev_cfg.depth_format,
					dev_cfg.depth_framerate);

			_cfg.enable_stream(RS2_STREAM_COLOR,
					dev_cfg.rgb_width, dev_cfg.rgb_height,
					dev_cfg.rgb_format,
					dev_cfg.rgb_framerate);

			std::cout << "Starting Pipeline[" << _serial << 
				"]..." << std::endl;

			if (!init) { 
				std::cerr << "Device not configured..." << std::endl;
				return false;
			}
			break;
			}
	}
	profile = _pipeline.start(_cfg);
	std::cout << "Pipeline started!" << std::endl;

	// Now that the device is started I can get information
	// about the sensors and finish the configuration.
	rs2::depth_sensor d_sensor = profile.get_device().first<rs2::depth_sensor>();
	if (d_sensor && d_sensor.is<rs2::depth_stereo_sensor>()) {
		if (operation != RSTRK_PLAYBACK) {
			// Set Accuracy
			d_sensor.set_option(RS2_OPTION_VISUAL_PRESET,
					RS2_RS400_VISUAL_PRESET_HIGH_DENSITY);

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
		if (conversion_thread.joinable()) {
			conversion_thread.join();
			std::cout << "Stopped conversion thread [" << 
				_serial << "]!" << std::endl;
		}

		if (input_thread.joinable()) {
			input_thread.join();
			std::cout << "Stopped input thread [" << 
				_serial << "]!" << std::endl;
		}
	}
	_pipeline.stop();	
}

void DeviceInterface::playbackPause() {
	if (_isplayback)
		profile.get_device().as<rs2::playback>().pause();
}

void DeviceInterface::playbackResume() {
	if (_isplayback)
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

#ifdef DEVINTERFACE_DEBUG
	std::cout << "Camera Intrinsic: " << std::endl;
	std::cout << "dcoeff: " << dCoeff.t() << std::endl;
	std::cout << "Cam Matrix: " << std::endl;
	std::cout << cameraMatrix << std::endl;
#endif

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

std::string& DeviceInterface::getSerial() {
	return _serial;
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
	spat.set_option(RS2_OPTION_HOLES_FILL, 2);
	spat.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5);

	rs2::temporal_filter temp(0.4, 20, 2);
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

	double dt = 0;
	timespec t_now, t_old;
	clock_gettime(CLOCK_MONOTONIC, &t_now);
	t_old = t_now;

	while (active) {
		rs2::frameset fs_raw;	
		rs2::frameset fs_filt;
		fs_raw = input_queue->wait_for_frame();

		if (fs_raw) {

			clock_gettime(CLOCK_MONOTONIC, &t_now);
			dt = (timespec2micro(&t_now) - timespec2micro(&t_old)) / 1e6;
			t_old = t_now;
			//std::cout << "Sensor DT = " << dt << std::endl;

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

			timespec t1, t2;
			clock_gettime(CLOCK_MONOTONIC, &t1);
			frame2Mat(rgb_cv, rgb_rs, CV_8UC3, 1); 
			frame2Mat(depth_cv, depth_rs, CV_16UC1, 0);		
			data.set_rgb(rgb_cv);
			data.set_depth(depth_cv);

			// The conversion of the depth into colored image takes 
			// between 5 to 10 ms!
			//
			//frame2Mat(depthc_cv, depth_rs.apply_filter(color_map),
			//		CV_8UC3, 1);
			//data.set_depth_col(depthc_cv);
			clock_gettime(CLOCK_MONOTONIC, &t2);

			dt = (timespec2micro(&t2) - timespec2micro(&t1)) / 1e6;
			//std::cout << "Conversion = " << dt << std::endl;
		} else {
			std::cout << "Filtering thread: Timeout! " << std::endl;
		}
		signal_new();
	}
}
