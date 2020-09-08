/**
 * \file rstracker.cpp
 */

#include <Eigen/Dense>
#include "utils/timelib.hpp"
#include "rstracker/rstracker.hpp"


RSTracker::RSTracker(DeviceInterface* pdev) {
	_pdev = pdev;

	_flow_thread_active = false;
	_flowActive = false;;

	opt_flow_period.tv_nsec = 200 * 1e6; // ms
	opt_flow_period.tv_sec = 0;

	// Create the tracker class
	_ptrk = new mmtracker::MMTracker(pdev->getSerial());
	_ptrk->setDepthScale(_pdev->getDevConfig().depth_scale);

	// Craete the filter class
	_pfilt = new DDFilter(Eigen::Vector3d::Zero(), 20, 0.03);	

	_ready = false;
	_shutting_down = false;

	_outfile.open(_pdev->getSerial() + "est.csv");
	_logfile.open("log_rstrk<" + _pdev->getSerial() + ">.txt");

	_local_aruco_id = 0;
	_global_aruco_ref = 0;
}

RSTracker::~RSTracker() {
	stop_flow();
	_flow_thread_active = false;
	if (_flow_thread.joinable()) {
		_flow_thread.join();
		std::cout << "Stopped optical flow thread!" << std::endl;
	}
	_outfile.close();
	_logfile.close();
}

bool RSTracker::isReady() {
	return _ready;
}

void RSTracker::stop_tracking() {
	_shutting_down = true;
	std::cout << "Stopping Tracker " << _pdev->getSerial() <<
		"..." << std::endl;
	if (_trk_thread.joinable()) {
		_trk_thread.join();
	}
	std::cout << "Tracker Stopped" << _pdev->getSerial() <<
		"!" << std::endl;	
	_pdev->stopDevice(true);
}

void RSTracker::stop_flow() {
	_flowActive = false;
}

void RSTracker::addWorldMap(GAtlas* map) {
	_gatlas = map;	
}

void RSTracker::start_device(int operation, std::string fname) {

	_outfile << "Time" << " ";
	_outfile << "w_px w_py w_pz" << " ";
	_outfile << "b_px b_py b_pz" << " ";
	_outfile << "b_vx b_vy b_vz" << " ";
	_outfile << std::endl;

	_pdev->startDevice(operation, fname);
	// Get intrinsic parameters from the device.
	rs2_intrinsics intr;
	_pdev->getCameraParam(intr);
	_ptrk->setCamMatrix(intr);	


	_ptrk->setDepthScale(_pdev->getDevConfig().depth_scale);

	// Get the intrisic parameters from the device as
	// cv::Mat	
	_pdev->getCameraParam(_cmat, _ddsf);
	_paruco_detector = new ArucoDetector(_cmat, _ddsf, 0.14, 0);

	_atlas_thread = std::thread(&RSTracker::autoSetWorldReference, this);
}


void RSTracker::autoSetWorldReference() {

	timespec nextAct;
	timespec thread_period {};
	thread_period.tv_nsec = 0;
	thread_period.tv_sec = 1;

	clock_gettime(CLOCK_MONOTONIC, &nextAct);
	while (!_shutting_down) {
		add_timespec(&nextAct, &thread_period, &nextAct);
		// Get a frame and detect the arucos
		cv::Mat cvFrame;
		_pdev->synchronize();
		_pdev->get_rgb(cvFrame);
		int Ndetected = 0;
		mx.lock();
		if (!cvFrame.empty()) {
			Ndetected = _paruco_detector->processImage(
					_aruco_map, _detect_data, cvFrame, 0);
		}

		// Create the transformation pairs
		std::vector<int> aruco_ids;
		for (auto& el : _aruco_map) {
			aruco_ids.push_back(el.first);
		}

		if (aruco_ids.size() > 0) {
			_local_aruco_id = aruco_ids[0];
		}
		

		// If I have at least a pair of aruco
		if (Ndetected > 1) {
			for (int i = 0; i < Ndetected; i++) {
				for (int j = i + 1; j < Ndetected; j++) {
					int id_i = aruco_ids[i];
					int id_j = aruco_ids[j];
					// Compute the transformation between arucos
					Eigen::Quaterniond q_ij =
						_aruco_map[id_i].q_CM_.inverse() * _aruco_map[id_j].q_CM_;

					Eigen::Vector3d v_ij = 
						_aruco_map[id_i].q_CM_.inverse() *
						(_aruco_map[id_j].C_p_CM_ - _aruco_map[id_i].C_p_CM_);

					// Send the info to the GAtlas
					gatlas::TransformData tr;
					tr.t = v_ij;
					tr.rot = q_ij;
					_gatlas->setTransform(id_i, id_j, tr);
				}
			}
		}

		mx.unlock();


		// Check whether we can map the local measurement in the 
		// global reference system	
		gatlas::TransformData tr;
		bool res = _gatlas->getTransform(_global_aruco_ref,
				_local_aruco_id, tr);

		Eigen::Vector3d w_p_wk = tr.t;
		Eigen::Quaterniond w_q_k = tr.rot;	

		mx.lock();
		if (res) {
			// w_q_wc = w_q_k * k_q_c 
			_w_q_c = w_q_k * _aruco_map[_local_aruco_id].q_CM_.inverse();
			// w_p_wc = w_p_wk - w_q_c * c_p_ck 
			_w_p_wc = w_p_wk - 
				_w_q_c * _aruco_map[_local_aruco_id].C_p_CM_; 

			if (!_ready) {
				std::cout << "[" << _pdev->getSerial() << "] CAMERA POSITION: " <<
					_w_p_wc.transpose() << std::endl;
			}

			timespec t_now;
			clock_gettime(CLOCK_MONOTONIC, &t_now);
			_logfile << timespec2micro(&t_now) << " [" << _pdev->getSerial() << "] CAMERA POSITION: " <<
					_w_p_wc.transpose() << std::endl;

			_ready = true;
		} else {
			_ready = false;
		}
		mx.unlock();

		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,  &nextAct, NULL);
	}
}



bool RSTracker::start_tracking() {
	// Block until the camera has acquired the refernce

	std::cout << "Start Tracking from [" << _pdev->getSerial() << "]" <<
		std::endl;
	_trk_thread = std::thread(&RSTracker::track_runnable, this);

	return true;
}

void RSTracker::start_flow() {
	_flowActive = true;

	if (_flow_thread_active)
		return;

	_flow_thread_active = true;
	_flow_thread = std::thread(&RSTracker::opticalflow_runnable, this);
}

mmtracker::MMTracker* RSTracker::getMMTracker() {
	return _ptrk;
}

DDFilter* RSTracker::getDDFilter() {
	return _pfilt;
}

DetectionData RSTracker::getArucoDetection() {
	return _detect_data;
}


bool RSTracker::wait_ready() {

	// Consider taking a generic aruco as reference.
	std::cout << "Camera [" << _pdev->getSerial() <<
		"] Ref Aruco Position = " <<
		_w_p_wc.transpose() << std::endl;

	// Check if I need it...
	/*
	_ptrk->set_transform(_aruco_map[0].C_p_CM_,
			_aruco_map[0].q_CM_);
	*/
	_ready = true;

	return true;
}

void RSTracker::track_runnable() {
	double dt = 0.03;
	timespec t_now, t_old;
	t_old = t_now;

	cv::Mat cvRGB, cvDepth;
	cv::Mat cvFrameDepth;	
	Eigen::Vector3d pos;
	Eigen::Vector3d W_t = Eigen::Vector3d::Zero();
	
	while (!_shutting_down) {
		// This runnable is synchronized on the arrival of new frame
		_pdev->synchronize();

		clock_gettime(CLOCK_MONOTONIC, &t_now);
		dt = (timespec2micro(&t_now) - timespec2micro(&t_old)) / 1e6;
		t_old = t_now;

		//cout << "Tracker DT = " << dt << endl;

		// Avoid too little 'dt'
		if (dt < 0.0001)
			dt = 0.001;


		// Get the RGB image
		_pdev->get_rgb(cvRGB);
		_pdev->get_depth(cvDepth);

		int num_detect = 0;

		Eigen::Vector3d pos_ = Eigen::Vector3d::Zero();
		if (!cvRGB.empty() && !cvDepth.empty()) {
			// Check whether the camera is stabilized
			num_detect = _ptrk->step(cvRGB, cvDepth);

			if (num_detect > 0) {
				std::vector<mmtracker::TargetData> targets;
				_ptrk->get_targets(targets);

				if (_ready) {
					for (auto tg : targets) {
						Eigen::Vector3d pos_ = tg.b_tg;	
						mx.lock();
						W_t = _w_p_wc + _w_q_c * pos_;
						mx.unlock();
						/*
						   W_t = (_aruco_map[0].q_CM_.inverse() *
						   (pos_ - _aruco_map[0].C_p_CM_));
						*/
						bool valid = true;
						
						for (int i = 0; i < 3; i++) {
							if (std::isnan(W_t(i)) || !std::isfinite(W_t(i))) {
								valid = false;
								break;
							}
						}

						if (valid) {
						_gatlas->update_target_data(tg.id,
								W_t,
								Eigen::Vector3d::Zero(),
								timespec2micro(&t_now));
						}

						_outfile << timespec2micro(&t_now) << " " <<
							tg.id << " ";
						_outfile << W_t(0) << " " << W_t(1) << " " <<
							W_t(2) << " ";
						_outfile << pos_(0) << " " << pos_(1) << " " <<
							pos_(2) << " ";
						_outfile << std::endl;
					}
				}
			}
		}
	}
}



void RSTracker::opticalflow_runnable() {
	
	uint64_t T_us = timespec2micro(&opt_flow_period);

	// Should add a setup function to control the 
	// optical flow algorithm parameters.
	/*
	double pyr_scale = 0.5;
	int levels = 2;
	int winsize = 20;
	int niters = 2;
	int poly_n = 20;
	double poly_sigma = poly_n * 0.2;
	int flags = 0;

	_ptrk->setup_flow();
	*/
	timespec nextAct;
	cv::Mat cvFrame;

	clock_gettime(CLOCK_MONOTONIC, &nextAct);
	cv::Mat cFrame;
	while (_flow_thread_active) {
		// Compute the next activation
		add_timespec(&nextAct, &opt_flow_period, &nextAct);

		_pdev->synchronize();
		_pdev->get_rgb(cvFrame);

		// Optical Flow Step
		std::vector<mmtracker::TargetData> untracked_points;
		_ptrk->optical_flow_step(cvFrame, untracked_points);

		// The step should tell me if there are untracked movements
		// Let's assume I have vector of positions
		
		// Check in the global map if there is something there
		std::vector<gatlas::TargetData> atlas_items;
		_gatlas->get_items(atlas_items);

		timespec t_now;
		clock_gettime(CLOCK_MONOTONIC, &t_now);

		for (auto dtc_p : untracked_points) {
			// Compute the Word Frame position of the
			// identified point.
			mx.lock();
			Eigen::Vector3d W_t = _w_p_wc + _w_q_c * dtc_p.b_tg;
			mx.unlock();

			for (auto el : atlas_items) {
				double dist = (el.pos - W_t).norm();
				if (dist < 0.2) {
#ifdef RSTRACKER_DEBUG
					std::cout << "<" << _pdev->getSerial() <<
						"> Locking new target [" << el.id <<
						"]!" << std::endl;
#endif
					_ptrk->add_target(el.id, dtc_p.roi);

					_logfile << timespec2micro(&t_now) << 
						" [" << _pdev->getSerial() <<
						"] Locking new target [" << el.id <<
						"]!" << std::endl;
				} else {
#ifdef RSTRACKER_DEBUG
					std::cout << "[" << _pdev->getSerial() <<
						"] Not a good match" << std::endl <<
						"Map: " << el.pos.transpose() << std::endl <<
						"Detected: " << W_t.transpose() << std::endl <<
						"w.r.t Cam: " << dtc_p.b_tg.transpose() << std::endl <<
						"Cam: " << _w_p_wc.transpose() << std::endl;
#endif
					_logfile << timespec2micro(&t_now) <<
						" [" << _pdev->getSerial() <<
						"] Not a good match" << std::endl <<
						"Map: " << el.pos.transpose() << std::endl <<
						"Detected: " << W_t.transpose() << std::endl <<
						"w.r.t Cam: " << dtc_p.b_tg.transpose() << std::endl <<
						"Cam: " << _w_p_wc.transpose() << std::endl;
				}
			}
		}
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,  &nextAct, NULL);
	}

	std::cout << "Terminating FlowThread!" << std::endl;
}
