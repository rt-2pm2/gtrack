/**
 * \file rstracker.cpp
 */

#include <Eigen/Dense>
#include "utils/timelib.hpp"
#include "rstracker/rstracker.hpp"


RSTracker::RSTracker(DeviceInterface* pdev) {
	_pdev = pdev;

	// Create the tracker class
	_ptrk = new MMTracker;
	_ptrk->setDepthScale(_pdev->getDevConfig().depth_scale);

	// Craete the filter class
	_pfilt = new DDFilter(Eigen::Vector3d::Zero(), 20, 0.03);	

	_ready = false;
	_shutting_down = false;

	_outfile.open(_pdev->getSerial() + "est.csv");
}

RSTracker::~RSTracker() {
	_outfile.close();
}

bool RSTracker::isReady() {
	return _ready;
}

bool RSTracker::stop_tracking() {
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

void RSTracker::addWorldMap(GlobalMap* map) {
	_world_map = map;	
	_ptrk->addWorldMap(map);
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

	// Get the intrisic parameters from the device as
	// cv::Mat	
	_pdev->getCameraParam(_cmat, _ddsf);
	_paruco_detector = new ArucoDetector(_cmat, _ddsf, 0.14, 0);

	std::thread(&RSTracker::autoSetWorldReference, this).detach();
}


void RSTracker::autoSetWorldReference() {
	int InitCounter = 60;
	cv::Mat cvFrame;
	while (InitCounter > 0) {	
		_pdev->synchronize();
		_pdev->get_rgb(cvFrame);
		int Ndetected = 0;
		if (!cvFrame.empty()) {
			Ndetected = _paruco_detector->processImage(
					_aruco_map, _detect_data, cvFrame, 0);
			InitCounter--;
		}

	}

	Eigen::Vector3d campos = _aruco_map[1].q_CM_.inverse() *
		(-_aruco_map[1].C_p_CM_);

	std::cout << "Camera [" << _pdev->getSerial() << "] Position = " <<
		campos.transpose() << std::endl;

	_ptrk->set_transform(_aruco_map[1].C_p_CM_,
			_aruco_map[1].q_CM_);
	_ready = true;
}

bool RSTracker::start_tracking() {
	// Block until the camera has acquired the refernce
	while (!_ready) { }

	std::cout << "Start Tracking from [" << _pdev->getSerial() << "]" <<
		std::endl;
	_trk_thread = std::thread(&RSTracker::track_runnable, this);

	return true;
}


MMTracker* RSTracker::getMMTracker() {
	return _ptrk;
}

DDFilter* RSTracker::getDDFilter() {
	return _pfilt;
}

DetectionData RSTracker::getArucoDetection() {
	return _detect_data;
}


void RSTracker::track_runnable() {
	double dt = 0.03;
	timespec t_now, t_old;
	t_old = t_now;

	cv::Mat cvRGB, cvDepth;
	cv::Mat cvFrameDepth;	
	std::array<float, 3> pos;
	Eigen::Vector3d W_t = Eigen::Vector3d::Zero();

	std::cout << "Activating Flow Thread!" << std::endl;
	_ptrk->start_flow();

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

		_pfilt->prediction(0.04);

		// Get the RGB image
		_pdev->get_rgb(cvRGB);
		_pdev->get_depth(cvDepth);

		int num_detect = 0;

		Eigen::Vector3d pos_ = Eigen::Vector3d::Zero();
		if (!cvRGB.empty() && !cvDepth.empty()) {
			// Check whether the camera is stabilized
			num_detect = _ptrk->step(cvRGB, cvDepth);

			if (num_detect > 0) {
				_ptrk->get_b_tg(0, pos);
				for (int i = 0; i < 3; i++) {
					pos_(i) = pos[i];
				}
				_pfilt->update(pos_);
			}
		}
		Eigen::Vector3d est_p = _pfilt->getPos();
		Eigen::Vector3d est_v = _pfilt->getVel();

		// Compute the position of the target w.r.t the World Frame	
		if (num_detect > 0) {
			W_t = (_aruco_map[1].q_CM_.inverse() *
					(pos_ - _aruco_map[1].C_p_CM_));	

			_world_map->add_target_data(0, W_t, Eigen::Vector3d::Zero(), timespec2micro(&t_now));
		}
		_outfile << timespec2micro(&t_now) << " ";
		_outfile << W_t(0) << " " << W_t(1) << " " << W_t(2) << " ";
		_outfile << est_p(0) << " " << est_p(1) << " " << est_p(2) << " ";
		_outfile << est_v(0) << " " << est_v(1) << " " << est_v(2);
		_outfile << std::endl;
	}
	_ptrk->stop_flow();
}
