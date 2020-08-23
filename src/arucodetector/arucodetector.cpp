#include <iostream>
#include "arucodetector/arucodetector.hpp"

// Helper functions
void ocv2eig(const cv::Vec3d& in, Eigen::Vector3d& out) {
	for (int i = 0; i < 3; i++) {
		out(i) = in[i];
	}
	return;
}

/*
 * Default constructor
 */
ArucoDetector::ArucoDetector() :
			dictionary_(cv::aruco::getPredefinedDictionary(
						cv::aruco::DICT_APRILTAG_36h11)),
			B_p_BC_( Eigen::Vector3d::Zero()),
			q_BC_(Eigen::Quaterniond::Identity()),
			CameraID_(-1),
			ArucoSize_(0.0)
{}

/**
 * Constructor with Camera Matrix and Coef 
 */
ArucoDetector::ArucoDetector(
		cv::Mat camMat, cv::Mat distCoeffs,
		double markerSize, int cameraId) :
					dictionary_(cv::aruco::getPredefinedDictionary(
								cv::aruco::DICT_APRILTAG_36h11)),
					B_p_BC_(Eigen::Vector3d::Zero()), 
                    q_BC_(Eigen::Quaterniond::Identity()),
					cameraMatrix_(camMat),
					distCoeffs_(distCoeffs) {
	CameraID_ = cameraId;
	ArucoSize_ = markerSize;
}


void ArucoDetector::setOffsets(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
	B_p_BC_ = p;
	q_BC_ = q;
}


void ArucoDetector::setMarkerSize(double size) {
    ArucoSize_ = size;
}

void ArucoDetector::setCameraParams(const cv::Mat& camMat,
        const cv::Mat& distCoeffs) {
    camMat.copyTo(cameraMatrix_);
    distCoeffs.copyTo(distCoeffs_);
}

int ArucoDetector::processImage(
		std::unordered_map<int, BodyTarget>& out_map,
		DetectionData& ddata,
        const cv::Mat& image, 
        const uint64_t& timestamp) {

	int NumDetected = 0;
    ddata.timestamp = timestamp;

	NumDetected = detect(image, ddata);

	if (NumDetected > 0) {
		solvePnP(ddata);

        // Generate the Map with the information about detected targets
		generateMap(ddata, out_map);
	}

	return NumDetected;
}

/**
 * Detect Marker in an image
 */
int ArucoDetector::detect(const cv::Mat& image, DetectionData& dd) {
	cv::aruco::detectMarkers(
            image,
            dictionary_,
            dd.mk_corners_,
            dd.mk_ids_);

	return dd.mk_ids_.size();
}

/**
 * Estimate pose of detected markers
 */
int ArucoDetector::solvePnP(DetectionData& dd) {
	if (dd.mk_ids_.size() > 0) {
		cv::aruco::estimatePoseSingleMarkers(dd.mk_corners_,
				ArucoSize_,
				cameraMatrix_,
				distCoeffs_,
				dd.rvecs_,
				dd.tvecs_);
	} else {
		return 0;
	}

	return dd.mk_ids_.size();

}


int ArucoDetector::generateMap(const DetectionData& dd,
		std::unordered_map<int, BodyTarget>& outMap) {
    
    unsigned int Ntarget = dd.mk_ids_.size();

	for (unsigned int i = 0; i < Ntarget; i++) {
		int id = dd.mk_ids_[i];

		Eigen::Vector3d C_p_CM; // Camera to Marker in Camera frame
		Eigen::Vector3d rv;
		Eigen::Quaterniond q_CM; // Marker orientation w.r.t. Camera frame

		// Convert the position to eigen format
		ocv2eig(dd.tvecs_[i], C_p_CM);

		// Convert the orientation to eigen format
		ocv2eig(dd.rvecs_[i], rv);
		double alpha = rv.norm();
		rv.normalize();
		q_CM = Eigen::AngleAxisd(alpha, rv);

		BodyTarget bt;

		bt.timestamp = dd.timestamp;
		bt.targetId = id;
		bt.C_p_CM_ = C_p_CM;
		bt.q_CM_ = q_CM;

		outMap[id] = bt;
	}

    return Ntarget;
}

int ArucoDetector::getCameraId() const {
	return CameraID_;
}

void ArucoDetector::getCameraMat(cv::Mat& target) const {
	cameraMatrix_.copyTo(target);
}

void ArucoDetector::getDistCoeff(cv::Mat& target) const {
	distCoeffs_.copyTo(target);
}

double ArucoDetector::getArucoSize() const {
	return ArucoSize_;
}
