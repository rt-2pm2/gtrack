/**
 * \file aruco_data.hpp
 *
 */

#ifndef INCLUDE_ARUCODETECTOR_ARUCO_DATA_HPP_
#define INCLUDE_ARUCODETECTOR_ARUCO_DATA_HPP_

#include <vector>
#include <opencv2/core.hpp>
#include <eigen3/Eigen/Dense>

struct DetectionData {
	uint64_t timestamp;

	// ID of the detected markers.
	std::vector<int> mk_ids_;

	// Image coordinates of the detected markers' corners.
	std::vector<std::vector<cv::Point2f> > mk_corners_;

	// Attitude of the detected markers wrt the camera frame.
	std::vector<cv::Vec3d> rvecs_;

	// Position of the detected markers wrt the camera frame.
	std::vector<cv::Vec3d> tvecs_;
};

struct BodyTarget {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	uint64_t timestamp; // Timestamp of the image
	int targetId; // Identifier of the target

	Eigen::Vector3d C_p_CM_; // Position of marker in camera frame
	Eigen::Quaterniond q_CM_; // Orientation of marker wrt camera
};

struct WorldTarget {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	uint64_t timestamp; // Timestamp of the image
	int targetId; // Identifier of the target

	Eigen::Vector3d W_p_WT_; // Position of the target
	Eigen::Quaterniond q_WT_; // Orientation of the target
};

#endif /* INCLUDE_ARUCODETECTOR_ARUCO_DATA_HPP_ */
