#ifndef ARUCODETECTOR_H
#define ARUCODETECTOR_H

#include <unordered_map>

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "aruco_data.hpp"


class ArucoDetector {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	/*
	 * Default constructur
	 */
	ArucoDetector();

    /**
	 * \brief	Class constructor.
	 *
	 * \param[in]	camMat		Camera Matrix.
	 * \param[in]	distCoeffs	Distortion Coefficients.
	 * \param[in]	markerSize	Size of the aruco marker.
	 * \param[in]	id			Id of the camera.
	 */
	ArucoDetector(cv::Mat camMat, cv::Mat distCoeffs,
			double markerSize, int id);

	/**
	 * \brief Set the offset of the camera with respect to the body frame.
	 *
	 * \param[in]	poff	Position offset.
	 * \param[in]	qoff	Attitude offset.
	 *
	 * \return 	void.
	 */
	void setOffsets(const Eigen::Vector3d& poff, const Eigen::Quaterniond& qoff);

	/**
	 * \brief Set camera parameters
	 */
	void setCameraParams(const cv::Mat& camMat, const cv::Mat& distCoeffs);


	/**
	 * \brief Set size of the aruco marker
	 */
	void setMarkerSize(double s);


	/**
	 * \brief	Detect aruco markers and solve the PnP problem for each of them.
	 * \param[out]  out_map	Map of the identifier target.
	 * \param[in]	image	Image data to be processed.
	 * \param[in]	t		Image timestamp.
	 *
	 * \return 				Number of detected markers.
	 */
	int processImage(std::unordered_map<int, BodyTarget>& out_map,
			DetectionData& ddata, const cv::Mat& image, const uint64_t& t);

	int getCameraId() const;
	void getCameraMat(cv::Mat& t) const;
	void getDistCoeff(cv::Mat& t) const;
	double getArucoSize() const;

private:
	cv::Ptr<cv::aruco::Dictionary> dictionary_;
	Eigen::Vector3d B_p_BC_; // Body to Camera in Body frame
	Eigen::Quaterniond q_BC_; // Camera orientation w.r.t. body frame

	int CameraID_;

	// Camera properties
	cv::Mat cameraMatrix_;
	cv::Mat distCoeffs_;

	double ArucoSize_;

	DetectionData detection_data_;

	std::unordered_map<int, BodyTarget> targets_;

	int detect(const cv::Mat& image, DetectionData& dd);

	int solvePnP(DetectionData& dd);
	
	int generateMap(const DetectionData& dd, 
            std::unordered_map<int, BodyTarget>& outMap);

};


#endif
