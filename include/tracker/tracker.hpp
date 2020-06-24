/**
 * \file tracker.hpp
 * \author Luigi
 *
 */
#ifndef _TRACKER_HPP_
#define _TRACKER_HPP_

#include <librealsense2/rsutil.h> 
#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include <unordered_map>
#include <thread>


struct TargetData {
	std::array<float, 3> b_tg;

	std::array<int, 3> depth_tg;
	std::array<int, 3> depth_tg_std;
	
	cv::Mat mask;
	cv::Moments img_tg_moments;

	cv::Rect2d roi;
	cv::Mat tg_mask;
	cv::Mat flow_mask;
	cv::Mat rgb_roi;
	cv::Mat depth_roi;

	cv::Point img_target;
};


class Tracker {
	public:
		Tracker();
		~Tracker();

		void set_roi(int x, int y, int w, int h);

		void set_position(Eigen::Vector3d& pos);

		void position2pixel(cv::Point& pt, const Eigen::Vector3d& pos);

		/**
		 * Estimation step.
		 * Takes the color image, the depth image and the
		 * region of interest where the target is located
		 */
		void step(cv::Mat& rgb, cv::Mat& depth);

		void start_flow();

		void stop_flow();

		void setCamMatrix(rs2_intrinsics intr, double scale);

		void get_ROI(cv::Rect2d& roi);

		void get_img_tg(cv::Point& tg);

		void get_b_tg(std::array<float, 3>& tg);

		void get_mask(cv::Mat& m);

		void get_depthROI(cv::Mat& m);

		void get_depthTG(std::array<int, 3>& tg);

		void get_rgbROI(cv::Mat& m); 

		void get_histogram(cv::Mat& hist_img, int numBins,
				const cv::Mat& depth_roi, int target_depth);

		void get_flowmask(cv::Mat& m);
	private:

		/**
		 * RGB Image
		 */
		cv::Mat cvFrame;

		/**
		 * Map of detected targets
		 */
		std::unordered_map<int, TargetData*> targets;

		/** 
		 * Estimated position of the target in the image
		 * coordinate frame (pixels)
		 */
		cv::Point _img_target;

		/**
		 * Coordinate of the target in body frame (camera frame)
		 */
		std::array<float, 3> _b_target;

		/**
		 * Region Of Interest reprensenting the area of the image
		 * where the target is supposed to be.
		 */
		TargetData _tg_data;

		/**
		 * Intrinsic paramter of the camera necessary to convert
		 * pixel coordinates into real world coordinates.
		 */
		rs2_intrinsics _camera_intr;

		/**
		 * Scale of the depth sensor (m/tick)
		 */
		double _dscale;

		/**
		 * K-means
		 */
		double computeKmeans(double* tg, double* tg_std,
				const cv::Mat& depth,
				int ks, int attempts, double err);

		double findTarget_Kmeans(std::array<float, 3>& b_target,
				cv::Point& img_target, const cv::Mat& depth,
				const cv::Rect2d& roi, int ks, int attemps);

		void find_target_in_roi(TargetData& tg_data,
				int ks, int attempts);
		
		/**
		 * Instogram tools
		 */
		void computeHist(cv::Mat& hist, int numBins,
				const cv::Mat& data, double scale);

		/**
		 * Thread
		 */
		bool _flow_thread_active;
		bool _flowActive;
		std::thread _flow_thread;
		void opticalflow_runnable(); 	
};

#endif
