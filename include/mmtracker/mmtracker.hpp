/**
 * \file tracker.hpp
 * \author Luigi
 *
 * This class models the Target Tracker using the depth camera 
 * data.
 *
 * The tracker is fed with data with a "step" function which 
 * takes as arguments rgb and depth data.
 * There is also a thread for the computation of the optical flow.
 * The idea is to use the optical flow to lock new targets or to 
 * recover the tracking in case the other method fail.
 *
 */
#ifndef _TRACKER_HPP_
#define _TRACKER_HPP_

#include <librealsense2/rsutil.h> 
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/tracking/tracker.hpp>

#include <unordered_map>
#include <thread>
#include <mutex>
#include <fstream>

#include "gatlas/gatlas.hpp"

namespace mmtracker {
/**
 * This data structure represent the target
 * tracked by a single device. For this reason the information
 * is with respect to sensor.
 */
struct TargetData {
	/**
	 * ID
	 */
	int id;

	/**
	 * Pixel coordinate of the target.
	 */
	cv::Point img_target;

	/**
	 * Coordinates of the target in camera frame
	 * [m].
	 */
	Eigen::Vector3d b_tg;

	/**
	 * Intermediate target information:
	 * - (u,v) coordinates in image frame
	 * - depth of that pixel [m].
	 */
	std::array<int, 3> depth_tg; 

	/**
	 * Distribution of the target depth
	 * representation.
	 */
	std::array<int, 3> depth_tg_std;
	
	/**
	 * Depth Mask selecting the part of the 
	 * image with the target depth.
	 */
	cv::Mat depth_mask;

	/**
	 * Image moments of the Depth Mask.
	 */
	cv::Moments depth_mask_moments;

	/**
	 * Region Of Interest locating the target.
	 */
	cv::Rect2d roi;

	/**
	 * Part of the image RGB containing the 
	 * target.
	 */
	cv::Mat rgb_roi;

	/**
	 * Part of the depth image containing the
	 * target.
	 */
	cv::Mat depth_roi;
};


class MMTracker {
	public:
		MMTracker();
		MMTracker(std::string name);
		~MMTracker();

		void set_transform(const Eigen::Vector3d& t,
				const Eigen::Quaterniond& q);

		void add_target(int id, cv::Rect2d roi);

		void set_roi(int id, int x, int y, int w, int h);

		void set_position(int id, Eigen::Vector3d& pos);

		void position2pixel(cv::Point& pt, const Eigen::Vector3d& pos);

		/**
		 * Estimation step.
		 * Takes the color image, the depth image and the
		 * region of interest where the target is located
		 */
		int step(cv::Mat& rgb, cv::Mat& depth);

		/**
		 * Optical flow step.
		 * Compute the optical flow using the current
		 * frame passed to the function and information
		 * of the previous step.
		 */
		void optical_flow_step(cv::Mat& cvFrame,
				std::vector<TargetData>& untracked);

		void set_delta_depth_param(double d);

		void set_flow_thr(double thr, double norm_thr);

		void setCamMatrix(rs2_intrinsics intr);

		void setDepthScale(double scale);

		// GETTERS
		bool get_ROI(int i, cv::Rect2d& roi);

		bool get_img_tg(int i, cv::Point& tg);

		bool get_b_tg(int i, Eigen::Vector3d& tg);

		bool get_mask(int i, cv::Mat& m);

		bool get_depthROI(int i, cv::Mat& m);

		bool get_depthTG(int i, std::array<int, 3>& tg);

		bool get_rgbROI(int i, cv::Mat& m); 

		bool get_flowmask(cv::Mat& m);

		void get_histogram(cv::Mat& hist_img, int numBins,
				const cv::Mat& depth_roi, int target_depth);

		/**
		 * Get the list of targets tracked by the camera
		 */
		int get_targets(std::vector<TargetData>& T);


		std::string _mmtracker_name;
	private:

		void ResetMMParam();


		/**
		 * Transformation world to camera
		 */
		Eigen::Vector3d C_p_CM_;
		Eigen::Quaterniond q_CM_;

		/**
		 * Opencv Tracker class
		 */
		cv::Ptr<cv::Tracker> opt_tracker;

		/**
		 * Max number of targets for the optical flow analysis.
		 */
		int _MaxTargets;

		/**
		 * Log file
		 */
		std::ofstream _log_flow;
		std::ofstream _log_trk;
		std::ofstream _log_debug_trk;


		/**
		 * Mutex for shared data
		 */
		std::mutex _mx;

		/**
		 * RGB Image
		 */
		cv::Mat cvFrame;
		
		/**
		 * Depth Image
		 */
		cv::Mat cvDepth;

		int _frame_height;
		int _frame_width;

		/**
		 * Map of detected targets
		 */
		std::unordered_map<int, TargetData*> _targets;

		/**
		 * Region Of Interest reprensenting the area of the image
		 * where the target is supposed to be.
		 */
		cv::Mat _flow_mask;

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
		int find_target_depth(double* tg, double* tg_std,
				const cv::Mat& depth, int attempts, double err);

		bool find_target_in_roi(TargetData* tg_data,
				int ks, int attempts);

		void find_target_in_roi(Eigen::Vector3d& b_tg,
				const cv::Rect2d& roi);
	
		/**
		 * Delta Depth used to select the pixel 
		 * associated to the target in the depth image.
		 */
		double _delta_depth_param;

		
		/**
		 * Instogram tools
		 */
		void computeHist(cv::Mat& hist, int numBins,
				const cv::Mat& data, double scale);

		/**
		 * Thread
		bool _flow_thread_active;
		bool _flowActive;
		std::thread _flow_thread;
		void opticalflow_runnable();

		 */

		// Optical Flow Parameters
		/**
		 * Under this value the flow is considered zero
		 */
		double _min_flow_threshold;

		/**
		 * Threshold applied to the normalized flow 
		 */
		double _min_flow_threshold_norm;

		/**
		 * The optical flow is computed on a scaled image for reducing
		 * the computation load. '_opt_flow_scale' is the resize factor.
		 */
		double _opt_flow_scale;

		/**
		 * Period of the task computing the optical flow.
		 */
		timespec opt_flow_period;

		/**
		 * The detection of a new target from the optical flow is made
		 * considering the clustering of the thresholded normalized 
		 * flow. The idea is to consider a region a possible target
		 * when the area is bigger than a given amount.
		 * '_opt_flow_detect_thr' is that amount.
		 */
		double _opt_flow_detect_thr;


		// For the optical flow
		cv::Mat magnitude, angle, magn_norm;
		cv::Mat cvPrev;
		cv::Mat bgr;

		double pyr_scale;
		int levels;
		int winsize;
		int niters;
		int poly_n;
		double poly_sigma;
		int flags;
};

}
#endif
