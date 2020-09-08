/**
 * \file rstracker.hpp
 * This class warps together the components
 * to perform tracking using the realsense
 * camera.
 */

#ifndef _RSTRACKER_HPP_
#define _RSTRACKER_HPP_

#include <unordered_map>
#include <fstream>
#include <opencv2/core.hpp>
#include <thread>
#include <mutex>

#include "devinterface/devinterface.hpp"
#include "mmtracker/mmtracker.hpp"
#include "arucodetector/arucodetector.hpp"
#include "filter/ddfilter.hpp"
#include "gatlas/gatlas.hpp"
#include <time.h>

class RSTracker {
	public:
		RSTracker(DeviceInterface* pdev);
		~RSTracker();

		/**
		 * Start RealSense device
		 */
		void start_device(int operation, std::string fname);


		/**
		 * Add World Map
		 */
		void addWorldMap(GAtlas* map);


		/**
		 * Get Ref to inner classes...dirty and temporary
		 */
		DDFilter* getDDFilter();
		mmtracker::MMTracker* getMMTracker();
		DetectionData getArucoDetection();

		/**
		 * Start tracking thread
		 */
		bool start_tracking();

		/**
		 * Stop tracking
		 */
		void stop_tracking();

		/**
		 * Start Flow Thread
		 */
		void start_flow();

		/**
		 * Stop Flow Thread
		 */
		void stop_flow();

		/**
		 * Check if the tracker is set up with the arucos
		 */
		bool isReady();

	private:
		std::mutex mx;

		/**
		 * Id of the reference aruco
		 */
		int _local_aruco_id;
		int _global_aruco_ref;

		/**
		 * Pose of the sensor with respect to the global
		 * reference.
		 */
		Eigen::Vector3d _w_p_wc;
		Eigen::Quaterniond _w_q_c;

		/**
		 * Ready flag
		 */
		bool _ready;

		/**
		 * Tracker class: it has the basic functionalities to take 
		 * images and return a measurement/estimation of the 
		 * target position.
		 */
		mmtracker::MMTracker* _ptrk;

		/**
		 * Device class
		 */
		DeviceInterface* _pdev;

		/**
		 * Filter class
		 */
		DDFilter* _pfilt;

		/**
		 * Aruco detector class
		 */
		ArucoDetector* _paruco_detector;	

		/**
		 * Map of the detected aruco marker
		 */
		std::unordered_map<int, BodyTarget> _aruco_map;

		/**
		 * Aruco detection data
		 */
		DetectionData _detect_data;

		/**
		 * World Map reference
		 */
		GAtlas* _gatlas;


		/**
		 * Synch Function
		 */
		bool wait_ready();

		/**
		 * Runnable with the tracking routine
		 */
		void track_runnable();

		/**
		 * Autosetting routine
		 */
		void autoSetWorldReference();

		/**
		 * Thread
		 */
		bool _flow_thread_active;
		bool _flowActive;	
		void opticalflow_runnable();
		std::thread _flow_thread;
		timespec opt_flow_period;

		std::thread _trk_thread;

		std::thread _atlas_thread;

		bool _shutting_down;

		cv::Mat _cmat; 
		cv::Mat _ddsf;

		std::ofstream _outfile;
		std::ofstream _logfile;
};

#endif
