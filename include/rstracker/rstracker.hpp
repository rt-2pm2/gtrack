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

#include "devinterface/devinterface.hpp"
#include "mmtracker/mmtracker.hpp"
#include "arucodetector/arucodetector.hpp"
#include "filter/ddfilter.hpp"
#include "utils/shared_map.hpp"

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
		void addWorldMap(SharedMap* map);


		/**
		 * Get Ref to inner classes...dirty and temporary
		 */
		DDFilter* getDDFilter();
		MMTracker* getMMTracker();
		DetectionData& getArucoDetection();

		/**
		 * Start tracking thread
		 */
		bool start_tracking();

		/**
		 * Stop tracking
		 */
		bool stop_tracking();

	private:

		/**
		 * Ready flag
		 */
		bool _ready;

		/**
		 * Tracker class: it has the basic functionalities to take 
		 * images and return a measurement/estimation of the 
		 * target position.
		 */
		MMTracker* _ptrk;

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
		SharedMap* _world_map;

		/**
		 * Runnable with the tracking routine
		 */
		void track_runnable();

		/**
		 * Autosetting routine
		 */
		void autoSetWorldReference();

		std::thread _trk_thread;

		bool _shutting_down;

		cv::Mat _cmat; 
		cv::Mat _ddsf;

		std::ofstream _outfile;
};

#endif