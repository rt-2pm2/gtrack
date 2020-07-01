/**
 * \file devinterface.hpp
 * \author Luigi
 *
 * \brief
 * Class managing the interface with the realsense.
 *
 * The idea is to conceal the realsense interface with the
 * pipelines and expose the CV matrices for the downstream
 * elaboration.
 *
 */
#ifndef _DEVINTERFACE_HPP_
#define _DEVINTERFACE_HPP_

#define QUEUE_SIZE (2)

#include <thread>
#include <condition_variable>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include "utils/camdata.hpp"


/**
 * Data structure which contain the configuration for the streams.
 * This structure is used when constructing the class.
 */
struct DevConfig {
	bool playback;

	int depth_width;
	int depth_height;
	int depth_framerate;
	float depth_scale;

	int rgb_width;
	int rgb_height;
	int rgb_framerate;

	rs2_format depth_format;
	rs2_format rgb_format;

	DevConfig() {
		playback = false;
		depth_width = 1280;
		depth_height = 720;
		depth_scale = 0.0001;
		depth_framerate = 30;

		rgb_width = 1280;
		rgb_height = 720;
		rgb_framerate = 30;

		depth_format = RS2_FORMAT_Z16;
		rgb_format = RS2_FORMAT_RGB8;
	}

	DevConfig(const DevConfig& dc) {
		playback = dc.playback;
		depth_width = dc.depth_width;
		depth_height = dc.depth_height;
		depth_scale = dc.depth_scale;
		depth_framerate = dc.depth_framerate;

		rgb_width = dc.rgb_width;
		rgb_height = dc.rgb_height;
		rgb_framerate = dc.rgb_framerate;

		depth_format = dc.depth_format;
		rgb_format = dc.rgb_format;
	}
};


class DeviceInterface {
	public:
		/**
		 * Default Constructor
		 */
		DeviceInterface();

		/**
		 * Constructor with Configuration Struct
		 */
		DeviceInterface(const DevConfig& cf);

		/**
		 * Default destructor
		 */
		~DeviceInterface();


		// ==============================================
		// User Methods

		/**
		 * Start the device pipeline and finish the configuration
		 * of the streams.
		 */
		bool startDevice(bool playback, std::string bag_name);

		/**
		 * Stop
		 */
		void stopDevice(bool wait);


		/**
		 * Pause
		 */
		void playbackPause();

		/**
		 * Resume
		 */
		void playbackResume();

		/**
		 * Get the parameter of the camera
		 */
		bool getCameraParam(cv::Mat& cameraMat, cv::Mat& coeff);
		bool getCameraParam(rs2_intrinsics& intr);

		/**
		 * Get images
		 */
		void get_rgb(cv::Mat& m);
		void get_depth(cv::Mat& m);
		void get_depth_col(cv::Mat& m);

		bool synchronize();

	private:
		/**
		 * Flag to indicate that the basic stream info
		 * has been set.
		 */
		bool init;

		/**
		 * Flag to indicate the activation status of the pipeline
		 */
		bool active;

		/**
		 * Playback status
		 */
		bool playback;

		/**
		 * Realsense device Pipeline
		 */
		rs2::pipeline _pipeline;

		/**
		 * Realsense device profile
		 */
		rs2::pipeline_profile profile;

		/**
		 * Pipeline Config class
		 */
		rs2::config _cfg;

		/**
		 * Camera intrisic parameter
		 */
		rs2_intrinsics camera_intr;

		/**
		 * Queue for the raw data
		 */
		rs2::frame_queue* input_queue;

		/**
		 * Queue of filtered data
		 */
		rs2::frame_queue* filt_queue;

		/** 
		 * Device Configuration Structure
		 */
		DevConfig dev_cfg;

		/**
		 * Data
		 */
		CamData data;

		/** Working Threads
		 *
		 * In order to get the data, there are 
		 * 2 working threads.
		 * The first thread, takes data from the device and
		 * perform basic filtering.
		 * A second thread takes the filtered data and 
		 * convert it into cv::Mat format.
		 */
		std::thread input_thread;
		std::thread conversion_thread;

		/**
		 * Functions
		 */
		void inflow();
		void filtering();

		/**
		 * Condition Variable
		 */
		std::condition_variable cv_synch;
		std::mutex mx_synch; 
		bool updated;

		void signal_new();

};
#endif
