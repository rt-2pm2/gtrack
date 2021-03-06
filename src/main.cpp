#include <unistd.h>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <stdio.h>
#include <fstream>
#include <unordered_map>
#include <jsoncpp/json/json.h>
#include <chrono>

#include "arucodetector/arucodetector.hpp"
#include "devinterface/devinterface.hpp"
#include "rstracker/rstracker.hpp"
#include "gatlas/gatlas.hpp"

#include "filter/ddfilter.hpp"
#include "utils/timelib.hpp"
#include "utils/utils.hpp"

#include <dirent.h>

using namespace std;
using namespace mmtracker;

int selected_vehicle = 3;

bool should_end = false;

int _WIDTH = 848;
int _HEIGHT = 480;
int _FPS = 30;
double _DEPTH_SCALE = 0.0001;

static int name_filter(const struct dirent* dir_ent) {
    if (!strcmp(dir_ent->d_name, ".") || !strcmp(dir_ent->d_name, "..")) return 0;
    std::string fname = dir_ent->d_name;

    if (fname.find(".bag") == std::string::npos) return 0;

    return 1;
}

int search_bags(const std::string path, std::vector<std::string>& v_names) {
	const char *dirp = path.c_str();
	struct dirent **namelist;
	int nfiles = scandir(dirp, &namelist, *name_filter, alphasort);

	v_names.clear();
    for (int i = 0; i < nfiles; i++) {
        std::string fname = namelist[i]->d_name;
        v_names.push_back(fname);
        free(namelist[i]);
    }
    free(namelist);

	return nfiles;
}


int main(int argc, char* argv[]) {
	int key = 49;

	double dt;	

	std::string filename("rec_");	
	std::string config_file("config.json");	
	std::ofstream _outfile("global.csv");
	
	//std::string ip("127.0.0.1");
	std::string ip {};

	bool playback = false;
	bool recording = false;
	bool rem_server = false;
	int operation = RSTRK_ONLINE;
	int option;

	/**
	 * Parse options
	 * - 'p': playback
	 */
	while ((option = getopt(argc, argv, "rpf:s:")) != -1) {
		switch (option) {
			case 'r': // Recording
				recording = true;
				playback = false;
				operation = RSTRK_RECORDING;
				break;
			case 'p': // Activate playback
				playback = true;
				recording = false;
				operation = RSTRK_PLAYBACK;
				break;
			case 'f': // Select file
				filename = std::string(optarg);
				cout << "Filename : " << filename << endl;
				break;
			case 's': // Server
				ip = optarg;
				rem_server = true;
				cout << "Atlas Server @: " << optarg << endl;
		}
	}

	if (playback) { cout << " ==== Playback Mode ==== " << endl; }
	if (recording) { cout << "++++ Recording Active ++++ " << endl; }

	/**
	 * If external Atlas server required
	 */
	GAtlas* wmap;
	if (rem_server) {
		wmap = new GAtlas(ip, 8080);
	} else {
		wmap = new GAtlas();
	}

	if (file_exist(config_file)) {
		cout << "Found configuration file!" << endl;
		bool isParsable = false;
		Json::CharReaderBuilder builder;
		Json::Value obj;
		JSONCPP_STRING errs;
		ifstream ifs(config_file);
		isParsable = Json::parseFromStream(builder, ifs,
				&obj, &errs);
		if (isParsable) {
			cout << "Parsing..." << endl;
			_WIDTH = obj["width"].asUInt();
			_HEIGHT = obj["height"].asUInt();
			_FPS = obj["fps"].asUInt();
			_DEPTH_SCALE = obj["depth_scale"].asDouble();
		} else {
			cout << "Error parsing..." << endl;
		}
	} else {
		cout << "No config, using default." << endl;
	}

	timespec t_now, t_old;
	clock_gettime(CLOCK_MONOTONIC, &t_now);

	// =========================================
	// Realsense Device	
	//
	DevConfig devcfg;
	devcfg.depth_width = _WIDTH;
	devcfg.depth_height = _HEIGHT;
	devcfg.depth_framerate = _FPS;
	devcfg.rgb_width = _WIDTH;
	devcfg.rgb_height = _HEIGHT;
	devcfg.rgb_framerate = _FPS;
	
	// Check the connected devices
	rs2::context ctx;
	std::vector<DeviceInterface*> mydevs;
	std::vector<RSTracker*> ptrackers;

	/*
	 * For each device create the device interface class and pass it to the 
	 * constructor of the RSTracker which host all the other components.
	 * I wanted to ecapsulate the other components in the RSTracker because
	 * I think they are all related to the same task. Anyway, I preferred to 
	 * use just references in the RSTracker because the components can be 
	 * independet for testing/reusing purposes.
	 *
	 * I keep a reference to the devices to easily fetch images for the UI.
	 */

	cout << " ============================= " << endl;
	cout << " <-.->     Starting      <-.-> " << endl;
	cout << endl;

	if (operation == RSTRK_PLAYBACK) {
		std::vector<std::string> v_names;
		search_bags("./", v_names);
		
		for (auto bagfile : v_names) {
			cout << "Adding Device..." << endl;
			DeviceInterface* pdev = new DeviceInterface();
			RSTracker* ptrk = new RSTracker(pdev);
			ptrk->addWorldMap(wmap);

			// Add to the vector of device and trackers
			mydevs.push_back(pdev);
			ptrackers.push_back(ptrk);	

			// Start the acquisitioa(not the tracking)
			ptrk->start_device(operation, bagfile);
			ptrk->start_tracking();
			ptrk->start_flow();
		}
	} else {
		auto devlist = ctx.query_devices();
		if (devlist.size() == 0) {
			cerr << "No devices connected!" << endl;
			return -1;
		}
		for (auto&& dev : devlist) {
			cout << "Adding Device..." << endl;
			DeviceInterface* pdev = new DeviceInterface(devcfg, ctx, dev);
			RSTracker* ptrk = new RSTracker(pdev);
			ptrk->addWorldMap(wmap);

			// Add to the vector of device and trackers
			mydevs.push_back(pdev);
			ptrackers.push_back(ptrk);	

			// Start the acquisitioa(not the tracking)
			ptrk->start_device(operation, filename);

			// Start the tracking and detection
			ptrk->start_tracking();
			ptrk->start_flow();
		}
	}

	cv::Mat cvRGB, cvDepthCol, cvFrame;
	std::string title;
	
	cv::VideoWriter oWriter_rgb("./output_rgb.avi",
			cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30,
			cv::Size(devcfg.depth_width, devcfg.depth_height), true);

	// =========================================
	// Opencv Windows 
	cv::namedWindow("Monitor", cv::WINDOW_AUTOSIZE + cv::WINDOW_OPENGL);
	cv::moveWindow("Monitor", 0, 0);

#ifdef HIST_DEBUG
	cv::namedWindow("Hist", cv::WINDOW_AUTOSIZE + cv::WINDOW_OPENGL);
	cv::moveWindow("Hist", 1280, 200);
#endif

#ifdef MASK_DEBUG
	cv::namedWindow("Mask", cv::WINDOW_AUTOSIZE + cv::WINDOW_OPENGL);
	cv::moveWindow("Mask", 1280, 0);

	cv::namedWindow("Flow", cv::WINDOW_AUTOSIZE + cv::WINDOW_OPENGL);
	cv::moveWindow("Flow", 1280, 800);
#endif

	clock_gettime(CLOCK_MONOTONIC, &t_now);
	t_old = t_now;

	int img_type = 1;
	int dev_select = 0;
	DeviceInterface* mydev = mydevs[0];
	MMTracker* trk = ptrackers[0]->getMMTracker();
	DDFilter* ddfil = ptrackers[0]->getDDFilter();

	int num_devices = ptrackers.size(); 

	while (key != 27) {
		int curr_key = cv::waitKey(1);

		// Poll from the queue of processed frames
		if (curr_key == 27)
			break;

		if (curr_key != 255 && curr_key != -1) {
			std::cout << "Changing to " << curr_key << std::endl;
			key = curr_key;
		}

		// Select the device and image type
		switch (key) {
			case 49:
				title = "Device 0";
				dev_select = 0;
				mydev = mydevs[dev_select];
				trk = ptrackers[dev_select]->getMMTracker();
				ddfil = ptrackers[dev_select]->getDDFilter();
				break;
			case 50:
				if (num_devices > 1) {
					title = "Device 1";
					dev_select = 1;
					mydev = mydevs[dev_select];
					trk = ptrackers[dev_select]->getMMTracker();
					ddfil = ptrackers[dev_select]->getDDFilter();
				}
				break;
			case 51:
				if (num_devices > 2) {
					title = "Device 2";
					dev_select = 2;
					mydev = mydevs[dev_select];
					trk = ptrackers[dev_select]->getMMTracker();
					ddfil = ptrackers[dev_select]->getDDFilter();
				}
				break;
			case 52:
				img_type = 1;
				break;
			case 53:
				img_type = 2;
				break;
			default:
				title = "Device 0";
				mydev = mydevs[0];
				trk = ptrackers[0]->getMMTracker();
				ddfil = ptrackers[0]->getDDFilter();
				break;
		}

		// Wait for new data from the selected device to arrive
		mydev->synchronize();	
		clock_gettime(CLOCK_MONOTONIC, &t_now);
		dt = (timespec2micro(&t_now) - timespec2micro(&t_old)) / 1e6;
		t_old = t_now;
		
		// Get the RGB image
		mydev->get_rgb(cvRGB);
		// Get the Depth information (measure + visualization map)
		mydev->get_depth_col(cvDepthCol);
		
		switch (img_type) {
			case 1:
				title = "RGB";
				cvRGB.copyTo(cvFrame);
				break;
			case 2:
				title = "Depth";
				cvDepthCol.copyTo(cvFrame);
				break;
			default:
				title = "RGB";
				cvRGB.copyTo(cvFrame);
				break;
		}

		if (!cvFrame.empty()) {
#ifdef ARUCO_DEBUG
			cv::Mat cmat, ddsf;
			mydev->getCameraParam(cmat, ddsf);
			DetectionData ddata = 
				ptrackers[dev_select]->getArucoDetection();

			cv::aruco::drawDetectedMarkers(cvFrame,
					ddata.mk_corners_,
					ddata.mk_ids_);

			for (int i = 0; i < ddata.rvecs_.size(); i++) {
				cv::aruco::drawAxis(cvFrame,
						cmat, ddsf,
						ddata.rvecs_[i],
						ddata.tvecs_[i],
						0.1);
			}
#endif
			cv::Point tg;
			trk->get_img_tg(selected_vehicle, tg);

			cv::Rect2d roi;
			trk->get_ROI(selected_vehicle, roi);

			Eigen::Vector3d pos;
			trk->get_b_tg(selected_vehicle, pos);
			Eigen::Vector3d pos_(pos[0], pos[1], pos[2]);

			/*
			Eigen::Vector3d est_p = ddfil->getPos();
			Eigen::Vector3d est_v = ddfil->getVel();
			*/

#ifdef MASK_DEBUG
			cv::Mat mask;
			trk->get_mask(selected_vehicle, mask);
			if (!mask.empty())
				cv::imshow("Mask", mask);

			cv::Mat fl_mask;
			trk->get_flowmask(fl_mask);
			if (!fl_mask.empty())
				imshow("Flow", fl_mask);
#endif

#ifdef HIST_DEBUG
			cv::Mat hist_img;
			cv::Mat depth_roi;
			std::array<int, 3> v;
			trk->get_depthROI(selected_vehicle, depth_roi);
			trk->get_depthTG(selected_vehicle, v);
			int numBins = 200;	
			if (!depth_roi.empty()) {
				trk->get_histogram(hist_img, numBins, depth_roi, v[2]);
				cv::imshow("Hist", hist_img);
			}
#endif

			// =====================================================
			// Visualization =======================================
			/*
			cv::Point kf_pt;
			cv::Point dd_pt;
			trk->position2pixel(dd_pt, est_p);
			*/

			//cv::circle(cvFrame, dd_pt, 10, cv::Scalar(0, 0, 255), 2);
			cv::circle(cvFrame, tg, 15, cv::Scalar(0, 255, 0), 2);
			cv::rectangle(cvFrame, roi, cv::Scalar(0, 0, 255), 2, 1);
			//cout << "Main DT = " << dt << endl;
			cv::imshow("Monitor", cvFrame);
			oWriter_rgb.write(cvFrame);

		/*	
			static bool initroi = true;
			if (initroi) {
				initroi = false;
				cv::Rect2d roi;
				mydev.playbackPause();
				roi = cv::selectROI("Monitor", cvFrame);
				mydev.playbackResume();
				trk.add_target(0, roi);
			}
		*/	
		}

	}
	
	should_end = true;
	oWriter_rgb.release();
	_outfile.close();
	cout << "Ending..." << endl;

	for (auto el : ptrackers) {
		el->stop_tracking();
	}

	cout << "Terminating..." << endl;
	cv::waitKey(5000);

	return 0;
}
