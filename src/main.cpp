#include <unistd.h>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <stdio.h>
#include <fstream>
#include <unordered_map>

#include "arucodetector/arucodetector.hpp"
#include "devinterface/devinterface.hpp"
#include "tracker/tracker.hpp"
#include "filter/ddfilter.hpp"
#include "utils/timelib.hpp"

#define _WIDTH (848)
#define _HEIGHT (420)

#define ARUCO_DEBUG

using namespace std;

std::unordered_map<int, BodyTarget> aruco_map;


int main(int argc, char* argv[]) {
	int key = 49;

	ofstream outfile("est.csv");

	std::string filename("record_");	

	bool playback = true;
	int option;

	double sx = 10.0;
	double sy = 0.001;

	while ((option = getopt(argc, argv, ":p:f:c:")) != -1) {
		switch (option) {
			//For option i, r, l, print that these are options
			case 'p':
				if (atoi(optarg) == 1) {
					cout << "Playback" << endl;
					playback = true;
				} else {
					cout << "Recording" << endl;
					playback = false;
				}
				break;
			case 'f': //here f is used for some file name		
				filename = std::string(optarg);
				cout << "Filename : " << filename << endl;
				break;
			case 'c':
				sy = atof(optarg);
				cout << "PolyFilter x std: " << sy << endl;
		}
	}

	outfile << "Time" << " ";
	outfile << "w_px w_py w_pz" << " ";
	outfile << "b_px b_py b_pz" << " ";
	outfile << "b_vx b_vy b_vz" << " ";
	outfile << "b_ax b_ay b_az" << std::endl;

	cout << "Starting..." << endl;
	std::string full = std::string("data/") + filename+std::string(".bag");
	cout  << full << endl;

	// =========================================
	// Realsense Device	
	//

	DevConfig devcfg;
	devcfg.depth_width = 848;
	devcfg.depth_height = 480;
	devcfg.depth_framerate = 30;
	devcfg.rgb_width = 848;
	devcfg.rgb_height = 480;
	devcfg.rgb_framerate = 30;

	
	cout << "Creating the device" << endl;
	DeviceInterface mydev(devcfg);
	mydev.startDevice(playback, full);

	rs2_intrinsics intr;
	mydev.getCameraParam(intr);

	cv::Mat cmat; 
	cv::Mat ddsf;
	mydev.getCameraParam(cmat, ddsf);

	cv::Mat cvRGB, cvDepth, cvDepthCol, cvFrame;
	std::string title;
	
	cv::VideoWriter oWriter_rgb("./output_rgb.avi",
			cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10,
			cv::Size(devcfg.depth_width, devcfg.depth_height), true);

	// =========================================
	// Aruco Detector
	// Import the camera intrisic from the realsense
	ArucoDetector aruco_detector(cmat, ddsf, 0.10, 0);

	// =========================================
	// Tracker
	Tracker trk;

	//trk.add_target(0, cv::Rect2d(1100, 450, 100, 100));
	trk.setCamMatrix(intr, 0.0001);

	// =========================================
	// Filters
	Eigen::Vector3d p0(0.0, 0.0, 0.0);
	double dt = 0.03;
	DDFilter ddfil(p0, 20, dt);


	// =========================================
	// Opencv Windows 
	cv::namedWindow("Monitor", cv::WINDOW_AUTOSIZE);
	cv::moveWindow("Monitor", 0, 0);

	cv::namedWindow("Mask", cv::WINDOW_AUTOSIZE);
	cv::moveWindow("Mask", 1280, 0);

	cv::namedWindow("Hist", cv::WINDOW_AUTOSIZE);
	cv::moveWindow("Hist", 1280, 200);

	cv::namedWindow("Flow", cv::WINDOW_AUTOSIZE);
	cv::moveWindow("Flow", 1280, 800);


	int InitializationCounter = 20;
	int CameraStabCounter = 40;

	std::cout << "Activating Flow Thread!" << std::endl;
	trk.start_flow();

	timespec t_now, t_old;
	clock_gettime(CLOCK_MONOTONIC, &t_now);
	t_old = t_now;
	while (key != 27) {
		int curr_key = cv::waitKey(1);

		// Poll from the queue of processed frames
		if (curr_key == 27)
			break;

		if (curr_key != 255 && curr_key != -1) {
			std::cout << "Changing to " << curr_key << std::endl;
			key = curr_key;
		}

		// Wait for new data to arrive
		mydev.synchronize();	
		clock_gettime(CLOCK_MONOTONIC, &t_now);
		dt = (timespec2micro(&t_now) - timespec2micro(&t_old)) / 1e6;
		// Avoid too little 'dt'
		if (dt < 0.0001)
			dt = 0.001;
		t_old = t_now;
		
		// Get the RGB image
		mydev.get_rgb(cvRGB);

		// Get the Depth information (measure + visualization map)
		mydev.get_depth(cvDepth);
		mydev.get_depth_col(cvDepthCol);

		switch (key) {
			case 49:
				title = "RGB";
				cvRGB.copyTo(cvFrame);
				break;
			case 50:
				title = "Depth";
				cvDepthCol.copyTo(cvFrame);
				break;
			default:
				title = "RGB";
				cvRGB.copyTo(cvFrame);
				break;
		}	

		if (CameraStabCounter > 0) {
			CameraStabCounter--;
		}

		double depth_target = 0;

		if (!cvFrame.empty() && !cvRGB.empty() && !cvDepth.empty()) {
			cv::Mat outputImage = cvFrame.clone();

			if (playback && CameraStabCounter == 0) {
				trk.step(cvRGB, cvDepth);

				cv::Point tg;
				cv::Mat mask;
				cv::Rect2d roi;
				std::array<int, 3> v;
				std::array<float, 3> pos;

				trk.get_mask(0, mask);
				trk.get_img_tg(0, tg);
				trk.get_depthTG(0, v);
				trk.get_ROI(0, roi);
				trk.get_b_tg(0, pos);

				if (!mask.empty())
					cv::imshow("Mask", mask);

				// Filter update
				Eigen::Vector3d pos_;
				pos_(0) = pos[0];
				pos_(1) = pos[1];
				pos_(2) = pos[2];

				ddfil.prediction(dt);
				ddfil.update(pos_);

				Eigen::Vector3d est_p = ddfil.getPos();
				Eigen::Vector3d est_v = ddfil.getVel();
				
				if (InitializationCounter == 0){
					//trk.set_position(0, est_p);
				}

				if (InitializationCounter > 0) {
					InitializationCounter--;

					int Ndetected = 0;
					DetectionData detect_data;

					if (!cvFrame.empty()) {
						Ndetected = aruco_detector.processImage(
								aruco_map, detect_data, cvFrame, 0);
					}

#ifdef ARUCO_DEBUG
					if (Ndetected > 0) {
						cv::aruco::drawDetectedMarkers(outputImage,
								detect_data.mk_corners_,
								detect_data.mk_ids_);

						cv::aruco::drawAxis(outputImage,
								cmat, ddsf,
								detect_data.rvecs_, detect_data.tvecs_,
								0.1);
					}
#endif
				}

				// Compute the position of the target w.r.t the World Frame
				Eigen::Vector3d W_t = (aruco_map[1].q_CM_.inverse() * (pos_ - aruco_map[1].C_p_CM_));

				outfile << timespec2micro(&t_now) << " ";
				outfile << W_t(0) << " " << W_t(1) << " " << W_t(2) << " ";
				outfile << est_p(0) << " " << est_p(1) << " " << est_p(2) << " ";
				outfile << est_v(0) << " " << est_v(1) << " " << est_v(2) << " ";
				outfile << aruco_map[1].C_p_CM_(0) << " " <<
					aruco_map[1].C_p_CM_(1) << " " << aruco_map[1].C_p_CM_(2);
				outfile << endl;


				/*
				cv::Mat fl_mask;
				trk.get_flowmask(0, fl_mask);
				if (!fl_mask.empty())
					imshow("Flow", fl_mask);

				cv::Mat hist_img;
				cv::Mat depth_roi;
				trk.get_depthROI(0, depth_roi);
				int numBins = 300;	
				trk.get_histogram(hist_img, numBins, depth_roi, v[2]);

				cv::imshow("Hist", hist_img);
				*/

				// =====================================================
				// Visualization =======================================
				cv::Point kf_pt;
				cv::Point dd_pt;
				trk.position2pixel(dd_pt, est_p);

				cv::circle(outputImage, dd_pt, 10, cv::Scalar(0, 0, 255), 2);
				cv::circle(outputImage, tg, 15, cv::Scalar(0, 255, 0), 2);
				cv::rectangle(outputImage, roi, cv::Scalar(0, 0, 255), 2, 1);
			}
			//cout << " DT = " << dt << endl;
			cv::imshow("Monitor", outputImage);
			oWriter_rgb.write(outputImage);

			/*
			static bool initroi = true;
			if (initroi) {
				initroi = false;
				cv::Rect2d roi;
				mydev.playbackPause();
				roi = cv::selectROI("Monitor", outputImage);
				mydev.playbackResume();
				trk.add_target(0, roi);
			}
			*/
		}


	}
	oWriter_rgb.release();
	outfile.close();
	cout << "Ending..." << endl;
	trk.stop_flow();
	mydev.stopDevice(true);

	return 0;
}
