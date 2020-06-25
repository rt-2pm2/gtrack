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
#include "filter/polyfilter.hpp"
#include "filter/ddfilter.hpp"

using namespace std;

std::unordered_map<int, BodyTarget> aruco_map;


int main(int argc, char* argv[]) {
	int key = 49;

	ofstream outfile("est.csv");

	std::string filename("record");

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


	cout << "Starting..." << endl;
	std::string full = std::string("data/") + filename+std::string(".bag");
	cout  << full << endl;

	// =========================================
	// Realsense Device	
	cout << "Creating the device" << endl;
	DeviceInterface mydev;
	mydev.startDevice(playback, full);

	rs2_intrinsics intr;
	mydev.getCameraParam(intr);

	cv::Mat cmat; 
	cv::Mat ddsf;
	mydev.getCameraParam(cmat, ddsf);

	cv::Mat cvRGB, cvDepth, cvDepthCol, cvFrame;
	std::string title;
	
	// =========================================
	// Aruco Detector
	// Import the camera intrisic from the realsense
	ArucoDetector aruco_detector(cmat, ddsf, 0.10, 0);

	// =========================================
	// Tracker
	Tracker trk;

	trk.add_target(0, cv::Rect2d(500, 150, 70, 75));
	trk.setCamMatrix(intr, 0.0001);

	// =========================================
	// Filters
	Eigen::Vector3d p0(0.0, 0.0, 0.0);
	Eigen::Vector3d sigma_x(sx, sx, sx);
	Eigen::Vector3d sigma_y(sy, sy, sy);
	double dt = 0.03;
	PolyFilter polyfil(p0, sigma_x, sigma_y, dt);
	DDFilter ddfil(p0, 20, dt);


	// =========================================
	// Opencv Windows 
	cv::namedWindow("Monitor", cv::WINDOW_AUTOSIZE);
	cv::moveWindow("Monitor", 0, 0);

	cv::namedWindow("Mask", cv::WINDOW_AUTOSIZE);
	cv::moveWindow("Mask", 1280, 0);

	cv::namedWindow("Hist", cv::WINDOW_AUTOSIZE);
	cv::moveWindow("Hist", 1280, 200);

	trk.start_flow();

	int InitializationCounter = 20;
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

		double depth_target = 0;

		if (!cvFrame.empty() && !cvRGB.empty() && !cvDepth.empty()) {
			cv::Mat outputImage = cvFrame.clone();

			if (playback) {
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
				cv::imshow("Mask", mask);

				// Filter update
				Eigen::Vector3d pos_;
				pos_(0) = pos[0];
				pos_(1) = pos[1];
				pos_(2) = pos[2];

				polyfil.prediction(0.03);
				polyfil.update(pos_);

				ddfil.prediction(0.03);
				ddfil.update(pos_);

				Eigen::Vector3d est_pos = polyfil.getPos();
				Eigen::Vector3d est_pos_dd = ddfil.getPos();

				//if (counter++ > 15)
				//	trk.set_position(est_pos_dd);

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
								cameraMatrix, distCoeffs,
								detect_data.rvecs_, detect_data.tvecs_,
								0.1);
					}
#endif
				}

				cv::Mat fl_mask;
				trk.get_flowmask(0, fl_mask);
				if (!fl_mask.empty())
					imshow("Flow", fl_mask); 

				outfile << polyfil.getPos().transpose() << " "; 
				outfile << polyfil.getVel().transpose() << " ";
				//outfile << polyfil.getAcc().transpose();

				outfile << ddfil.getPos().transpose() << " ";
				outfile << ddfil.getVel().transpose() << " ";
				outfile << endl;
				//outfile << ddfil.getAcc().transpose() << endl;

				cv::Mat hist_img;
				cv::Mat depth_roi;
				trk.get_depthROI(0, depth_roi);
				int numBins = 300;	
				trk.get_histogram(hist_img, numBins, depth_roi, v[2]);

				cv::imshow("Hist", hist_img);

				// =====================================================
				// Visualization =======================================
				cv::Point kf_pt;
				cv::Point dd_pt;
				trk.position2pixel(kf_pt, est_pos);
				trk.position2pixel(dd_pt, est_pos_dd);

				cv::circle(outputImage, kf_pt, 10, cv::Scalar(255, 0, 0), 2);
				cv::circle(outputImage, dd_pt, 10, cv::Scalar(0, 0, 255), 2);
				cv::circle(outputImage, tg, 15, cv::Scalar(0, 255, 0), 2);
				cv::rectangle(outputImage, roi, cv::Scalar(0, 0, 255), 2, 1);
			}
			cv::imshow("Monitor", outputImage);
		}


	}
	outfile.close();
	cout << "Ending..." << endl;
	mydev.stopDevice(true);

	return 0;
}
