#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core.hpp>

#include "tracker/tracker.hpp"
#include "utils/timelib.hpp"


Tracker::Tracker() {}


Tracker::~Tracker() {
	stop_flow();
	_flow_thread_active = false;
	if (_flow_thread.joinable())
		_flow_thread.join();

	for (auto el : _targets) {
		delete el.second;
	}
}


void Tracker::start_flow() {
	_flowActive = true;
	_flow_thread_active = true;

	_flow_thread = std::thread(&Tracker::opticalflow_runnable, this);
}


void Tracker::stop_flow() {
	_flowActive = false;
}


void Tracker::add_target(int id, cv::Rect2d roi) {
	if (_targets.count(id) == 0) {
		std::cout << "Adding new target: ID[" << id << "]" << std::endl;
		TargetData* p_td = new TargetData();
		p_td->roi = roi;
		_targets.insert(std::pair<int, TargetData*>(id, p_td));
	} else {
		std::cout << "Updating target: ID[" << id << "]" << std::endl;
		_targets[id]->roi = roi;
	}
}


void Tracker::set_roi(int id, int x, int y, int w, int h) {
	if (_targets.count(id) == 0) {
		std::cout << "No target with ID = " << id << std::endl;
		return;
	} else {
		_targets[id]->roi = cv::Rect2d(x, y, w, h);
	}
}


void Tracker::set_position(int id, Eigen::Vector3d& pos) {

	if (_targets.count(id) == 0) {
		std::cout << "No target with ID = " << id << std::endl;
		return;
	} else {
		// Get the target position with respect to the camera frame
		float b_target_[3] {};
		for (int i = 0; i < 3; i++) {
			b_target_[i] = pos(i);
		}

		float upixel[2] {};

		rs2_project_point_to_pixel(upixel, &_camera_intr, b_target_);
		_targets[id]->img_target.x = upixel[0];
		_targets[id]->img_target.y = upixel[1];

		// The std of the target area is used to update the height and
		// width. 
		_targets[id]->roi.x = upixel[0] -
			(_targets[id]->roi.height / 2.0);
		_targets[id]->roi.y = upixel[1] -
			(_targets[id]->roi.width / 2.0);
	}
}


void Tracker::position2pixel(cv::Point& pt, const Eigen::Vector3d& pos) {
	float b_target_[3] {};
	for (int i = 0; i < 3; i++) {
		b_target_[i] = pos(i);
	}

	float upixel[2] {};
	rs2_project_point_to_pixel(upixel, &_camera_intr, b_target_);

	pt.x = upixel[0];
	pt.y = upixel[1];
}

void Tracker::setCamMatrix(rs2_intrinsics intr, double scale) {
	_camera_intr = intr;
	_dscale = scale;
}


void Tracker::step(cv::Mat& rgb, cv::Mat& depth) {
	int nclust = 4;
	int attempts = 1;

	// Copy the RGB frame to the internal variable
	rgb.copyTo(cvFrame);

	// Check the active ROIs
	for (auto el : _targets) {
		TargetData* tg_data = el.second;

		// Extract the ROI region from the RGB image.
		tg_data->rgb_roi = cv::Mat(cvFrame, tg_data->roi);
		// Extract the ROI region from the Depth image.
		tg_data->depth_roi = cv::Mat(depth, tg_data->roi);
	

		// Localization step:
		find_target_in_roi(*tg_data, nclust, attempts);

		// Update the ROI
		// Compute the pixel coordinates of the target in the full frame.
		int img_x_global = tg_data->depth_tg[0] + tg_data->roi.tl().x;
		int img_y_global = tg_data->depth_tg[1] + tg_data->roi.tl().y;
		int img_x_std = tg_data->depth_tg_std[0];
		int img_y_std = tg_data->depth_tg_std[1];	

		tg_data->img_target.x = img_x_global;
		tg_data->img_target.y = img_y_global;


		// The std of the target area is used to update the height and
		// width. 
		// The ROI is moved in the center of the target.
		tg_data->roi.height = std::min(150, std::max(5 * img_x_std, 100));
		tg_data->roi.width = std::min(150, std::max(5 * img_y_std, 100));

		tg_data->roi.x = img_x_global - (tg_data->roi.height / 2.0);
		tg_data->roi.y = img_y_global - (tg_data->roi.width / 2.0);


		// Get the target position with respect to the camera frame
		float b_target_[3] {};
		float upixel[2] {(float)img_x_global, (float)img_y_global};

		rs2_deproject_pixel_to_point(b_target_, &_camera_intr,
				upixel, tg_data->depth_tg[2] * _dscale);

		for (int i = 0; i < 3; i++) {
			tg_data->b_tg[i] = b_target_[i];
		}
	}
}


void Tracker::get_rgbROI(int id, cv::Mat& roi) {
	if (_targets.count(id) == 0) {
		std::cout << "No target with ID = " << id << std::endl;
		return;
	} else {
		_targets[id]->rgb_roi.copyTo(roi);
	}
}

void Tracker::get_ROI(int id, cv::Rect2d& roi) {
	if (_targets.count(id) == 0) {
		std::cout << "No target with ID = " << id << std::endl;
		return;
	} else {
		roi = _targets[id]->roi;
	}
}

void Tracker::get_flowmask(int id, cv::Mat& m) {
	if (_targets.count(id) == 0) {
		std::cout << "No target with ID = " << id << std::endl;
		return;
	} else {
		//_targets[id]->flow_mask.copyTo(m);
		_tg_data.flow_mask.copyTo(m);
	}
}

void Tracker::get_img_tg(int id, cv::Point& tg) {
	if (_targets.count(id) == 0) {
		std::cout << "No target with ID = " << id << std::endl;
		return;
	} else {
		tg = _targets[id]->img_target;
	}
}

void Tracker::get_b_tg(int id, std::array<float, 3>& tg) {
	if (_targets.count(id) == 0) {
		std::cout << "No target with ID = " << id << std::endl;
		return;
	} else {
		tg = _targets[id]->b_tg;
	}
}

void Tracker::get_mask(int id, cv::Mat& m) {
	if (_targets.count(id) == 0) {
		std::cout << "No target with ID = " << id << std::endl;
		return;
	} else {
		_targets[id]->depth_mask.copyTo(m);
	}
}

void Tracker::get_depthROI(int id, cv::Mat& m) {
	if (_targets.count(id) == 0) {
		std::cout << "No target with ID = " << id << std::endl;
		return;
	} else {
		_targets[id]->depth_roi.copyTo(m);
	}
}

void Tracker::get_depthTG(int id, std::array<int, 3>& tg) {
	if (_targets.count(id) == 0) {
		std::cout << "No target with ID = " << id << std::endl;
		return;
	} else {
		tg = _targets[id]->depth_tg;
	}
}


// ============================================================
// ============================================================

double Tracker::findTarget_Kmeans(std::array<float, 3>& b_target,
		cv::Point& img_target, const cv::Mat& depth,
		const cv::Rect2d& roi, int ks, int attempts) {

	cv::Mat labels;
	double tg_dist;
	double tg_dist_std;

	// Find the distance from the target.
	cv::Mat depth_roi = cv::Mat(depth, roi);

	cv::Mat depth_roi_32f;
	depth_roi.convertTo(depth_roi_32f, CV_32F);
	depth_roi_32f *= _dscale;

	double compactness = computeKmeans(&tg_dist, &tg_dist_std,
			depth_roi_32f, ks, attempts, 0.1);

	// Select the part of the image which as a measured distance, 
	// near the one estimated from the target.
	// Precisely, select the image where the distance is 
	// < (tg_distance + tg_distance_std)
	cv::threshold(depth_roi_32f, _tg_data.depth_mask,
			tg_dist + 1.0 * tg_dist_std, 1.0,
			cv::THRESH_BINARY_INV);

	//imshow("Monitor Threshold", thr_img);

	// The threshold is a image with 1.0 in the selected region,
	// compute the center of mass of the selected region.
	cv::Moments Mm = cv::moments(_tg_data.depth_mask);

	double x_tg_local = Mm.m10 / Mm.m00;
	double y_tg_local = Mm.m01 / Mm.m00;

	// Compute the pixel coordinate in the full frame.
	int img_x_global = x_tg_local + roi.tl().x;
	int img_y_global = y_tg_local + roi.tl().y;

	// Get the target position with respect to the camera frame
	float b_target_[3] {};
	float upixel[2] {(float)img_x_global, (float)img_y_global};
	rs2_deproject_pixel_to_point(b_target_, &_camera_intr,
			upixel, tg_dist);

	for (int i = 0; i < 3; i++) {
		b_target[i] = b_target_[i];
	}

	img_target.x = img_x_global;
	img_target.y = img_y_global;

	return tg_dist;
}


void Tracker::find_target_in_roi(TargetData& tg_data, int ks, int attempts) {
	double tg_dist;
	double tg_dist_std;

	cv::Mat depth_roi_32f;
	tg_data.depth_roi.convertTo(depth_roi_32f, CV_32F);

	// Compute the depth cluster of the target.
	double compactness = computeKmeans(&tg_dist, &tg_dist_std,
			depth_roi_32f, ks, attempts, 0.1);


	// Select the part of the image which as a measured distance, 
	// near the one estimated from the target.
	// Precisely, select the image where the distance is 
	// < (tg_distance + tg_distance_std)
	cv::Mat thr1, thr2;
	cv::Mat mask1, mask2, mask;
	cv::threshold(depth_roi_32f, thr1,
			tg_dist + 0.2 * tg_dist_std, 1.0,
			cv::THRESH_BINARY_INV);
	cv::bitwise_and(thr1, thr1, mask1);
	//mask1.convertTo(mask1, CV_8U);
	
	cv::threshold(depth_roi_32f, thr2,
			tg_dist - 4.0 * tg_dist_std, 1.0,
			cv::THRESH_BINARY);
	cv::bitwise_and(thr2, thr2, mask2);
	//mask1.convertTo(mask2, CV_8U);

	cv::bitwise_and(mask1, mask2, mask);
	//mask.convertTo(mask, CV_8U);


	// The threshold is a image with 1.0 in the selected region,
	// compute the center of mass of the selected region.
	mask.copyTo(tg_data.depth_mask);
	cv::Moments Mm = cv::moments(mask);
	tg_data.depth_mask_moments = Mm;

	tg_data.depth_tg[0] = (int)(Mm.m10 / Mm.m00);
	tg_data.depth_tg[1] = (int)(Mm.m01 / Mm.m00);
	tg_data.depth_tg[2] = (int)tg_dist;

	tg_data.depth_tg_std[0] = (int)std::sqrt(Mm.mu20 / Mm.m00);
	tg_data.depth_tg_std[1] = (int)std::sqrt(Mm.mu02 / Mm.m00);
	tg_data.depth_tg_std[2] = (int)tg_dist_std;

}


double Tracker::computeKmeans(
		double* tg_distance, double* tg_distance_std,
		const cv::Mat& depth, int ks, int attempts, double err) {
	cv::Mat labels;
	cv::Mat centers;

	int nrow = depth.rows;
	int ncol = depth.cols;
	
	// Convert the depth cv::Mat into a vector form 
	cv::Mat tg_v;
	tg_v = depth.reshape(0, nrow * ncol);
	cv::Mat tg_v_nz;

	for (int i = 0; i < tg_v.rows; i++) {
		const float* pf = tg_v.ptr<float>(i);	
		if (*pf > 1) {
			tg_v_nz.push_back(*pf);
		}
	}

	// Run the KMean algorithm on the reshaped depth cv::Mat
	// to find 'ks' clusters and respective centers
	double compactness = cv::kmeans(tg_v_nz, ks, labels,
			cv::TermCriteria(cv::TermCriteria::EPS, 300, err),
			attempts, cv::KMEANS_PP_CENTERS, centers);

	cv::Mat indexes;
	cv::sortIdx(centers, indexes,
			cv::SORT_EVERY_COLUMN + cv::SORT_ASCENDING);

	// Create a mask to select the element of the different clusters.
	int min_loc = indexes.at<int>(0);
	cv::Mat index = (labels == min_loc);
	float min = centers.at<float>(min_loc);

	cv::Mat mask;
	cv::bitwise_and(index, index, mask);

	// Compute Mean and Std of the clusters.
	std::vector<double> avg;
	std::vector<double> std;
	cv::meanStdDev(tg_v_nz, avg, std, mask);

	*tg_distance = min;
	*tg_distance_std = std[0];

	return compactness;
}


void Tracker::opticalflow_runnable() {
	double pyr_scale = 0.5;
	int levels = 2;
	int winsize = 20;
	int niters = 2;
	int poly_n = 20;
	double poly_sigma = poly_n * 0.2;
	int flags = 0;

	cv::VideoWriter oWriter_flow("./output_flow.avi",
			cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 5,
			cv::Size(1280, 720), true);

	timespec nextAct;
	timespec period;
	period.tv_nsec = 0 * 1e6; // ms
	period.tv_sec = 1;

	clock_gettime(CLOCK_MONOTONIC, &nextAct);

	while(cvFrame.empty()) {}

	cv::Mat flow;
	cv::Mat magnitude, angle, magn_norm;
	cv::Mat cvPrev, cvNext;

	while (_flow_thread_active) {
		// Compute the next activation
		add_timespec(&nextAct, &period, &nextAct);
		
		cv::resize(cvFrame, cvNext, cv::Size(),
				0.5, 0.5, cv::INTER_LINEAR);
		//cvFrame.copyTo(cvNext);

		// Convert the image to gray and prepare the flow Mat
		// The flow is composed by two matrices where each channel is 
		// a coordinate (x, y)
		cv::cvtColor(cvNext, cvNext, cv::COLOR_RGB2GRAY);

		if (!cvPrev.empty() && _flowActive) {
			cv::Mat bgr;

			cv::calcOpticalFlowFarneback(cvPrev, cvNext, flow, 
					pyr_scale, levels,
					winsize, niters,
					poly_n, poly_sigma, flags);  

			cv::Mat flowParts[2];

			cv::split(flow, flowParts);
			cv::cartToPolar(flowParts[0], flowParts[1],
					magnitude, angle, true);

			cv::normalize(magnitude, magn_norm, 0.0f, 1.0f,
					cv::NORM_MINMAX);

			angle *= ((1.f / 360.f) * (180.f / 255.f));

			cv::Mat flow_mask;
			cv::Mat nonzero_pix;
			cv::threshold(magnitude, flow_mask, 0.9, 1.0,
					cv::THRESH_BINARY);
			cv::findNonZero(flow_mask, nonzero_pix); 
			nonzero_pix.convertTo(nonzero_pix, CV_32FC2, 1, 0);

			_tg_data.flow_mask = flow_mask;

			if (!nonzero_pix.empty()) {
				int ks = 1;
				int attempts = 2;
				cv::Mat centers;
				cv::Mat labels;
				double err = 0.5;
				cv::kmeans(nonzero_pix, ks, labels,
						cv::TermCriteria(cv::TermCriteria::EPS +
							cv::TermCriteria::COUNT, 20, err),
						attempts, cv::KMEANS_PP_CENTERS, centers);
			}

			//build hsv image
			cv::Mat _hsv[3], hsv, hsv8;
			_hsv[0] = angle;
			_hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
			_hsv[2] = magn_norm;

			cv::merge(_hsv, 3, hsv);
			hsv.convertTo(hsv8, CV_8U, 255.0);
			cv::cvtColor(hsv8, bgr, cv::COLOR_HSV2BGR);

			// Save video to file
			if (!bgr.empty()) {
				cv::Mat out;
				cv::cvtColor(bgr, out, cv::COLOR_BGR2RGB);
				oWriter_flow.write(out);
			}
		}

		cvPrev = cvNext;
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,  &nextAct, NULL);
	}

	oWriter_flow.release();
}

void Tracker::computeHist(cv::Mat& hist, int numBins,
		const cv::Mat& data, double scale) {
	int nimages = 1;
	float min_val = 0;
	float max_val = std::numeric_limits<uint16_t>::max() * scale;
	float range[] = {min_val, max_val}; //the upper boundary is exclusive
	const float* histRange = {range};

	// Compute the mask to remove the shadows in the depth image.
	cv::Mat mask;
	cv::bitwise_and(data, data, mask);
	mask.convertTo(mask, CV_8U);

	cv::Mat data_32f(data.rows, data.cols, CV_32F);
	data.convertTo(data_32f, CV_32F);

	data_32f *= scale;

	// Calculate histogram
	cv::calcHist(&data_32f, nimages, 0, mask, hist, 1, &numBins, &histRange, true, false);
}

void Tracker::get_histogram(cv::Mat& hist_img, int numBins,
		const cv::Mat& depth_roi, int target_depth) {

	cv::Mat t_hist;
	computeHist(t_hist, numBins, depth_roi, 1);

	int hist_w = 512, hist_h = 400;
	hist_img = cv::Mat(hist_h, hist_w, CV_8UC3, cv::Scalar(0,0,0));	

	int bin_w = cvRound((double) hist_w/numBins);

	// Normalize histogram bin length to fit the image.
	float max_val = std::numeric_limits<uint16_t>::max();	
	cv::normalize(t_hist, t_hist, 0, hist_img.rows, cv::NORM_MINMAX);

	for(int h = 0; h < numBins; h++) {
		float binVal = t_hist.at<float>(h);
		cv::rectangle(hist_img,
				cv::Point(h*bin_w, 0),
				cv::Point((h+1)*bin_w - 1, binVal),
				cv::Scalar(0, 0, 255),
				-1);
	}

	int index = target_depth / (max_val / numBins);
	cv::rectangle(hist_img,
			cv::Point(index * bin_w, 0),
			cv::Point((index+1)*bin_w - 1, 150.0),
			cv::Scalar(0, 255, 0),
			-1);
}
