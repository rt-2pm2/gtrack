#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core.hpp>

#include "mmtracker/mmtracker.hpp"
#include "utils/timelib.hpp"

using namespace mmtracker;

MMTracker::MMTracker() {
	_MaxTargets = 3;

	_delta_depth_param = 5.0;

	opt_tracker = cv::TrackerCSRT::create();
	
	_min_flow_threshold = 0.6;
	_min_flow_threshold_norm = 0.7;

	_opt_flow_scale = 0.5;
	_opt_flow_detect_thr = 150.0;
	
	pyr_scale = 0.5;
	levels = 2;
	winsize = 20;
	niters = 2;
	poly_n = 20;
	poly_sigma = poly_n * 0.2;
	flags = 0;

	_log_flow.open("trk_flow_log.csv");
	_log_flow << "time[us] min_flow max_flow Npix" << std::endl;

	_log_trk.open("trk_log.csv");
}


MMTracker::~MMTracker() {
	_log_flow.close();
	_log_trk.close();

	for (auto el : _targets) {
		delete el.second;
	}
}


void MMTracker::set_delta_depth_param(double d) {
	_delta_depth_param = d;
}


void MMTracker::set_transform(const Eigen::Vector3d& t, const Eigen::Quaterniond& q) {
	C_p_CM_ = t;
	q_CM_ = q;
}

void MMTracker::set_flow_thr(double thr, double norm_thr) {
 	_min_flow_threshold = thr;
	_min_flow_threshold_norm = norm_thr;
}

void MMTracker::add_target(int id, cv::Rect2d roi) {
	std::unique_lock<std::mutex> lk(_mx);
	if (_targets.count(id) == 0) {
		std::cout << "Adding new target: ID[" << id << "]" << std::endl;
		std::cout << "ROI = " << roi << std::endl;

		TargetData* p_td = new TargetData();
		p_td->id = id;
		p_td->roi = roi;
		_targets.insert(std::pair<int, TargetData*>(id, p_td));

		//opt_tracker->init(cvFrame, roi);
	} else {
		std::cout << "Updating target: ID[" << id << "]" << std::endl;
		_targets[id]->roi = roi;
		//opt_tracker->init(cvFrame, roi);
	}
}



void MMTracker::set_roi(int id, int x, int y, int w, int h) {
	std::unique_lock<std::mutex> lk(_mx);
	if (_targets.count(id) == 0) {
		//std::cout << "No target with ID = " << id << std::endl;
		return;
	} else {
		_targets[id]->roi = cv::Rect2d(x, y, w, h);
	}
}


void MMTracker::set_position(int id, Eigen::Vector3d& pos) {
	std::unique_lock<std::mutex> lk(_mx);
	if (_targets.count(id) == 0) {
		//std::cout << "No target with ID = " << id << std::endl;
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


void MMTracker::position2pixel(cv::Point& pt, const Eigen::Vector3d& pos) {
	std::unique_lock<std::mutex> lk(_mx);
	float b_target_[3] {};
	for (int i = 0; i < 3; i++) {
		b_target_[i] = pos(i);
	}

	float upixel[2] {};
	rs2_project_point_to_pixel(upixel, &_camera_intr, b_target_);

	pt.x = upixel[0];
	pt.y = upixel[1];
}


void MMTracker::setCamMatrix(rs2_intrinsics intr) {
	_camera_intr = intr;
}

void MMTracker::setDepthScale(double scale) {
	_dscale = scale;
}


int MMTracker::step(cv::Mat& rgb, cv::Mat& depth) {
	int out = 0;

	//XXX I should improve the way I protect the variables
	std::unique_lock<std::mutex> lk(_mx);
	int nclust = 4;
	int attempts = 1;

	// Copy the RGB frame to the internal variable
	rgb.copyTo(cvFrame);
	depth.copyTo(cvDepth);

	// For each ROI perform the localization of the 
	// target. The idea is that the ROIs are added 
	// with some other methods (NN, OpticalFlow)
	
	// Get the timestamp
	timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	uint64_t t_micro = timespec2micro(&t);

	auto el_it = _targets.begin();
	while (el_it !=  _targets.end()) {
		TargetData* tg_data = el_it->second;
		cv::Rect2d roi = tg_data->roi;

		if (roi.empty()) {
			continue;
		}

		// Extract the ROI region from the RGB image.
		tg_data->rgb_roi = cv::Mat(cvFrame, roi);
		// Extract the ROI region from the Depth image.
		tg_data->depth_roi = cv::Mat(depth, roi);

		// Localization step.
		// This is the core of the localization.
		// 1) I use a KMeans approach to locate the correct distance of the target 
		// in the ROI;
		// 2) I mask the pixel with a given distance from the camera to locate the 
		// position of the target in the image;
		// 3) I convert the pixel coordinates into real-world coordinates.
		//cv::Rect2d local_roi = tg_data->roi;
		//bool ok = opt_tracker->update(cvFrame, local_roi);
		bool measvalid = find_target_in_roi(tg_data, nclust, attempts);

		if (measvalid) {
			// Get the target position with respect to the camera frame
			float b_target_[3] {};
			float upixel[2] {
				(float)tg_data->img_target.x,
				(float)tg_data->img_target.y
			};

			rs2_deproject_pixel_to_point(b_target_, &_camera_intr,
					upixel, tg_data->depth_tg[2] * _dscale);

			for (int i = 0; i < 3; i++) {
				tg_data->b_tg[i] = b_target_[i];
			}

			_log_trk << t_micro << " " << el_it->first << " " <<
				b_target_[0] << " " << b_target_[1] << " " <<
				b_target_[2] << std::endl;
			el_it++;
		} else {
			std::cout << "Depth not valid" << std::endl;	
			/*
			   img_x_global = local_roi.tl().x + local_roi.width;
			   img_y_global = local_roi.tl().y + local_roi.height;
			   tg_data->roi = local_roi;
			*/	

			// If the depth measurement was not valid remove the 
			// target from the local map of the tracked vehicles.
			delete el_it->second;
			el_it = _targets.erase(el_it);
		}
	}
	out = _targets.size();

	return out;
}


bool MMTracker::get_rgbROI(int id, cv::Mat& roi) {
	bool out = false;
	std::unique_lock<std::mutex> lk(_mx);
	if (_targets.count(id) == 0) {
		//std::cout << "No target with ID = " << id << std::endl;
		return false;
	} else {
		_targets[id]->rgb_roi.copyTo(roi);
		return true;
	}
}

bool MMTracker::get_ROI(int id, cv::Rect2d& roi) {
	std::unique_lock<std::mutex> lk(_mx);
	if (_targets.count(id) == 0) {
		//std::cout << "No target with ID = " << id << std::endl;
		return false;
	} else {
		roi = _targets[id]->roi;
		return true;
	}
}

bool MMTracker::get_flowmask(int id, cv::Mat& m) {
	std::unique_lock<std::mutex> lk(_mx);
	if (_targets.count(id) == 0) {
		//std::cout << "No target with ID = " << id << std::endl;
		return false;
	} else {
		//_targets[id]->flow_mask.copyTo(m);
		_flow_mask.copyTo(m);
		return true;
	}
}


int MMTracker::get_targets(std::vector<TargetData>& T) {
	std::lock_guard<std::mutex> lk(_mx);
	int count = 0;
	for (auto el : _targets) {
		T.push_back(*el.second);
		count++;
	}
	return count;
}


bool MMTracker::get_img_tg(int id, cv::Point& tg) {
	std::unique_lock<std::mutex> lk(_mx);
	if (_targets.count(id) == 0) {
		//std::cout << "No target with ID = " << id << std::endl;
		return false;
	} else {
		tg = _targets[id]->img_target;
		return true;
	}
}

bool MMTracker::get_b_tg(int id, Eigen::Vector3d& tg) {
	std::unique_lock<std::mutex> lk(_mx);
	if (_targets.count(id) == 0) {
		//std::cout << "No target with ID = " << id << std::endl;
		return false;
	} else {
		tg = _targets[id]->b_tg;
		return true;
	}
}

bool MMTracker::get_mask(int id, cv::Mat& m) {
	std::unique_lock<std::mutex> lk(_mx);
	if (_targets.count(id) == 0) {
		//std::cout << "No target with ID = " << id << std::endl;
		return false;
	} else {
		_targets[id]->depth_mask.copyTo(m);
		return true;
	}
}

bool MMTracker::get_depthROI(int id, cv::Mat& m) {
	std::unique_lock<std::mutex> lk(_mx);
	if (_targets.count(id) == 0) {
		//std::cout << "No target with ID = " << id << std::endl;
		return false;
	} else {
		_targets[id]->depth_roi.copyTo(m);
		return true;
	}
}

bool MMTracker::get_depthTG(int id, std::array<int, 3>& tg) {
	std::unique_lock<std::mutex> lk(_mx);
	if (_targets.count(id) == 0) {
		//std::cout << "No target with ID = " << id << std::endl;
		return false;
	} else {
		tg = _targets[id]->depth_tg;
		return true;
	}
}


// ============================================================
// ============================================================


bool MMTracker::find_target_in_roi(TargetData* tg_data,
		int ks, int attempts) {
	bool depthvalid = true;

	double tg_dist;
	double tg_dist_std;

	cv::Mat depth_roi_32f;
	tg_data->depth_roi.convertTo(depth_roi_32f, CV_32F);

	// Compute the depth cluster of the target.
	int numK = find_target_depth(&tg_dist, &tg_dist_std,
			depth_roi_32f, attempts, 0.1);

	// Select the part of the image which as a measured distance, 
	// near the one estimated from the target.
	cv::Mat thr1, thr2;
	cv::Mat mask;

	double Delta = _delta_depth_param * tg_dist_std;

	// X  < tg_dist + Delta 
	cv::threshold(depth_roi_32f, thr1, tg_dist + Delta, 1.0,
			cv::THRESH_BINARY_INV);
	// X > tg_dist - Delta	
	cv::threshold(depth_roi_32f, thr2, tg_dist - Delta, 1.0,
			cv::THRESH_BINARY);

	cv::bitwise_and(thr1, thr2, mask);

	// I check that the selected depth is not on the border of the 
	// roi. I need to apply this step because if the background
	// is not well separated from the drone the detection will
	// drift away.

	cv::Mat indexes;
	cv::findNonZero(mask, indexes);
	int nrows = mask.rows;
	int ncols = mask.cols;
	 
	for (int k_ = 0; k_ < indexes.total(); k_++) {
		cv::Point p = indexes.at<cv::Point>(k_);
		if (p.x == 0 || p.x == (ncols - 1)) {
			depthvalid = false;
			break;
		}
		if (p.y == 0 || p.y == (nrows - 1)) {
			depthvalid = false;
			break;
		}
	}

	if (depthvalid) {
		// The threshold is a image with 1.0 in the selected region,
		// compute the center of mass of the selected region.
		cv::Moments Mm = cv::moments(mask);
		tg_data->depth_mask_moments = Mm;

		int depth_tg[3];
		int depth_tg_std[3];

		depth_tg[0] = (int)(Mm.m10 / Mm.m00);
		depth_tg[1] = (int)(Mm.m01 / Mm.m00);
		depth_tg[2] = (int)tg_dist;
		depth_tg_std[0] = (int)std::sqrt(Mm.mu20 / Mm.m00);
		depth_tg_std[1] = (int)std::sqrt(Mm.mu02 / Mm.m00);
		depth_tg_std[2] = (int)tg_dist_std;

		// Compute the pixel coordinates of the target in the full frame.
		int img_x_global = depth_tg[0] + tg_data->roi.tl().x;
		int img_y_global = depth_tg[1] + tg_data->roi.tl().y;
		int img_x_std = depth_tg_std[0];
		int img_y_std = depth_tg_std[1];	

		tg_data->img_target.x = img_x_global;
		tg_data->img_target.y = img_y_global;

		// The std of the target area is used to update the height and
		// width. 
		// The ROI is moved in the center of the target.
		tg_data->roi.height = std::min(80, std::max(5 * img_x_std, 80));
		tg_data->roi.width = std::min(130, std::max(5 * img_y_std, 130));

		int base_x = img_x_global - (tg_data->roi.width / 2.0);
		int base_y = img_y_global - (tg_data->roi.height / 2.0);

		tg_data->roi.x = std::max(0,
				std::min(base_x, cvFrame.cols - (int)tg_data->roi.width)
				);

		tg_data->roi.y = std::max(0,
				std::min(base_y, cvFrame.rows - (int)tg_data->roi.height)
				);

		tg_data->depth_tg[0] = depth_tg[0];
		tg_data->depth_tg[1] = depth_tg[1];
		tg_data->depth_tg[2] = depth_tg[2];
		tg_data->depth_tg_std[0] = depth_tg_std[0];
		tg_data->depth_tg_std[1] = depth_tg_std[1];
		tg_data->depth_tg_std[2] = depth_tg_std[2];
		
	} 

	return depthvalid;
}





void MMTracker::find_target_in_roi(
		Eigen::Vector3d& b_tg,
		const cv::Rect2d& roi) {
	double tg_dist;
	double tg_dist_std;

	cv::Mat depth_roi(cvDepth, roi);
	cv::Mat depth_roi_32f;
	depth_roi.convertTo(depth_roi_32f, CV_32F);

	// Compute the depth cluster of the target.
	int numK = find_target_depth(&tg_dist, &tg_dist_std,
			depth_roi_32f, 2, 0.1);

	// Select the part of the image which as a measured distance, 
	// near the one estimated from the target.
	cv::Mat thr1, thr2;
	cv::Mat mask;

	double Delta = _delta_depth_param * tg_dist_std;

	// X  < tg_dist + Delta 
	cv::threshold(depth_roi_32f, thr1, tg_dist + Delta, 1.0,
			cv::THRESH_BINARY_INV);
	// X > tg_dist - Delta	
	cv::threshold(depth_roi_32f, thr2, tg_dist - Delta, 1.0,
			cv::THRESH_BINARY);

	cv::bitwise_and(thr1, thr2, mask);
	
	// The threshold is a image with 1.0 in the selected region,
	// compute the center of mass of the selected region.
	cv::Moments Mm = cv::moments(mask);
	
	float b_tg_pix[3] {};
	b_tg_pix[0] = (int)(Mm.m10 / Mm.m00);
	b_tg_pix[1] = (int)(Mm.m01 / Mm.m00);
	b_tg_pix[2] = (int)tg_dist;

	float upixel[2] {
		(float)b_tg_pix[0] + roi.tl().x,
		(float)b_tg_pix[1] + roi.tl().y
	};

	float b_target_[3];
	rs2_deproject_pixel_to_point(b_target_, &_camera_intr,
				upixel, b_tg_pix[2] * _dscale);

	for (int i = 0; i < 3; i++) {
		b_tg(i) = b_target_[i];
	}
}


int MMTracker::find_target_depth(
		double* tg_distance, double* tg_distance_std,
		const cv::Mat& depth, int attempts, double err) {
	cv::Mat labels;
	cv::Mat distances;

	int nrow = depth.rows;
	int ncol = depth.cols;

	int MaxK = 8;
	
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
	// to find 'ks' clusters and respective distances
	int ks = 0;
	double cmp = 0;

	if (tg_v_nz.total() > 0) {
		ks++;
		double cmp = cv::kmeans(tg_v_nz, ks, labels,
				cv::TermCriteria(cv::TermCriteria::EPS +
					cv::TermCriteria::COUNT, 20, err),
				attempts, cv::KMEANS_PP_CENTERS, distances);

		while (ks < MaxK) {
			ks++;
			cv::Mat local_labels = labels;
			cv::Mat local_distances = distances;
			double local_cmp;

			local_cmp = cv::kmeans(tg_v_nz, ks, local_labels,
					cv::TermCriteria(cv::TermCriteria::EPS +
						cv::TermCriteria::COUNT, 10, err),
					attempts, cv::KMEANS_USE_INITIAL_LABELS,
					local_distances);


			double improv_ratio = (cmp - local_cmp)/cmp;
			if (improv_ratio < 0.3) {
				break;
			} else {
				labels = local_labels;
				distances = local_distances;
				cmp = local_cmp; }
		}
	
	// Order the clusters in ascending order.
	cv::Mat indexes;
	cv::sortIdx(distances, indexes,
			cv::SORT_EVERY_COLUMN + cv::SORT_ASCENDING);

	// Create a mask to select the element of the nearest clusters.
	int min_loc = indexes.at<int>(0);
	cv::Mat mask = (labels == min_loc);
	float min = distances.at<float>(min_loc);

	// Compute Mean and Std of the clusters.
	std::vector<double> avg;
	std::vector<double> std;
	cv::meanStdDev(tg_v_nz, avg, std, mask);

	*tg_distance = min;
	*tg_distance_std = std[0];
	}

	return ks;
}


void MMTracker::optical_flow_step(cv::Mat& cvFrame,
		std::vector<TargetData>& untracked) {
	cv::Mat flow;
	cv::Mat cvNext;
	timespec temp;

	_frame_height = cvFrame.rows;
	_frame_width = cvFrame.cols;

	int flow_height = _frame_height * _opt_flow_scale;
	int flow_width = _frame_width * _opt_flow_scale;

	cv::resize(cvFrame, cvNext, cv::Size(),
			_opt_flow_scale, _opt_flow_scale, cv::INTER_LINEAR);

	// Convert the image to gray and prepare the flow Mat
	// The flow is composed by two matrices where each channel is 
	// a coordinate (x, y)
	cv::cvtColor(cvNext, cvNext, cv::COLOR_RGB2GRAY);
	if (!cvPrev.empty()) {
		clock_gettime(CLOCK_MONOTONIC, &temp);
		cv::calcOpticalFlowFarneback(cvPrev, cvNext, flow, 
				pyr_scale, levels,
				winsize, niters,
				poly_n, poly_sigma, flags);  

		cv::Mat flowParts[2];

		cv::split(flow, flowParts);
		cv::cartToPolar(flowParts[0], flowParts[1],
				magnitude, angle, true);

		double max, min;
		cv::minMaxLoc(magnitude, &min, &max);

		// Thresholding the flow to remove noise.
		cv::Mat magn_thr;
		cv::threshold(magnitude, magn_thr, _min_flow_threshold,
				0, cv::THRESH_TOZERO);
		// Normalizing
		cv::normalize(magn_thr, magn_norm, 0.0f, 1.0f,
				cv::NORM_MINMAX);
		angle *= ((1.f / 360.f) * (180.f / 255.f));

		cv::Mat flow_mask;
		cv::Mat nonzero_pix;
		// Select the part with a noticeable movement
		cv::threshold(magn_norm, flow_mask,
				_min_flow_threshold_norm, 1.0, cv::THRESH_BINARY);
		cv::findNonZero(flow_mask, nonzero_pix); 
		nonzero_pix.convertTo(nonzero_pix, CV_32FC2, 1, 0);

		if (!flow_mask.empty()) {
			_flow_mask = flow_mask;
		}

		int Npix = nonzero_pix.total();
		_log_flow << timespec2micro(&temp) << " " << min <<
			" " << max << " " << Npix << std::endl;
		if (Npix > _opt_flow_detect_thr) {
			int ks = std::min(_MaxTargets, Npix);
			int attempts = 2;
			cv::Mat centers;
			cv::Mat labels;
			double err = 0.5;

			
			double perf = cv::kmeans(nonzero_pix, ks, labels,
					cv::TermCriteria(cv::TermCriteria::EPS +
						cv::TermCriteria::COUNT, 50, err),
					attempts, cv::KMEANS_PP_CENTERS, centers);

			centers = centers / _opt_flow_scale;

			// Check if the movement is associated to a point
			// which is already tracked
			for (int i = 0; i < ks; i++) {
				cv::Mat mask = (labels == i);
				double area = cv::sum(mask)[0];	

				if (area > _opt_flow_detect_thr) {
					bool newroi = true;
					cv::Point p(centers.row(i));

					// LOCK
					std::unique_lock<std::mutex> lk(_mx);
					for (auto el : _targets) {
						//std::cout << el.second->roi << std::endl;
						cv::Rect2d roi = el.second->roi;
						if (!roi.empty() && roi.contains(p)) {
							//std::cout << "Already tracked" << std::endl;
							newroi = false;	
						}	
					}
					lk.unlock();
					// UNLOCK

					if (newroi) {
						// Define a ROI in the area of the movement
						int base_x = p.x - 40;
						int base_y = p.y - 40;

						int roix = std::max(0,
								std::min(base_x, _frame_width - 80)
								);
						int roiy = std::max(0,
								std::min(base_y, _frame_height - 80)
								);

						cv::Rect2d tgroi(roix, roiy, 80, 80);
						Eigen::Vector3d pos_;

						// Try to identify a target postion in the ROI
						find_target_in_roi(pos_, tgroi);

						TargetData untrk_item;
						untrk_item.b_tg = pos_;
						untrk_item.roi = tgroi;

						untracked.push_back(untrk_item);
					}
				}
			}
		}

		/*
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
		   }
		*/
	}
	cvPrev = cvNext;
}


void MMTracker::computeHist(cv::Mat& hist, int numBins,
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

void MMTracker::get_histogram(cv::Mat& hist_img, int numBins,
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
