#ifndef _CAMDATA_HPP_
#define _CAMDATA_HPP_

#include <mutex>
#include <opencv2/core.hpp>

class CamData {
	private:
		mutable std::mutex mx;

		cv::Mat rgb;
		cv::Mat depth;
		cv::Mat depth_color;

		bool updated;

	public:
		CamData();
		~CamData();

		void set_rgb(const cv::Mat& m);
		void set_depth(const cv::Mat& m);
		void set_depth_col(const cv::Mat& m);

		void get_rgb(cv::Mat& m);
		void get_depth(cv::Mat& m);
		void get_depth_col(cv::Mat& m);
};

#endif
