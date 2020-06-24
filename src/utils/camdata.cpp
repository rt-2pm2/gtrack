#include "utils/camdata.hpp"

CamData::CamData() {}

CamData::~CamData() {
	mx.unlock();
}

void CamData::set_rgb(const cv::Mat& r) {
	mx.lock();
	r.copyTo(rgb);
	mx.unlock();
}

void CamData::set_depth(const cv::Mat& d) {
	mx.lock();
	d.copyTo(depth);
	mx.unlock();
}

void CamData::get_rgb(cv::Mat& r) {
	mx.lock();
	rgb.copyTo(r);
	mx.unlock();
}

void CamData::get_depth(cv::Mat& d) {
	mx.lock();
	depth.copyTo(d);
	mx.unlock();
}


void CamData::set_depth_col(const cv::Mat& d) {
	mx.lock();
	d.copyTo(depth_color);
	mx.unlock();
}

void CamData::get_depth_col(cv::Mat& d) {
	mx.lock();
	depth_color.copyTo(d);
	mx.unlock();
}
