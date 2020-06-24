#include "utils/opencv_conv.hpp"

void frame2Mat(cv::Mat& out, const rs2::frame& fr, int type, int color) {

	int width = fr.as<rs2::video_frame>().get_width();
	int height = fr.as<rs2::video_frame>().get_height();

	cv::Mat mat(cv::Size(width, height), type, (void*)fr.get_data(),
			cv::Mat::AUTO_STEP);

	if (color > 0)
		cv::cvtColor(mat, out, cv::COLOR_BGR2RGB);
	else
		out = mat;
}
