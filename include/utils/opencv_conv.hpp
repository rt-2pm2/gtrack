#ifndef _OPENCV_CONV_HPP_
#define _OPENCV_CONV_HPP_

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>

#include <opencv2/optflow.hpp>
#include <opencv2/highgui.hpp>

#include <librealsense2/rs.hpp>

void frame2Mat(cv::Mat& out, const rs2::frame& fr, int type, int col);

#endif
