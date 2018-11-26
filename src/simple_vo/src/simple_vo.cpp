/*
 * Created on Wed Nov 21 2018
 *
 * Copyright (c) 2018 EpsAvlc
 * All rights reserved
 *
 * Author: EpsAvlc(https://github.com/EpsAvlc)
 */

#include "simple_vo.h"

#include <iomanip>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>

using namespace simple_vo;
using namespace std;
using namespace cv;

SimpleVO::SimpleVO(ros::NodeHandle &nh, ros::NodeHandle &nh_local)
    : nh_(nh), nh_local_(nh_local), state_(INITIALIZING)
{
    updateParam();
    readConfigFile();
    odometry_R = Mat::eye(3, 3, DataType<double>::type);
    odometry_t = Mat::zeros(3, 1, DataType<double>::type);
    traj_ = cv::Mat::zeros(600, 600, CV_8UC3);// 用于绘制轨迹
    img_sub_ = nh_.subscribe("camera/image_raw", 1, &SimpleVO::callback, this);
}

SimpleVO::~SimpleVO()
{
    nh_local_.deleteParam("fast_threshold_");
    nh_local_.deleteParam("scale_factor");
    nh_local_.deleteParam("level_pyramid");
    nh_local_.deleteParam("match_ratio");
    nh_local_.deleteParam("config_file_name");
}

void SimpleVO::updateParam()
{
    nh_local_.param<int>("fast_threshold_", fast_threshold_, 40);
    nh_local_.param<double>("scale_factor", scale_factor_, 1.2);
    nh_local_.param<int>("level_pyramid", level_pyramid_, 8);
    nh_local_.param<double>("match_ratio", match_ratio_, 2.0);
    nh_local_.param<string>("config_file_name", config_file_name_, "");
}

void SimpleVO::callback(const sensor_msgs::ImageConstPtr img_ptr)
{
    cv_bridge::CvImageConstPtr cv_img_ptr = cv_bridge::toCvShare(img_ptr);
    addFrame(cv_img_ptr);
}

void SimpleVO::addFrame(const cv_bridge::CvImageConstPtr img_ptr)
{
    switch (state_)
    {
    case INITIALIZING:
    {
        state_ = OK;
        curr_ = ref_ = img_ptr;
        extractKeyPoints();
        updateStates();
        break;
    }
    case OK:
    {
        // Prevent bags loop play.
        ROS_ASSERT(img_ptr->header.stamp > ref_->header.stamp);
        curr_ = img_ptr;
        extractKeyPoints();
        featureTracking();
        poseEstimation();
        updateStates();
        drawTrace();
        break;
    }
    case LOST:
    {
        break;
    }
    }
}

void SimpleVO::extractKeyPoints()
{
    std::vector<cv::KeyPoint> key_points_curr;
    FAST(curr_->image, key_points_curr, fast_threshold_);
    KeyPoint::convert(key_points_curr, points_curr_);
    // if the tracked points's num is too small, re-detector the ref image's keypoints.
    if(points_ref_.size() < 500)
    {
        std::vector<cv::KeyPoint> key_points_ref;
        FAST(ref_->image, key_points_ref, fast_threshold_);
        KeyPoint::convert(key_points_ref, points_ref_);
    }
}

void SimpleVO::featureTracking()
{
    const double klt_win_size = 21.0;
	const int klt_max_iter = 30;
	const double klt_eps = 0.001;
	std::vector<uchar> status;
	std::vector<float> error;
	std::vector<float> min_eig_vec;
	cv::TermCriteria termcrit(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, klt_max_iter, klt_eps);
	cv::calcOpticalFlowPyrLK(ref_->image, curr_->image,
		points_ref_, points_curr_,
		status, error,
		cv::Size2i(klt_win_size, klt_win_size),
		4, termcrit, 0);

	std::vector<cv::Point2f>::iterator point_ref_it = points_ref_.begin();
	std::vector<cv::Point2f>::iterator point_cur_it = points_curr_.begin();
	disparities_.clear(); 
    disparities_.reserve(points_curr_.size());
	for (size_t i = 0; point_ref_it != points_ref_.end(); ++i)
	{
		if (!status[i])
		{
			point_ref_it = points_ref_.erase(point_ref_it);
			point_cur_it = points_curr_.erase(point_cur_it);
			continue;
		}
		disparities_.push_back(norm(Point2d(point_ref_it->x - point_cur_it->x, point_ref_it->y - point_cur_it->y)));
		++point_ref_it;
		++point_cur_it;
    }
}

void SimpleVO::updateStates()
{
    ref_ = curr_;
    points_ref_ = points_curr_;
}

void SimpleVO::readConfigFile()
{
    FileStorage fs(config_file_name_.c_str(), FileStorage::READ);
    ROS_ASSERT(fs.isOpened());

    float k_data[9] = {(float)fs["camera.fx"], 0, (float)fs["camera.cx"],
                      0, (float)fs["camera.fy"], (float)fs["camera.cy"],
                      0, 0, 1};
    // the array 'k_data' will be release out of the function, so we need to clone the mat's data.
    cam_intrinsic_ = Mat(3, 3, DataType<float>::type, k_data).clone(); 
    fs.release();
}

void SimpleVO::poseEstimation()
{
    Mat essential_matrix = findEssentialMat(points_ref_, points_curr_, cam_intrinsic_);
    // if(!essential_matrix.isContinuous())
    //     return;
    ROS_ASSERT(essential_matrix.isContinuous());
    Mat cur_R, cur_t;
    recoverPose(essential_matrix, points_ref_, points_curr_, cam_intrinsic_, cur_R, cur_t);
    // cout << cur_t.cols << ", " << cur_t.rows << endl;
    // Eigen::Matrix3f R_eigen;
    // cv2eigen(R, R_eigen);
    // Eigen::Vector3f euler_angle = R_eigen.eulerAngles(0, 1, 2);
    // cout << euler_angle.y() / M_PI * 180 << endl;

    double scale = 1;
    odometry_t = odometry_t + scale * (odometry_R * cur_t);
    odometry_R = odometry_R * cur_R;
    // cout << odometry_R << endl;
}

void SimpleVO::drawTrace()
{
    char text[100];
    int font_face = cv::FONT_HERSHEY_PLAIN;
	double font_scale = 1;
	int thickness = 1;
	cv::Point text_org(10, 50);

	double x=0.0, y=0.0,z=0.0;

    if(odometry_t.rows!=0)
    {
        x = odometry_t.at<double>(0);
        y = odometry_t.at<double>(1);
        z = odometry_t.at<double>(2);
    }
    cout << x << " " << y << " " << z << std::endl;

    int draw_x = int(-x) + 300;
    int draw_y = int(-z) + 100;
    cv::circle(traj_, cv::Point(draw_x, draw_y), 1, CV_RGB(255, 0, 0), 2);

    cv::rectangle(traj_, cv::Point(10, 30), cv::Point(580, 60), CV_RGB(0, 0, 0), CV_FILLED);
    sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", x, y, z);
    cv::putText(traj_, text, text_org, font_face, font_scale, cv::Scalar::all(255), thickness, 8);

    cv::imshow("Road facing camera", curr_->image);
    cv::imshow("Trajectory", traj_);

    cv::waitKey(1);
}