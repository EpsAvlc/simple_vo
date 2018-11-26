/*
 * Created on Wed Nov 21 2018
 *
 * Copyright (c) 2018 EpsAvlc
 * All rights reserved
 *
 * Author: EpsAvlc(https://github.com/EpsAvlc)
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#ifndef SIMPLE_VO_SIMPLE_VO_H_
#define SIMPLE_VO_SIMPLE_VO_H_

namespace simple_vo
{
class SimpleVO
{
public:
    SimpleVO(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
    ~SimpleVO();
private:
    void updateParam();
    void callback(const sensor_msgs::ImageConstPtr img_ptr);
    void addFrame(const cv_bridge::CvImageConstPtr img_ptr);
    void extractKeyPoints();
    void featureTracking();
    void updateStates();
    void readConfigFile();
    void poseEstimation();
    void drawTrace();

    enum VOState
    {
        INITIALIZING=-1,
        OK,
        LOST
    };

    VOState state_;
    ros::NodeHandle nh_, nh_local_;
    ros::Subscriber img_sub_;

    cv_bridge::CvImageConstPtr curr_, ref_;
    std::vector<cv::Point2f> points_curr_;
    std::vector<cv::Point2f> points_ref_;
    std::vector<double> disparities_;
    cv::Mat cam_intrinsic_;
    cv::Mat odometry_R;
    cv::Mat odometry_t;
    cv::Mat traj_;

    int fast_threshold_;
    double scale_factor_;
    int level_pyramid_;
    double match_ratio_;
    std::string config_file_name_;
};
}

#endif // !SIMPLE_VO_SIMPLE_VO_H_

