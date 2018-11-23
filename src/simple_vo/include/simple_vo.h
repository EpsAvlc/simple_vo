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
    void computeDescriptors();
    void featureMatching();
    void updateStates();

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
    cv::Ptr<cv::ORB> orb_;
    std::vector<cv::KeyPoint> key_points_curr_;
    std::vector<cv::KeyPoint> key_points_ref_;
    cv::Mat descriptors_curr_;
    cv::Mat descriptors_ref_;
    std::vector<cv::DMatch> feature_matches_;

    int num_of_features_;
    double scale_factor_;
    int level_pyramid_;
    double match_ratio_;
};
}

#endif // !SIMPLE_VO_SIMPLE_VO_H_

