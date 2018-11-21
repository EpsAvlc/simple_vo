/*
 * Created on Wed Nov 21 2018
 *
 * Copyright (c) 2018 EpsAvlc
 * All rights reserved
 *
 * Author: EpsAvlc(https://github.com/EpsAvlc)
 */

#include "simple_vo.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace simple_vo;
using namespace std;
using namespace cv;

SimpleVO::SimpleVO(ros::NodeHandle& nh, ros::NodeHandle& nh_local)
: nh_(nh), nh_local_(nh_local), state_(INITIALIZING)
{
    updateParam();
    orb_ = ORB::create(num_of_features_, scale_factor_, level_pyramid_);
    img_sub_ = nh_.subscribe("camera/image_raw", 1, &SimpleVO::callback, this);
}

SimpleVO::~SimpleVO()
{
    nh_local_.deleteParam("num_of_features");
    nh_local_.deleteParam("scale_factor");
    nh_local_.deleteParam("level_pyramid");
    nh_local_.deleteParam("match_ratio");
}

void SimpleVO::updateParam()
{
    nh_local_.param<int>("num_of_features", num_of_features_, 500);
    nh_local_.param<double>("scale_factor", scale_factor_, 1.2);
    nh_local_.param<int>("level_pyramid", level_pyramid_, 8);
    nh_local_.param<double>("match_ratio", match_ratio_, 2.0);
}

void SimpleVO::callback(const sensor_msgs::ImageConstPtr img_ptr)
{
    cv_bridge::CvImageConstPtr cv_img_ptr = cv_bridge::toCvShare(img_ptr);
    addFrame(cv_img_ptr);
}

void SimpleVO::addFrame(const cv_bridge::CvImageConstPtr img_ptr)
{
    switch(state_)
    {
    case INITIALIZING:
    {
        state_ = OK;
        curr_ = ref_ = img_ptr;
        extractKeyPoints();
        // compute the 3d position of features in ref frame
        computeDescriptors();
        break;
    }
    case OK:
    {
        curr_ = img_ptr;
        extractKeyPoints();
        computeDescriptors();
        featureMatching();
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
    orb_->detect(curr_->image, key_points_curr_);
}

void SimpleVO::computeDescriptors()
{
    descriptors_ref_ = descriptors_curr_.clone();
    key_points_ref_.swap(key_points_curr_);
    orb_->compute(curr_->image, key_points_curr_, descriptors_curr_);
}

void SimpleVO::featureMatching()
{
    // match desp_ref and desp_curr, use OpenCV's brute force match 
    vector<cv::DMatch> matches;
    cv::BFMatcher matcher (cv::NORM_HAMMING);
    matcher.match (descriptors_ref_, descriptors_curr_, matches);
    ROS_ASSERT(matches.size() > 0);

    cout << std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance << endl;

    // TODO:Some bug comes here.
    cout << "I ENTERED" << endl;
    Mat match_res;

    // select the best matches
    float min_dis = std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;

    feature_matches_.clear();
    for ( cv::DMatch& m : matches )
    {
        if (m.distance < max<float>(min_dis * match_ratio_, 30.0))
        {
            feature_matches_.push_back(m);
        }
    }

    drawMatches(ref_->image, key_points_ref_, 
        curr_->image, key_points_curr_, feature_matches_, match_res);
    imshow("res", match_res);
    waitKey(0);
}