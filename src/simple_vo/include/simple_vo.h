/*
 * Created on Wed Nov 21 2018
 *
 * Copyright (c) 2018 EpsAvlc
 * All rights reserved
 *
 * Author: EpsAvlc(https://github.com/EpsAvlc)
 */

#include <ros/ros.h>

#ifndef SIMPLE_VO_SIMPLE_VO_H_
#define SIMPLE_VO_SIMPLE_VO_H_

namespace simple_vo
{
class SimpleVO
{
public:
    SimpleVO(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
private:
    ros::NodeHandle nh_, nh_local_;
};
}

#endif // !SIMPLE_VO_SIMPLE_VO_H_

