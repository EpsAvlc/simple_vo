/*
 * Created on Wed Nov 21 2018
 *
 * Copyright (c) 2018 EpsAvlc
 * All rights reserved
 *
 * Author: EpsAvlc(https://github.com/EpsAvlc)
 */

#include <ros/ros.h>

#include "simple_vo.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_vo_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");

    simple_vo::SimpleVO svo(nh, nh_local);
    ros::spin();
}
