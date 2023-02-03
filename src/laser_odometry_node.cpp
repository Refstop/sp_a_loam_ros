//
// Created by bhbhchoi on 23. 2. 2.
//
#include <ros/ros.h>

#include "laserOdometry/laserOdometryOption.h"
#include "laser_odometry_wrapper/laser_odometry_wrapper.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "laser_odometry_node");
    int skip_frame_num = 5;
    auto laser_odometry_node = std::shared_ptr<laserOdometryWrapper>(
            new laserOdometryWrapper(loOption(skip_frame_num, true)));
    ros::spin();
    ros::shutdown();
    return 0;
}