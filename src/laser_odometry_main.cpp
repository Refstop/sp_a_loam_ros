//
// Created by bhbhchoi on 23. 2. 2.
//
#include <ros/ros.h>

#include "laserOdometry/laserOdometryOption.h"
#include "laser_odometry_node/laser_odometry_node.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "laser_odometry_node");
    int skip_frame_num = 1;
    // nh.param<int>("mapping_skip_frame", skip_frame_num, 2);
    auto laser_odometry_node = std::shared_ptr<laserOdometryNode>(
            new laserOdometryNode(loOption(skip_frame_num, false)));
    laser_odometry_node->run();
    ros::shutdown();
    return 0;
}