//
// Created by bhbhchoi on 23. 1. 26.
//
#include <ros/ros.h>

#include "scanRegistration/scanRegistrationOption.h"
#include "scan_registration_node/scan_registration_node.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "scan_registration_node");
    srOption option("VLP16", true);
    auto scan_registration_node = std::make_shared<scanRegistrationNode>(option);
    ros::spin();
    ros::shutdown();
    return 0;
}