//
// Created by bhbhchoi on 23. 1. 26.
//
#include <ros/ros.h>

#include "scanRegistration/scanRegistrationOption.h"
#include "scan_registration_wrapper/scan_registration_wrapper.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "scan_registration_node");
    scanRegistrationOption option("VLP16", true);
    auto scan_registration_node = std::make_shared<scanRegistrationWrapper>(option);
    ros::spin();
    ros::shutdown();
    return 0;
}