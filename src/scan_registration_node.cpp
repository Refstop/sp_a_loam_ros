//
// Created by bhbhchoi on 23. 1. 26.
//
#include <rclcpp/rclcpp.hpp>

#include "scanRegistration/scanRegistrationOption.h"
#include "scan_registration_wrapper/scan_registration_wrapper.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    scanRegistrationOption option("VLP16", true);
    auto scan_registration_node = std::make_shared<scanRegistrationWrapper>(option);
    rclcpp::spin(scan_registration_node);
    rclcpp::shutdown();
    return 0;
}