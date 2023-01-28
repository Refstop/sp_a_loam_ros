//
// Created by bhbhchoi on 23. 1. 26.
//

#ifndef SP_A_LOAM_SCAN_REGISTRATION_WRAPPER_H
#define SP_A_LOAM_SCAN_REGISTRATION_WRAPPER_H
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "common.h"
#include "scanRegistration/scanRegistration.h"

//using std::placeholders::_1;
using namespace std::chrono_literals;

class scanRegistrationWrapper: public rclcpp::Node {
public:
    scanRegistrationWrapper(scanRegistrationOption option);
private:
    void laserCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr laser_cloud_msg);
    void pclToMsg(const pcl::PointCloud<PointType> &laser_cloud, sensor_msgs::msg::PointCloud2 &laser_cloud_out_msg, const builtin_interfaces::msg::Time timestamp);
    std::unique_ptr<scanRegistration> scan_registration_handler_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_laser_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_,
                                                                pub_corner_points_sharp_,
                                                                pub_corner_points_less_sharp_,
                                                                pub_surf_points_flat_,
                                                                pub_surf_points_less_flat_;


};

#endif //SP_A_LOAM_SCAN_REGISTRATION_WRAPPER_H
