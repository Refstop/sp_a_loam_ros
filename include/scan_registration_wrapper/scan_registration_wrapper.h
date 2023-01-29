//
// Created by bhbhchoi on 23. 1. 26.
//

#ifndef SP_A_LOAM_ROS_SCAN_REGISTRATION_WRAPPER_H
#define SP_A_LOAM_ROS_SCAN_REGISTRATION_WRAPPER_H
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "common.h"
#include "scanRegistration/scanRegistration.h"

using namespace std::chrono_literals;

class scanRegistrationWrapper {
public:
    scanRegistrationWrapper(scanRegistrationOption option);
private:
    void laserCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& laser_cloud_msg);
    void pclToMsg(const pcl::PointCloud<PointType> &laser_cloud, sensor_msgs::PointCloud2 &laser_cloud_out_msg, const ros::Time timestamp);
    std::unique_ptr<scanRegistration> scan_registration_handler_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_laser_cloud_;
    ros::Publisher pub_laser_cloud_,
                   pub_corner_points_sharp_,
                   pub_corner_points_less_sharp_,
                   pub_surf_points_flat_,
                   pub_surf_points_less_flat_;
};

#endif //SP_A_LOAM_ROS_SCAN_REGISTRATION_WRAPPER_H
