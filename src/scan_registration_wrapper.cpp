//
// Created by bhbhchoi on 23. 1. 26.
//
#include "scan_registration_wrapper/scan_registration_wrapper.h"

scanRegistrationWrapper::scanRegistrationWrapper(scanRegistrationOption option):
    scan_registration_handler_(std::make_unique<scanRegistration>(std::move(option))) {
    sub_laser_cloud_ = nh_.subscribe(
            "velodyne_points", 100,
            &scanRegistrationWrapper::laserCloudCallback, this);
    pub_laser_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(
            "velodyne_cloud_2_sp", 100);
    pub_corner_points_sharp_ = nh_.advertise<sensor_msgs::PointCloud2>(
            "laser_cloud_sharp_sp", 100);
    pub_corner_points_less_sharp_ = nh_.advertise<sensor_msgs::PointCloud2>(
            "laser_cloud_less_sharp_sp", 100);
    pub_surf_points_flat_ = nh_.advertise<sensor_msgs::PointCloud2>(
            "laser_cloud_flat_sp", 100);
    pub_surf_points_less_flat_ = nh_.advertise<sensor_msgs::PointCloud2>(
            "laser_cloud_less_flat_sp", 100);
}

void scanRegistrationWrapper::laserCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& laser_cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ> laser_cloud_in;
    pcl::fromROSMsg(*laser_cloud_msg, laser_cloud_in);

    scan_registration_handler_->laserCloudHandler(std::move(laser_cloud_in));

    sensor_msgs::PointCloud2 laser_cloud_out_msg;
    pclToMsg(scan_registration_handler_->getLaserCloud(), laser_cloud_out_msg, laser_cloud_msg->header.stamp);
    pub_laser_cloud_.publish(laser_cloud_out_msg);

    sensor_msgs::PointCloud2 corner_points_sharp_msg;
    pclToMsg(scan_registration_handler_->getCornerPointsSharp(), corner_points_sharp_msg, laser_cloud_msg->header.stamp);
    pub_corner_points_sharp_.publish(corner_points_sharp_msg);

    sensor_msgs::PointCloud2 corner_points_less_sharp_msg;
    pclToMsg(scan_registration_handler_->getCornerPointsLessSharp(), corner_points_less_sharp_msg, laser_cloud_msg->header.stamp);
    pub_corner_points_less_sharp_.publish(corner_points_less_sharp_msg);

    sensor_msgs::PointCloud2 surf_points_flat2;
    pclToMsg(scan_registration_handler_->getSurfPointsFlat(), surf_points_flat2, laser_cloud_msg->header.stamp);
    pub_surf_points_flat_.publish(surf_points_flat2);

    sensor_msgs::PointCloud2 surf_points_less_flat2;
    pclToMsg(scan_registration_handler_->getSurfPointsLessFlat(), surf_points_less_flat2, laser_cloud_msg->header.stamp);
    pub_surf_points_less_flat_.publish(surf_points_less_flat2);
}

void scanRegistrationWrapper::pclToMsg(const pcl::PointCloud<PointType>::Ptr laser_cloud, sensor_msgs::PointCloud2 &laser_cloud_out_msg, const ros::Time timestamp) {
    pcl::toROSMsg(*laser_cloud, laser_cloud_out_msg);
    laser_cloud_out_msg.header.stamp = timestamp;
    laser_cloud_out_msg.header.frame_id = "camera_init";
}