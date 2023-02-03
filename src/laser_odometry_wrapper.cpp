//
// Created by bhbhchoi on 23. 2. 2.
//
#include "laser_odometry_wrapper/laser_odometry_wrapper.h"

laserOdometryWrapper::laserOdometryWrapper(loOption option):
    laser_odometry_handler_(std::make_unique<laserOdometry>(option)),
    skip_frame_num_(option.skipFrameNum) {
    sub_corner_points_sharp_ = nh_.subscribe("laser_cloud_sharp_sp", 100,
                                             &laserOdometryWrapper::laserCloudSharpHandler, this);
    sub_corner_points_less_sharp_ = nh_.subscribe("laser_cloud_less_sharp_sp", 100,
                                                  &laserOdometryWrapper::laserCloudLessSharpHandler, this);
    sub_surf_points_flat_ = nh_.subscribe("laser_cloud_flat_sp", 100,
                                          &laserOdometryWrapper::laserCloudFlatHandler, this);
    sub_surf_points_less_flat_ = nh_.subscribe("laser_cloud_less_flat_sp", 100,
                                               &laserOdometryWrapper::laserCloudLessFlatHandler, this);
    sub_laser_cloud_full_res_ = nh_.subscribe("velodyne_cloud_2_sp", 100,
                                              &laserOdometryWrapper::laserCloudFullResHandler, this);

    pub_laser_cloud_corner_last_ = nh_.advertise<sensor_msgs::PointCloud2>("laser_cloud_corner_last_sp", 100);
    pub_laser_cloud_surf_last_= nh_.advertise<sensor_msgs::PointCloud2>("laser_cloud_surf_last_sp", 100);
    pub_laser_cloud_full_res_= nh_.advertise<sensor_msgs::PointCloud2>("velodyne_cloud_3_sp", 100);
    pub_laser_odometry_= nh_.advertise<nav_msgs::Odometry>("laser_odom_to_init_sp", 100);
    pub_laser_path_= nh_.advertise<nav_msgs::Path>("laser_odom_path_sp", 100);
}

void laserOdometryWrapper::laserCloudSharpHandler(const sensor_msgs::PointCloud2::ConstPtr& corner_points_sharp_msg) {
    pcl::PointCloud<PointType> corner_points_sharp;
    pcl::fromROSMsg(*corner_points_sharp_msg, corner_points_sharp);
    laser_odometry_handler_->setCornerSharpBuf(corner_points_sharp_msg->header.stamp.toSec(), corner_points_sharp);
}

void laserOdometryWrapper::laserCloudLessSharpHandler(const sensor_msgs::PointCloud2::ConstPtr& corner_points_less_sharp_msg) {
    pcl::PointCloud<PointType> corner_points_less_sharp;
    pcl::fromROSMsg(*corner_points_less_sharp_msg, corner_points_less_sharp);
    laser_odometry_handler_->setCornerPointsLessSharp(corner_points_less_sharp_msg->header.stamp.toSec(), corner_points_less_sharp);
}

void laserOdometryWrapper::laserCloudFlatHandler(const sensor_msgs::PointCloud2::ConstPtr& surf_points_flat_msg) {
    pcl::PointCloud<PointType> surf_points_flat;
    pcl::fromROSMsg(*surf_points_flat_msg, surf_points_flat);
    laser_odometry_handler_->setSurfPointsFlat(surf_points_flat_msg->header.stamp.toSec(), surf_points_flat);
}

void laserOdometryWrapper::laserCloudLessFlatHandler(const sensor_msgs::PointCloud2::ConstPtr& surf_points_less_flat_msg) {
    pcl::PointCloud<PointType> surf_points_less_flat;
    pcl::fromROSMsg(*surf_points_less_flat_msg, surf_points_less_flat);
    laser_odometry_handler_->setSurfPointsLessFlat(surf_points_less_flat_msg->header.stamp.toSec(), surf_points_less_flat);
}

void laserOdometryWrapper::laserCloudFullResHandler(const sensor_msgs::PointCloud2::ConstPtr& laser_cloud_full_res_msg) {
    pcl::PointCloud<PointType> laser_cloud_full_res;
    pcl::fromROSMsg(*laser_cloud_full_res_msg, laser_cloud_full_res);
    laser_odometry_handler_->setFullRes(laser_cloud_full_res_msg->header.stamp.toSec(), laser_cloud_full_res);
    full_res_timestamp_ = laser_cloud_full_res_msg->header.stamp;
}

void laserOdometryWrapper::pclToMsg(const PointTypeStamped::Ptr laser_cloud, sensor_msgs::PointCloud2 &laser_cloud_out_msg) {
    pcl::toROSMsg(laser_cloud->laserCloud, laser_cloud_out_msg);
    laser_cloud_out_msg.header.stamp = ros::Time().fromSec(laser_cloud->timeStamp);
    laser_cloud_out_msg.header.frame_id = "camera";
}

void laserOdometryWrapper::run() {
    int frame_count_ = 0;
    ros::Rate rate(100);
    while(ros::ok()) {
        ros::spinOnce();
        if(laser_odometry_handler_->run()) {
            nav_msgs::Odometry laser_odometry;
            laser_odometry.header.frame_id = "camera_init";
            laser_odometry.child_frame_id = "laser_odom";
            laser_odometry.header.stamp = full_res_timestamp_;
            laser_odometry.pose.pose.orientation.x = laser_odometry_handler_->getOdomRotation().x();
            laser_odometry.pose.pose.orientation.y = laser_odometry_handler_->getOdomRotation().y();
            laser_odometry.pose.pose.orientation.z = laser_odometry_handler_->getOdomRotation().z();
            laser_odometry.pose.pose.orientation.w = laser_odometry_handler_->getOdomRotation().w();
            laser_odometry.pose.pose.position.x = laser_odometry_handler_->getOdomTranslation().x();
            laser_odometry.pose.pose.position.y = laser_odometry_handler_->getOdomTranslation().y();
            laser_odometry.pose.pose.position.z = laser_odometry_handler_->getOdomTranslation().z();
            pub_laser_odometry_.publish(laser_odometry);

            geometry_msgs::PoseStamped laser_pose;
            laser_pose.header = laser_odometry.header;
            laser_pose.pose = laser_odometry.pose.pose;
            laser_path_.header.stamp = laser_odometry.header.stamp;
            laser_path_.poses.push_back(laser_pose);
            laser_path_.header.frame_id = "camera_init";
            pub_laser_path_.publish(laser_path_);


            if(frame_count_ % skip_frame_num_ == 0) {
                frame_count_ = 0;

                sensor_msgs::PointCloud2 laser_cloud_corner_last;
                pclToMsg(laser_odometry_handler_->getLaserCloudCornerLast(), laser_cloud_corner_last);
                pub_laser_cloud_corner_last_.publish(laser_cloud_corner_last);

                sensor_msgs::PointCloud2 laser_cloud_surf_last;
                pclToMsg(laser_odometry_handler_->getLaserCloudSurfLast(), laser_cloud_surf_last);
                pub_laser_cloud_surf_last_.publish(laser_cloud_surf_last);

                sensor_msgs::PointCloud2 laser_cloud_full_res;
                pclToMsg(laser_odometry_handler_->getLaserCloudFullRes(), laser_cloud_full_res);
                pub_laser_cloud_full_res_.publish(laser_cloud_full_res);

                frame_count_++;
            }
        }

        rate.sleep();
    }
}