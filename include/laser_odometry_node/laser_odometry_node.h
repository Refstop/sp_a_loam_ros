//
// Created by bhbhchoi on 23. 2. 2.
//

#ifndef SP_A_LOAM_ROS_LASER_ODOMETRY_NODE_H
#define SP_A_LOAM_ROS_LASER_ODOMETRY_NODE_H
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "common.h"
#include "laserOdometry/laserOdometry.h"

class laserOdometryNode {
public:
    laserOdometryNode(loOption option);
    void run();
private:
    void laserCloudSharpHandler(const sensor_msgs::PointCloud2::ConstPtr &corner_points_sharp_msg);
    void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2::ConstPtr &corner_points_less_sharp_msg);
    void laserCloudFlatHandler(const sensor_msgs::PointCloud2::ConstPtr &surf_points_flat_msg);
    void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2::ConstPtr &surf_points_less_flat_msg);
    void laserCloudFullResHandler(const sensor_msgs::PointCloud2::ConstPtr &laser_cloud_full_res_msg);
    void pclToMsg(const PointCloudStamped::Ptr laser_cloud, sensor_msgs::PointCloud2 &laser_cloud_out_msg);
    std::unique_ptr<laserOdometry> laser_odometry_handler_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_corner_points_sharp_, sub_corner_points_less_sharp_, sub_surf_points_flat_, sub_surf_points_less_flat_, sub_laser_cloud_full_res_;
    ros::Publisher pub_laser_cloud_corner_last_, pub_laser_cloud_surf_last_, pub_laser_cloud_full_res_;
    ros::Publisher pub_laser_odometry_, pub_laser_path_;

    int skip_frame_num_;
    nav_msgs::Path laser_path_;
    ros::Time full_res_timestamp_;
};

#endif //SP_A_LOAM_ROS_LASER_ODOMETRY_NODE_H
