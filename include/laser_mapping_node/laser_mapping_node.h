#ifndef SP_A_LOAM_ROS_LASER_MAPPING_NODE_H
#define SP_A_LOAM_ROS_LASER_MAPPING_NODE_H
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include "common.h"
#include "laserMapping/laserMapping.h"

class laserMappingNode {
    public:
        laserMappingNode(lmOption option);
        void run();
    private:
        void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &laser_cloud_corner_last);
        void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &laser_cloud_surf_last);
        void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laser_cloud_full_res);
        void laserOdometryHandler(const nav_msgs::OdometryConstPtr &laser_odometry);
        void pclToMsg(const PointCloudStamped::Ptr laser_cloud, sensor_msgs::PointCloud2 &laser_cloud_out_msg);
        nav_msgs::Odometry::Ptr odomToMsg(const OdometryStamped::Ptr odometry);

        std::unique_ptr<laserMapping> laser_mapping_handler_;
        ros::NodeHandle nh_;
        ros::Publisher pub_laser_cloud_surround_, pub_laser_cloud_map_, pub_laser_cloud_full_res_, pub_odom_aft_mapped_,
                       pub_odom_aft_mapped_high_frec_, pub_laser_after_mapped_path_, pub_laser_cloud_full_res_local_;
        ros::Subscriber sub_laser_cloud_corner_last_, sub_laser_cloud_surf_last_, sub_laser_cloud_full_res_, sub_laser_odometry_;
        nav_msgs::Path laser_after_mapped_path_;
};

#endif //SP_A_LOAM_ROS_LASER_MAPPING_NODE_H