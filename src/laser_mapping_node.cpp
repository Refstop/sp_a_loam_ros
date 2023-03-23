#include "laser_mapping_node/laser_mapping_node.h"

laserMappingNode::laserMappingNode(lmOption option)
    :laser_mapping_handler_(std::make_unique<laserMapping>(option)) {
    sub_laser_cloud_corner_last_ = nh_.subscribe("laser_cloud_corner_last_sp", 100,
                                        &laserMappingNode::laserCloudCornerLastHandler, this);
    sub_laser_cloud_surf_last_ = nh_.subscribe("laser_cloud_surf_last_sp", 100,
                                        &laserMappingNode::laserCloudSurfLastHandler, this);
    sub_laser_cloud_full_res_ = nh_.subscribe("velodyne_cloud_3_sp", 100,
                                        &laserMappingNode::laserCloudFullResHandler, this);
    sub_laser_odometry_ = nh_.subscribe("laser_odom_to_init_sp", 100,
                                        &laserMappingNode::laserOdometryHandler, this);

    pub_laser_cloud_surround_ = nh_.advertise<sensor_msgs::PointCloud2>("laser_cloud_surround_sp", 100);
    pub_laser_cloud_map_ = nh_.advertise<sensor_msgs::PointCloud2>("laser_cloud_map_sp", 100);
    pub_laser_cloud_full_res_ = nh_.advertise<sensor_msgs::PointCloud2>("velodyne_cloud_registered_sp", 100);
    pub_odom_aft_mapped_ = nh_.advertise<nav_msgs::Odometry>("aft_mapped_to_init_sp", 100);
    pub_odom_aft_mapped_high_frec_ = nh_.advertise<nav_msgs::Odometry>("aft_mapped_to_init_high_frec_sp", 100);
    pub_laser_after_mapped_path_ = nh_.advertise<sensor_msgs::PointCloud2>("aft_mapped_path_sp", 100);
    pub_laser_cloud_full_res_local_ = nh_.advertise<sensor_msgs::PointCloud2>("velodyne_cloud_registered_local_sp", 100);
}

void laserMappingNode::laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &laser_cloud_corner_last) {
    pcl::PointCloud<PointType> corner_last;
    pcl::fromROSMsg(*laser_cloud_corner_last, corner_last);
    laser_mapping_handler_->setCornerLast(laser_cloud_corner_last->header.stamp.toSec(), corner_last);
}
void laserMappingNode::laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &laser_cloud_surf_last) {
    pcl::PointCloud<PointType> surf_last;
    pcl::fromROSMsg(*laser_cloud_surf_last, surf_last);
    laser_mapping_handler_->setSurfLast(laser_cloud_surf_last->header.stamp.toSec(), surf_last);
}
void laserMappingNode::laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laser_cloud_full_res) {
    pcl::PointCloud<PointType> full_res;
    pcl::fromROSMsg(*laser_cloud_full_res, full_res);
    laser_mapping_handler_->setFullRes(laser_cloud_full_res->header.stamp.toSec(), full_res);
}

void laserMappingNode::laserOdometryHandler(const nav_msgs::OdometryConstPtr &laser_odometry) {

    Eigen::Quaterniond q_wodom_curr(
        laser_odometry->pose.pose.orientation.w,
        laser_odometry->pose.pose.orientation.x,
        laser_odometry->pose.pose.orientation.y,
        laser_odometry->pose.pose.orientation.z
    );
    Eigen::Vector3d t_wodom_curr{
        laser_odometry->pose.pose.orientation.x,
        laser_odometry->pose.pose.orientation.y,
        laser_odometry->pose.pose.orientation.z
    };
    
    Eigen::Quaterniond q_w_curr;
    Eigen::Vector3d t_w_curr;

    laser_mapping_handler_->setOdometry(laser_odometry->header.stamp.toSec(), q_wodom_curr, t_wodom_curr, q_w_curr, t_w_curr);
    
    nav_msgs::Odometry::Ptr odom_aft_mapped = odomToMsg(OdometryStamped::Ptr(
                                                new OdometryStamped(laser_odometry->header.stamp.toSec(), q_w_curr, t_w_curr)));
    pub_odom_aft_mapped_high_frec_.publish(*odom_aft_mapped);
    
}

nav_msgs::Odometry::Ptr laserMappingNode::odomToMsg(OdometryStamped::Ptr odometry) {
    nav_msgs::Odometry::Ptr odom = nav_msgs::Odometry::Ptr(new nav_msgs::Odometry());
    
    odom->header.frame_id = "camera_init";
    odom->child_frame_id = "aft_mapped";
    odom->header.stamp = ros::Time().fromSec(odometry->timeStamp);
    
    odom->pose.pose.orientation.x = odometry->q_odom.x();
    odom->pose.pose.orientation.y = odometry->q_odom.y();
    odom->pose.pose.orientation.z = odometry->q_odom.z();
    odom->pose.pose.orientation.w = odometry->q_odom.w();
    odom->pose.pose.position.x = odometry->t_odom.x();
    odom->pose.pose.position.y = odometry->t_odom.y();
    odom->pose.pose.position.z = odometry->t_odom.z();
    

    return odom;
}

void laserMappingNode::pclToMsg(const PointCloudStamped::Ptr laser_cloud, sensor_msgs::PointCloud2 &laser_cloud_out_msg) {
    pcl::toROSMsg(laser_cloud->laserCloud, laser_cloud_out_msg);
    laser_cloud_out_msg.header.stamp = ros::Time().fromSec(laser_cloud->timeStamp);
    laser_cloud_out_msg.header.frame_id = "camera_init";
}

void laserMappingNode::run() {
    int frame_count_ = 0;
    ros::Rate rate(100);
    while(ros::ok()) {
        ros::spinOnce();
        if(!laser_mapping_handler_->run())
            continue;
        
        if(frame_count_ % 5 == 0) {
            sensor_msgs::PointCloud2 laser_cloud_surround;
            pclToMsg(laser_mapping_handler_->getLaserCloudSurround(), laser_cloud_surround);
            pub_laser_cloud_surround_.publish(laser_cloud_surround);
        }
        
        if(frame_count_ % 20 == 0) {
            sensor_msgs::PointCloud2 laser_cloud_map;
            pclToMsg(laser_mapping_handler_->getLaserCloudMap(), laser_cloud_map);
            pub_laser_cloud_map_.publish(laser_cloud_map);
        }
        

        
        // A-LOAM과 SC-A-LOAM의 차이점: laserCloudFullRes3Local을 publish해준다.
		// laser_cloud_full_res_local은 현재 LiDAR 센서 기준으로 인식하는 pointcloud를 의미한다.
        sensor_msgs::PointCloud2 laser_cloud_full_res_local;
        pclToMsg(laser_mapping_handler_->getLaserCloudFullRes(), laser_cloud_full_res_local);
        pub_laser_cloud_full_res_local_.publish(laser_cloud_full_res_local);
        

        // laser_cloud_full_res는 world 좌표계 기준으로 현재 LiDAR가 인식한 pointcloud를 의미한다.
        sensor_msgs::PointCloud2 laser_cloud_full_res;
        pclToMsg(laser_mapping_handler_->getLaserCloudFullResGlobal(), laser_cloud_full_res);
        pub_laser_cloud_full_res_.publish(laser_cloud_full_res);
        

        
        Eigen::Quaterniond q_w_curr;
        Eigen::Vector3d t_w_curr;
        nav_msgs::Odometry::Ptr odom_aft_mapped = odomToMsg(laser_mapping_handler_->getOdometry());
        pub_odom_aft_mapped_.publish(odom_aft_mapped);
        

        geometry_msgs::PoseStamped laser_after_mapped_pose;
        laser_after_mapped_pose.header = odom_aft_mapped->header;
        laser_after_mapped_pose.pose = odom_aft_mapped->pose.pose;

        laser_after_mapped_path_.header.stamp = odom_aft_mapped->header.stamp;
        laser_after_mapped_path_.header.frame_id = "camera_init";
        laser_after_mapped_path_.poses.push_back(laser_after_mapped_pose);
        pub_laser_after_mapped_path_.publish(laser_after_mapped_path_);
        
        
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transform;

        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = "camera_init";
        transform.child_frame_id = "aft_mapped";
        transform.transform.translation.x = odom_aft_mapped->pose.pose.position.x;
        transform.transform.translation.y = odom_aft_mapped->pose.pose.position.y;
        transform.transform.translation.z = odom_aft_mapped->pose.pose.position.z;
        transform.transform.rotation.x = odom_aft_mapped->pose.pose.orientation.x;
        transform.transform.rotation.y = odom_aft_mapped->pose.pose.orientation.y;
        transform.transform.rotation.z = odom_aft_mapped->pose.pose.orientation.z;
        transform.transform.rotation.w = odom_aft_mapped->pose.pose.orientation.w;
        br.sendTransform(transform);

        
        frame_count_++;
    }
    // std::chrono::milliseconds dura(2);
    // std::this_thread::sleep_for(dura);
}