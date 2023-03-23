//
// Created by bhbhchoi on 23. 2. 2.
//
#include <ros/ros.h>

#include "laserMapping/laserMappingOption.h"
#include "laser_mapping_node/laser_mapping_node.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "laser_mapping_node");
    float lineRes = 0.4;
	float planeRes = 0.8;
	// nh.param<float>("mapping_line_resolution", lineRes, 0.4);
	// nh.param<float>("mapping_plane_resolution", planeRes, 0.8);
    auto laser_mapping_node = std::shared_ptr<laserMappingNode>(
        new laserMappingNode(lmOption(lineRes, planeRes, false)));
        
    laser_mapping_node->run();
    ros::shutdown();
    return 0;
}