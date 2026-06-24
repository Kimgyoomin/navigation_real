// /**
//  * @author SungJoon Yoon  (densee250@gmail.com)
//  * @version 1.0
//  * @date 2025-03-10
//  * @copyright Copyright (c) 2025, Robotics & Control Lab.
//  *
//  */

// ROS
#include "ros/ros.h"

#include "pongbot_heightmap_cupy/pongbot_heightmap_ros.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pongbot_heightmap_node");
    ros::NodeHandle nh("~");                    
    
    py::scoped_interpreter guard{};
    pongbot_heightmap_cupy::HeightMapNode mapNode(nh);
    py::gil_scoped_release release;

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}