#include <ros/ros.h>
#include "LocalizationServer.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "genz_localization");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    genz_icp_ros::LocalizationServer node(nh, nh_private);

    ros::spin();
    return 0;
}