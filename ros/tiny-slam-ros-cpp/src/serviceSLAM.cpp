#include <iostream>
#include <ros/ros.h>

#include "tiny_slam/tiny_slam.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tinySLAM");  // ServiceTinySLAM 외부에서 ros::init 으로 변경
    ros::NodeHandle nh;

    ServiceTinySLAM serviceSLAM(nh);    // nodehandle 전달필요

    while(ros::ok())
    {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        ROS_INFO("nothing");
    }
    
    return 0;
}