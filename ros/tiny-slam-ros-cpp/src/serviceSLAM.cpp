#include <iostream>
#include <ros/ros.h>

#include "tiny_slam/tiny_slam.h"

int main(int argc, char** argv)
{
    ServiceTinySLAM serviceSLAM(argc, argv);

    while(ros::ok())
    {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        ROS_INFO("ros info");
    }
    return 0;
}