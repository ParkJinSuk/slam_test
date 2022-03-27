#include <iostream>
#include <ros/ros.h>

#include "tiny_slam/tiny_slam.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tinySLAM");  // ServiceTinySLAM 외부에서 ros::init 으로 변경
    ros::NodeHandle nh;

    ServiceTinySLAM* pServiceSLAM;
    pServiceSLAM = new ServiceTinySLAM(nh); // nodehandle 전달필요

    while(ros::ok())
    {
        pServiceSLAM->run();
        ros::Duration(0.2).sleep();

        pServiceSLAM->stop();
        ros::Duration(0.2).sleep();

        pServiceSLAM->reset();
        ros::Duration(0.2).sleep();
        
        ros::spinOnce();
        ros::Duration(0.001).sleep();
    }
    
    return 0;
}