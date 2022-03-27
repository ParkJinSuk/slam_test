#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <memory>

#include <nav_msgs/OccupancyGrid.h>

//#define RVIZ_DEBUG 1

#include "../core/sensor_data.h"
#include "../ros/topic_with_transform.h"
#include "../ros/rviz_grid_viewer.h"
#include "../ros/utils.h"
#include "../core/maps/area_occupancy_estimator.h"
#include "../core/maps/const_occupancy_estimator.h"
#include "../core/maps/grid_cell_strategy.h"
#include "tiny_fascade.h"
#include "tiny_world.h"
#include "tiny_grid_cells.h"

class ServiceTinySLAM
{
public:
    ServiceTinySLAM(ros::NodeHandle nh);
    ~ServiceTinySLAM();

    void run();
    void stop();
    void reset();

private: // init function
    std::shared_ptr<GridCellFactory> init_cell_factory(TinyWorldParams &params);
    std::shared_ptr<CellOccupancyEstimator> init_occ_estimator();
    bool init_skip_exceeding_lsr();
    TinyWorldParams init_common_world_params();
    GridMapParams init_grid_map_params();
    void init_constants_for_ros(double &ros_tf_buffer_size,
                                double &ros_map_rate,
                                int &ros_filter_queue,
                                int &ros_subscr_queue);
    void init_frame_names(std::string &frame_odom, std::string &frame_robot_pose);

    void init_tiny_slam();  // tiny_slam.cpp main에 있는 소스
    ros::NodeHandle nh_;
};