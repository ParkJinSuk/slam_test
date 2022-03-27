#include "tiny_slam.h"

ServiceTinySLAM::ServiceTinySLAM(int argc, char** argv)
{
  init_tiny_slam(argc, argv);
}

ServiceTinySLAM::~ServiceTinySLAM()
{
  //
}

/*!
 * Determines the cell factory based on parameters came from a launch file.
 * \param[in] params - values from the launch file.
 * \return The pointer (shared) to a created factory of grid cells.
 */
std::shared_ptr<GridCellFactory> ServiceTinySLAM::init_cell_factory(TinyWorldParams &params) {
  std::string cell_type;
  ros::param::param<std::string>("~cell_type", cell_type, "avg");

  if (cell_type == "base") {
    params.localized_scan_quality = 0.2;
    params.raw_scan_quality = 0.1;
    return std::shared_ptr<GridCellFactory>{new TinyBaseCellFactory()};
  } else if (cell_type == "avg") {
    params.localized_scan_quality = 0.9;
    params.raw_scan_quality = 0.6;
    return std::shared_ptr<GridCellFactory>{new TinyAvgCellFactory()};
  } else {
    std::cerr << "Unknown cell type: " << cell_type << std::endl;
    std::exit(-1);
  }
}

/*!
 * Determines the estimator based on parameters came from a launch file.
 * \param[in] params - values from a launch file.
 * \return The pointer (shared) to a created estimator of a map cost.
 */
std::shared_ptr<CellOccupancyEstimator> ServiceTinySLAM::init_occ_estimator() {
  double occ_prob, empty_prob;
  ros::param::param<double>("~base_occupied_prob", occ_prob, 0.95);
  ros::param::param<double>("~base_empty_prob", empty_prob, 0.01);

  using OccEstPtr = std::shared_ptr<CellOccupancyEstimator>;
  std::string est_type;
  ros::param::param<std::string>("~occupancy_estimator", est_type, "const");

  if (est_type == "const") {
    return OccEstPtr{new ConstOccupancyEstimator(occ_prob, empty_prob)};
  } else if (est_type == "area") {
    return OccEstPtr{new AreaOccupancyEstimator(occ_prob, empty_prob)};
  } else {
    std::cerr << "Unknown estimator type: " << est_type << std::endl;
    std::exit(-1);
  }
}

/*!
 * Returns how to deal with exceeding values based on parameters came
 * from a launch file.
 */
bool ServiceTinySLAM::init_skip_exceeding_lsr() {
  bool param_value;
  ros::param::param<bool>("~skip_exceeding_lsr_vals", param_value, false);
  return param_value;
}

/**
 * Initializes constants for scan matcher
 * \return The structure contains requied paramteres
 */
TinyWorldParams ServiceTinySLAM::init_common_world_params() {
  double sig_XY, sig_T, width;
  int lim_bad, lim_totl;
  ros::param::param<double>("~scmtch_sigma_XY_MonteCarlo", sig_XY, 0.2);
  ros::param::param<double>("~scmtch_sigma_theta_MonteCarlo", sig_T, 0.1);
  ros::param::param<int>("~scmtch_limit_of_bad_attempts", lim_bad, 20);
  ros::param::param<int>("~scmtch_limit_of_total_attempts", lim_totl, 100);
  ros::param::param<double>("~hole_width", width,1.5);

  return TinyWorldParams(sig_XY, sig_T, lim_bad, lim_totl, width);
}

/**
 * Initializes constants for map
 * \return The structure contains requied paramteres
 */
GridMapParams ServiceTinySLAM::init_grid_map_params() {
  GridMapParams params;
  ros::param::param<double>("~map_height_in_meters", params.height, 20);
  ros::param::param<double>("~map_width_in_meters", params.width, 20);
  ros::param::param<double>("~map_meters_per_cell", params.meters_per_cell,
                                                                         0.1);
  return params;
}

/**
 * Initializes constants for ros utils
 * \return Requied parameters
 */
void ServiceTinySLAM::init_constants_for_ros(double &ros_tf_buffer_size,
                            double &ros_map_rate,
                            int &ros_filter_queue,
                            int &ros_subscr_queue) {
  ros::param::param<double>("~ros_tf_buffer_duration",ros_tf_buffer_size,5.0);
  ros::param::param<double>("~ros_rviz_map_publishing_rate", ros_map_rate, 5.0);
  ros::param::param<int>("~ros_filter_queue_size",ros_filter_queue,1000);
  ros::param::param<int>("~ros_subscribers_queue_size",ros_subscr_queue,1000);
}

void ServiceTinySLAM::init_frame_names(std::string &frame_odom, std::string &frame_robot_pose) {
  ros::param::param<std::string>("~odom", frame_odom, "odom_combined");
  ros::param::param<std::string>("~robot_pose", frame_robot_pose, "robot_pose");
}

/*!
 * The entry point: creates an environment world and the main node "tiny slam".
 */
void ServiceTinySLAM::init_tiny_slam(int argc, char** argv)
{
  ros::init(argc, argv, "tinySLAM");

  ros::NodeHandle nh;
  TinyWorldParams params = init_common_world_params();
  GridMapParams grid_map_params = init_grid_map_params();
  std::shared_ptr<ScanCostEstimator> cost_est{new TinyScanCostEstimator()};
  std::shared_ptr<GridCellStrategy> gcs{new GridCellStrategy{
    init_cell_factory(params), cost_est, init_occ_estimator()}};
  std::shared_ptr<TinySlamFascade> slam{new TinySlamFascade(gcs,
    params, grid_map_params, init_skip_exceeding_lsr())};

  double ros_map_publishing_rate, ros_tf_buffer_size;
  int ros_filter_queue, ros_subscr_queue;
  std::string frame_odom, frame_robot_pose;
  init_constants_for_ros(ros_tf_buffer_size, ros_map_publishing_rate,
                         ros_filter_queue, ros_subscr_queue);
  init_frame_names(frame_odom, frame_robot_pose);
  TopicWithTransform<sensor_msgs::LaserScan> scan_observer(nh,
    "laser_scan", frame_odom, ros_tf_buffer_size,
    ros_filter_queue, ros_subscr_queue);
  scan_observer.subscribe(slam);

  std::shared_ptr<RvizGridViewer> viewer(
    new RvizGridViewer(nh.advertise<nav_msgs::OccupancyGrid>("/map", 5),
                       ros_map_publishing_rate, frame_odom, frame_robot_pose));
  slam->set_viewer(viewer);
}