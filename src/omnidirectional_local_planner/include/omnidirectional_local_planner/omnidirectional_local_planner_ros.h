#ifndef SRC_OMNIDIRECTIONAL_LOCAL_PLANNER_ROS_H
#define SRC_OMNIDIRECTIONAL_LOCAL_PLANNER_ROS_H

#include <ros/ros.h>
#include <angles/angles.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <boost/shared_ptr.hpp>

#include <iostream>
#include <array>
#include <cmath>
#include <vector>

#include "omnidirectional_local_planner/omnidirectional_local_planner.h"

#define _DEBUG_LOCAL_PLANNER_ROS
#define _SAVE_DATA_CSV "/home/sousarbarb/catkin_ws/log/5dpo/omniplanner-log_"

namespace omnidirectional_local_planner {

class OmnidirectionalLocalPlannerROS : public nav_core::BaseLocalPlanner {
private:
  bool initialized_;
  tf2_ros::Buffer* tf_;
  boost::shared_ptr<OmnidirectionalLocalPlanner> omni_ctrl_;

  base_local_planner::OdometryHelperRos odom_helper_;
  base_local_planner::LocalPlannerUtil planner_util_;

  geometry_msgs::PoseStamped current_pose_;
  costmap_2d::Costmap2DROS* costmap_ros_;

  ros::Publisher pb_global_plan_;
  ros::Publisher pb_local_plan_;
#ifdef _DEBUG_LOCAL_PLANNER_ROS
  ros::Publisher pb_pos, pb_pos_r, pb_pos_r_1d, pb_pos_r_2d;
  ros::Publisher pb_pos_loc, pb_pos_loc_r, pb_pos_loc_r_1d, pb_pos_loc_r_2d;
  ros::Publisher pb_pos_loc_e, pb_pos_loc_e_1d;
  ros::Publisher pb_vel, pb_vel_r, pb_vel_pd_r, pb_vel_ff_r;
  #ifdef _SAVE_DATA_CSV
  int index_data_ = 0;
  std::vector<std::array<double,3>> history_X_;
  std::vector<std::array<double,3>> history_X_r_;
  #endif
#endif

 public:
  OmnidirectionalLocalPlannerROS();
  ~OmnidirectionalLocalPlannerROS() override;
  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros) override;
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
  bool isGoalReached() override;

 private:
  void ConvertPoseStampedMsg2Array3d(geometry_msgs::PoseStamped& _pose,
                                     std::array<double,3>& _X);
  void PublishData();
  bool ReadParameters(const std::string& _name,
                      OmnidirectionalLocalPlannerConfig& _config);
  void SaveHistory();
};

}  // namespace omnidirectional_local_planner

#endif //SRC_OMNIDIRECTIONAL_LOCAL_PLANNER_ROS_H
