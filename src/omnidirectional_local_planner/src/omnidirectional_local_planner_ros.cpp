#include "omnidirectional_local_planner/omnidirectional_local_planner_ros.h"

#include <base_local_planner/goal_functions.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>

PLUGINLIB_EXPORT_CLASS
(omnidirectional_local_planner::OmnidirectionalLocalPlannerROS, nav_core::BaseLocalPlanner)

namespace omnidirectional_local_planner {

OmnidirectionalLocalPlannerROS::OmnidirectionalLocalPlannerROS()
    : initialized_(false) , tf_(nullptr) , odom_helper_("odom") ,
      costmap_ros_(nullptr) { }

OmnidirectionalLocalPlannerROS::~OmnidirectionalLocalPlannerROS() = default;

void OmnidirectionalLocalPlannerROS::initialize(
    std::string name, tf2_ros::Buffer* tf,
    costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    ROS_INFO_NAMED(kLoggingName,"Initialization Procedure");

    // - read parameters
    ros::NodeHandle private_nh("~/" + name);
    OmnidirectionalLocalPlannerConfig config;
    if (!ReadParameters(name,config)) {
      ROS_FATAL_NAMED(kLoggingName,"Could not read all the parameters"
                                   " required from ROS Param Server");
      ros::requestShutdown();
      return;
    }

    // - publishers
    pb_global_plan_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
    pb_local_plan_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
#ifdef _DEBUG_LOCAL_PLANNER_ROS
    pb_pos          = private_nh.advertise<geometry_msgs::PoseStamped>(
        "X"         ,1);
    pb_pos_r        = private_nh.advertise<geometry_msgs::PoseStamped>(
        "X_r"       ,1);
    pb_pos_r_1d     = private_nh.advertise<geometry_msgs::PoseStamped>(
        "X_r_1d"    ,1);
    pb_pos_r_2d     = private_nh.advertise<geometry_msgs::PoseStamped>(
        "X_r_2d"    ,1);
    pb_pos_loc      = private_nh.advertise<geometry_msgs::PoseStamped>(
        "X_loc"     ,1);
    pb_pos_loc_r    = private_nh.advertise<geometry_msgs::PoseStamped>(
        "X_loc_r"   ,1);
    pb_pos_loc_r_1d = private_nh.advertise<geometry_msgs::PoseStamped>(
        "X_loc_r_1d",1);
    pb_pos_loc_r_2d = private_nh.advertise<geometry_msgs::PoseStamped>(
        "X_loc_r_2d",1);
    pb_pos_loc_e    = private_nh.advertise<geometry_msgs::PoseStamped>(
        "X_loc_e"   ,1);
    pb_pos_loc_e_1d = private_nh.advertise<geometry_msgs::PoseStamped>(
        "X_loc_e_1d",1);

    pb_vel      = private_nh.advertise<geometry_msgs::Twist>("V"     ,1);
    pb_vel_r    = private_nh.advertise<geometry_msgs::Twist>("V_r"   ,1);
    pb_vel_pd_r = private_nh.advertise<geometry_msgs::Twist>("V_pd_r",1);
    pb_vel_ff_r = private_nh.advertise<geometry_msgs::Twist>("V_ff_r",1);
#endif

    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ros_->getRobotPose(current_pose_);
    planner_util_.initialize(tf_,
                             costmap_ros_->getCostmap(),
                             costmap_ros_->getGlobalFrameID());

    // - create Omni Local Planner / Controller-only
    omni_ctrl_ = boost::shared_ptr<OmnidirectionalLocalPlanner>(
        new OmnidirectionalLocalPlanner(config,costmap_ros_,&current_pose_,
                                        &odom_helper_,&planner_util_));

    initialized_ = true;
  } else {
    ROS_WARN_NAMED(kLoggingName,"This planner has already been initialized");
  }
}

bool OmnidirectionalLocalPlannerROS::computeVelocityCommands(
    geometry_msgs::Twist& cmd_vel) {
  if (!initialized_) {
    ROS_WARN_NAMED(kLoggingName,"This planner has not been "
                                "initialized. Please call initialize() before "
                                "using this planner");
    return false;
  }

  if (!omni_ctrl_->SetRobotVelocity(cmd_vel)) {
    ROS_WARN_NAMED(kLoggingName,"Something went wrong when computing the "
                                "robot velocity");
    return false;
  }

  PublishData();

  return true;
}

bool OmnidirectionalLocalPlannerROS::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!initialized_) {
    ROS_WARN_NAMED(kLoggingName,"This planner has not been "
                                 "initialized. Please call initialize() before "
                                 "using this planner");
    return false;
  }

  ROS_INFO_NAMED(kLoggingName,"Local planner has a new plan");
  return omni_ctrl_->SetPlan(plan);
}

bool OmnidirectionalLocalPlannerROS::isGoalReached() {
  if (!initialized_) {
    ROS_WARN_NAMED(kLoggingName,"This planner has not been "
                                "initialized. Please call initialize() before "
                                "using this planner");
    return false;
  }

  if (!costmap_ros_->getRobotPose(current_pose_)) {
    ROS_ERROR_NAMED(kLoggingName,"Could not get current robot's pose");
    return false;
  }

  double x, y, th;
  if (!omni_ctrl_->GetCurrentGoal(x,y,th)) {
    ROS_ERROR_NAMED(kLoggingName,"Could not get current goal's pose");
    return false;
  }

  geometry_msgs::PoseStamped robot_vel;
  odom_helper_.getRobotVel(robot_vel);
  if (base_local_planner::getGoalPositionDistance(current_pose_,x,y) <=
      omni_ctrl_->config_.goal_tol_xy) {
    if (std::fabs(base_local_planner::getGoalOrientationAngleDifference(
            current_pose_,th)) <= omni_ctrl_->config_.goal_tol_th) {
      ROS_INFO_NAMED(kLoggingName, "Goal ( %lf , %lf , %lf ) reached!",
                     x, y, th);
#ifdef _DEBUG_LOCAL_PLANNER_ROS
  #ifdef _SAVE_DATA_CSV
      SaveHistory();
  #endif
#endif
      return true;
    }
  }

  return false;
}

void OmnidirectionalLocalPlannerROS::ConvertPoseStampedMsg2Array3d(
    geometry_msgs::PoseStamped& _pose, std::array<double, 3>& _X) {
  _X[0] = _pose.pose.position.x;
  _X[1] = _pose.pose.position.y;
  _X[2] = tf2::getYaw(_pose.pose.orientation);
}

void OmnidirectionalLocalPlannerROS::PublishData() {
  // Global and local paths
  nav_msgs::Path global_plan;
  nav_msgs::Path local_plan;

  omni_ctrl_->GetTrajectoryPath(global_plan);
  omni_ctrl_->GetFutureTrajectoryPath(local_plan);

  pb_global_plan_.publish(global_plan);
  pb_local_plan_.publish(local_plan);

#ifdef _DEBUG_LOCAL_PLANNER_ROS
  // Robot poses
  geometry_msgs::PoseStamped pos, pos_r, pos_r_1d, pos_r_2d;
  geometry_msgs::PoseStamped pos_loc, pos_loc_r, pos_loc_r_1d, pos_loc_r_2d;
  geometry_msgs::PoseStamped pos_loc_e, pos_loc_e_1d;
  omni_ctrl_->GetRobotPoses(pos, pos_r, pos_r_1d, pos_r_2d,
                            pos_loc, pos_loc_r, pos_loc_r_1d, pos_loc_r_2d,
                            pos_loc_e, pos_loc_e_1d);

  pb_pos.publish(pos);
  pb_pos_r.publish(pos_r);
  pb_pos_r_1d.publish(pos_r_1d);
  pb_pos_r_2d.publish(pos_r_2d);
  pb_pos_loc.publish(pos_loc);
  pb_pos_loc_r.publish(pos_loc_r);
  pb_pos_loc_r_1d.publish(pos_loc_r_1d);
  pb_pos_loc_r_2d.publish(pos_loc_r_2d);
  pb_pos_loc_e.publish(pos_loc_e);
  pb_pos_loc_e_1d.publish(pos_loc_e_1d);

  // Robot velocities
  geometry_msgs::Twist vel, vel_r;
  geometry_msgs::Twist vel_pd_r, vel_ff_r;
  omni_ctrl_->GetRobotVelocities(vel, vel_r, vel_pd_r, vel_ff_r);

  pb_vel.publish(vel);
  pb_vel_r.publish(vel_r);
  pb_vel_pd_r.publish(vel_pd_r);
  pb_vel_ff_r.publish(vel_ff_r);

  // Save data history to save later in a CSV file
  #ifdef _SAVE_DATA_CSV
  std::array<double,3> X{}, X_r{};

  ConvertPoseStampedMsg2Array3d(pos,X);
  ConvertPoseStampedMsg2Array3d(pos_r,X_r);

  history_X_.push_back(X);
  history_X_r_.push_back(X_r);
  #endif
#endif
}

bool OmnidirectionalLocalPlannerROS::ReadParameters(
    const std::string& _name, OmnidirectionalLocalPlannerConfig &_config) {
  ros::NodeHandle private_nh("~/");

  // Create C++ lambdas
  auto ReadRequiredParameter = [](ros::NodeHandle& _nh,
                                  const std::string& _param_name,
                                  auto& _param_value) -> bool {
    if (!_nh.getParam(_param_name,
                      _param_value)) {
      ROS_ERROR_NAMED(kLoggingName,"Parameter %s not found",
                      _param_name.c_str());
      return false;
    } else {
      ROS_INFO_NAMED(kLoggingName,"Parameter %s loaded (value: %s)",
                     _param_name.c_str(),std::to_string(_param_value).c_str());
      return true;
    }
  };
  auto ReadNotRequiredParameter = [](ros::NodeHandle& _nh,
                                     const std::string& _param_name,
                                     auto& _param_value,
                                     const auto& _param_default) {
    std::ostringstream param_value_str;
    if (!_nh.param(_param_name,_param_value,_param_default)) {
      param_value_str << _param_value;
      ROS_INFO_NAMED(kLoggingName,
                     "Parameter %s loaded (default value: %s)",
                     _param_name.c_str(),
                     param_value_str.str().c_str());
    } else {
      param_value_str << _param_value;
      ROS_INFO_NAMED(kLoggingName,
                     "Parameter %s loaded (value: %s)",
                     _param_name.c_str(),
                     param_value_str.str().c_str());
    }
  };

  // Get parameters from ROS Param Server
  // - controller frequency
  if (!ReadRequiredParameter(
          private_nh,
          "controller_frequency",
          _config.ctrl_freq))
    return false;

  // - conversion u into t
  double nom_vel;
  ReadNotRequiredParameter(
      private_nh,
      _name+"/nom_lin_vel",
      nom_vel, kNomVelocityLin);
  double map_res;
  if (!ReadRequiredParameter(
          private_nh,
          "global_costmap/resolution",
          map_res))
    return false;
  _config.SetConvU2T(nom_vel,map_res);

  // - future trajectory parameters
  ReadNotRequiredParameter(
      private_nh,
      _name+"/buffer_future_size",
      _config.buffer_future_size, kBufferFutureSize);

  // - activation of ff controllers
  ReadNotRequiredParameter(
      private_nh,
      _name+"/ff_ctrl_on",
      _config.ff_ctrl_on, true);

  // - maximum accelerations
  ReadNotRequiredParameter(
      private_nh,
      _name+"/acc_lim_x",
      _config.max_acc_lim[0], kMaxAcceleration[0]);
  ReadNotRequiredParameter(
      private_nh,
      _name+"/acc_lim_y",
      _config.max_acc_lim[1], kMaxAcceleration[1]);
  ReadNotRequiredParameter(
      private_nh,
      _name+"/acc_lim_th",
      _config.max_acc_lim[2], kMaxAcceleration[2]);

  // - maximum velocities
  ReadNotRequiredParameter(
      private_nh,
      _name+"/max_vel_x",
      _config.max_vel_lim[0], kMaxVelocity[0]);
  ReadNotRequiredParameter(
      private_nh,
      _name+"/max_vel_y",
      _config.max_vel_lim[1], kMaxVelocity[1]);
  ReadNotRequiredParameter(
      private_nh,
      _name+"/max_rot_vel",
      _config.max_vel_lim[2], kMaxVelocity[2]);

  // - goal tolerances
  ReadNotRequiredParameter(
      private_nh,
      _name+"/xy_goal_tolerance",
      _config.goal_tol_xy, kXYGoalTolerance);
  ReadNotRequiredParameter(
      private_nh,
      _name+"/yaw_goal_tolerance",
      _config.goal_tol_th, kThGoalTolerance);

  // - odometry topic
  std::string odom_topic;
  ReadNotRequiredParameter(
      private_nh,
      _name+"/odom_topic",
      odom_topic, std::string("odom"));
  odom_helper_.setOdomTopic(odom_topic);

  // - PD controller gains
  ReadNotRequiredParameter(
      private_nh,
      _name+"/pd_ctrl_kc_x",
      _config.pd_ctrl_kc[0], kPDCtrlKc[0]);
  ReadNotRequiredParameter(
      private_nh,
      _name+"/pd_ctrl_kc_y",
      _config.pd_ctrl_kc[1], kPDCtrlKc[1]);
  ReadNotRequiredParameter(
      private_nh,
      _name+"/pd_ctrl_kc_th",
      _config.pd_ctrl_kc[2], kPDCtrlKc[2]);
  ReadNotRequiredParameter(
      private_nh,
      _name+"/pd_ctrl_td_x",
      _config.pd_ctrl_td[0], kPDCtrlTd[0]);
  ReadNotRequiredParameter(
      private_nh,
      _name+"/pd_ctrl_td_y",
      _config.pd_ctrl_td[1], kPDCtrlTd[1]);
  ReadNotRequiredParameter(
      private_nh,
      _name+"/pd_ctrl_td_th",
      _config.pd_ctrl_td[2], kPDCtrlTd[2]);

  // 1st-order velocity model of the robot
  ReadNotRequiredParameter(
      private_nh,
      _name+"/robot_vel_kp_x",
      _config.ff_ctrl_kp[0], kFFCtrlKp[0]);
  ReadNotRequiredParameter(
      private_nh,
      _name+"/robot_vel_kp_y",
      _config.ff_ctrl_kp[1], kFFCtrlKp[1]);
  ReadNotRequiredParameter(
      private_nh,
      _name+"/robot_vel_kp_th",
      _config.ff_ctrl_kp[2], kFFCtrlKp[2]);
  ReadNotRequiredParameter(
      private_nh,
      _name+"/robot_vel_tau_x",
      _config.ff_ctrl_tau[0], kFFCtrlTau[0]);
  ReadNotRequiredParameter(
      private_nh,
      _name+"/robot_vel_tau_y",
      _config.ff_ctrl_tau[1], kFFCtrlTau[1]);
  ReadNotRequiredParameter(
      private_nh,
      _name+"/robot_vel_tau_th",
      _config.ff_ctrl_tau[2], kFFCtrlTau[2]);


  // Parameters read successfully
  ROS_INFO_NAMED(kLoggingName,"Parameters loaded successfully");
  return true;
}

void OmnidirectionalLocalPlannerROS::SaveHistory() {
  std::ostringstream filename;
  filename << _SAVE_DATA_CSV << index_data_ << ".csv";

  std::ofstream file;
  file.open(filename.str(), std::ios::out | std::ios::trunc);
  file << "i,X_r(x),X_r(y),X_r(th),X(x),X(y),X(th)" << std::endl;
  for (size_t i=0; i<history_X_.size(); i++)
    file << i << ","
         << history_X_r_[i][0]  << ","
         << history_X_r_[i][1]  << ","
         << history_X_r_[i][2]  << ","
         << history_X_[i][0]  << ","
         << history_X_[i][1]  << ","
         << history_X_[i][2]  << ","
         << std::endl;
  file.close();

  index_data_++;
}

}  // namespace omnidirectional_local_planner
