#include "omnidirectional_local_planner/omnidirectional_local_planner.h"

#include <base_local_planner/goal_functions.h>
#include <tf2/utils.h>

#include <Eigen/Eigen>

namespace omnidirectional_local_planner {

double NormalizeAngle(double _angle) {
  double norm_angle = _angle;

  while (norm_angle > M_PI)
    norm_angle -= 2*M_PI;
  while (norm_angle <= -M_PI)
    norm_angle += 2*M_PI;

  return norm_angle;
}

void RobotState::RotateGlobal2LocalFrame(Eigen::Vector3d& _X_global,
                                         Eigen::Vector3d& _X_local) {
  Eigen::AngleAxis<double> R_global2local =
      Eigen::AngleAxis<double>(-X(2), Eigen::Vector3d::UnitZ());

  _X_local = R_global2local * _X_global;
}

void RobotState::SetRobotVelocity(Eigen::Vector3d& _V,
                                  OmnidirectionalLocalPlannerConfig* _config) {
  // Check maximum acceleration limits
  for (int i=0; i<3; i++) {
    double max_vel_diff = _config->max_acc_lim[i] / _config->ctrl_freq;

    if (_V(i) - V(i) > max_vel_diff)
      V_r(i) = V(i) + max_vel_diff;
    else if (_V(i) - V(i) < -max_vel_diff)
      V_r(i) = V(i) - max_vel_diff;
    else
      V_r(i) = _V(i);
  }

  // Check maximum velocity limits
  for (int i=0; i<3; i++) {
    if (V_r(i) > _config->max_vel_lim[i])
      V_r(i) = _config->max_vel_lim[i];
    else if (V_r(i) < -_config->max_vel_lim[i])
      V_r(i) = -_config->max_vel_lim[i];
  }
}

Trajectory::Trajectory(OmnidirectionalLocalPlannerConfig *_config)
    : index_(0) , time_(0), config_(_config) {
  if (config_->buffer_future_size > 0)
    ResizeTrajectoryFutureMatrices();
  else
    ROS_WARN_NAMED(kLoggingName, "Buffer size not valid in the config struct");
}

void Trajectory::SetNewTrajectory(
    const std::vector<geometry_msgs::PoseStamped> &_plan) {
  // Assign plan/trajectory
  plan_ = _plan;

  // Reset indexes
  index_ = 0;
  time_ = 0;

  // Check size future trajectory buffer
  if (future_buffer_.rows() != config_->buffer_future_size)
    ResizeTrajectoryFutureMatrices();

  // Initialize linear least-squares matrices
  InitializeMatrices();

  // Initialize future trajectory buffer
  UpdateTrajectoryFutureBuffer(index_);
}

bool Trajectory::GetCurrentGoal(double& _x, double& _y, double& _th) {
  if (plan_.empty())
    return false;

  _x = plan_.back().pose.position.x;
  _y = plan_.back().pose.position.y;
  _th = tf2::getYaw(plan_.back().pose.orientation);

  return true;
}

int Trajectory::GetIndex() {
  return index_;
}

void Trajectory::GetTrajectoryPath(
    std::vector<geometry_msgs::PoseStamped> &_path) {
  // Set path poses
  _path = plan_;
}

void Trajectory::GetFutureTrajectoryPath(
    std::vector<geometry_msgs::PoseStamped> &_path,
    costmap_2d::Costmap2DROS* _costmap_ros) {
  _path.clear();

  // Set path poses
  size_t n = future_buffer_.rows();
  for (size_t i=0; i<n; i++) {
    geometry_msgs::PoseStamped p;

    p.header.frame_id = _costmap_ros->getGlobalFrameID();
    p.header.stamp = ros::Time::now();
    p.pose.position.x = future_buffer_(i,0);
    p.pose.position.y = future_buffer_(i,1);
    p.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0,0,future_buffer_(i,2));
    tf2::convert(q, p.pose.orientation);
    _path.push_back(p);
  }
}

bool Trajectory::GetPoseVelAcc(double _u, Eigen::Vector3d& _pose,
                               Eigen::Vector3d& _vel, Eigen::Vector3d& _acc) {
  // Error checking
  if (_u > config_->buffer_future_size-1) {
    ROS_WARN_NAMED(kLoggingName,"Invalid time instant to get a valid pose");
    return false;
  }

  // Pose
  size_t i_f = _u > 0? std::ceil(_u) : 1;
  size_t i_0 = i_f - 1;
  _pose =
      future_buffer_.row(i_0).transpose() +
      (future_buffer_.row(i_f).transpose()-future_buffer_.row(i_0).transpose())*
      _u;

  // Velocity
  _vel = L2_1d_coeff_.row(1).transpose();

  // Acceleration
  _acc = L2_2d_coeff_.transpose();

  return true;
}

int Trajectory::UpdateIndex() {
  index_++;
  return index_;
}

int Trajectory::UpdateIndex(int _index) {
  index_ = _index;
}

double Trajectory::UpdateTime() {
  time_ += config_->conv_u2t / config_->ctrl_freq;
  return time_;
}

void Trajectory::UpdateTrajectoryFutureBuffer(size_t _index_0) {
  // Error checking
  if (plan_.empty()) {
    ROS_WARN_NAMED(kLoggingName,"Current plan is empty");
    return;
  }

  // Initialize future trajectory
  size_t last_i = 0;
  size_t plan_size = plan_.size();
  if (_index_0 > plan_size)
    _index_0 = plan_size;

  for (size_t i=0; i<(size_t) config_->buffer_future_size; i++) {
    if (_index_0+i >= plan_size)
      break;

    future_buffer_(i,0) = plan_[_index_0+i].pose.position.x;
    future_buffer_(i,1) = plan_[_index_0+i].pose.position.y;
    future_buffer_(i,2) = tf2::getYaw(plan_[_index_0+i].pose.orientation);

    last_i++;
  }
  // - retain final pose of the trajectory
  for (size_t i=last_i; i<(size_t) config_->buffer_future_size; i++) {
    future_buffer_(i,0) = plan_[_index_0+last_i-1].pose.position.x;
    future_buffer_(i,1) = plan_[_index_0+last_i-1].pose.position.y;
    future_buffer_(i,2) =
        tf2::getYaw(plan_[_index_0+last_i-1].pose.orientation);
  }
  // - unwrap orientation
  for (size_t i=1; i<(size_t) config_->buffer_future_size; i++)
    future_buffer_(i,2) =
        future_buffer_(i-1,2) +
        NormalizeAngle(future_buffer_(i,2)-future_buffer_(i-1,2));

  // Initialize initial position and velocity
  for (size_t i=0; i<(size_t) config_->buffer_future_size; i++) {
    P_0_.row(i) = future_buffer_.row(0);
    V_0_.row(i) = future_buffer_.row(1) - future_buffer_.row(0);
  }

  // Least-squares: acceleration
  L2_2d_coeff_ =
      L2_2d_mat_.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).
          solve(future_buffer_ - P_0_ - U_mat_*V_0_);

  // Least-squares: velocity
  L2_1d_coeff_ =
      L2_1d_mat_.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).
                 solve(future_buffer_);
}

void Trajectory::InitializeMatrices() {
  for (size_t i=0; i<(size_t) config_->buffer_future_size; i++) {
    // - mat acceleration approximation
    L2_2d_mat_(i,0) = std::pow(i,2)/2.0;

    // - mat velocity approximation
    L2_1d_mat_(i,0) = 1;
    L2_1d_mat_(i,1) = i;

    // - vector U
    U_vec_(i,0) = i;
  }
  // - U diagonal matrix
  for (size_t i=0; i<(size_t) config_->buffer_future_size; i++) {
    for (size_t j=0; j<(size_t) config_->buffer_future_size; j++) {
      U_mat_(i,j) = (i != j)? 0 : U_vec_(i,0);
    }
  }
}

void Trajectory::ResizeTrajectoryFutureMatrices() {
  future_buffer_.resize(config_->buffer_future_size,
                        Eigen::NoChange_t::NoChange);
  P_0_.resize(config_->buffer_future_size, Eigen::NoChange_t::NoChange);
  V_0_.resize(config_->buffer_future_size, Eigen::NoChange_t::NoChange);
  L2_2d_mat_.resize(config_->buffer_future_size,1);
  L2_2d_coeff_.resize(1,3);
  L2_1d_mat_.resize(config_->buffer_future_size,2);
  L2_1d_coeff_.resize(2,3);
  U_vec_.resize(config_->buffer_future_size,
                Eigen::NoChange_t::NoChange);
  U_mat_.resize(config_->buffer_future_size,
                config_->buffer_future_size);
}

OmnidirectionalLocalPlanner::OmnidirectionalLocalPlanner(
    OmnidirectionalLocalPlannerConfig _config,
    costmap_2d::Costmap2DROS* _costmap_ros,
    geometry_msgs::PoseStamped* _current_pose,
    base_local_planner::OdometryHelperRos* _odom_helper,
    base_local_planner::LocalPlannerUtil* _planner_util)
    : config_(_config) , global_plan_(&config_) ,
      odom_helper_(_odom_helper) , planner_util_(_planner_util) ,
      costmap_ros_(_costmap_ros) , current_pose_(_current_pose) { }

bool OmnidirectionalLocalPlanner::SetPlan(
    const std::vector<geometry_msgs::PoseStamped>& _plan) {
  global_plan_.SetNewTrajectory(_plan);
  return planner_util_->setPlan(_plan);
}

bool OmnidirectionalLocalPlanner::SetRobotVelocity(
    geometry_msgs::Twist &_cmd_vel) {
  // Update robot position and velocity
  UpdateRobot();

  // Update time
  double u_ref = UpdateTimeReference();

  // Get global references for the robot feed-forward controllers
  if (!global_plan_.GetPoseVelAcc(u_ref,
                                  robot_.X_r,
                                  robot_.X_r_1d,robot_.X_r_2d))
    return false;

  // Transform references in the global frame onto the local one
  robot_.RotateGlobal2LocalFrame(robot_.X_r,robot_.X_loc_r);
  robot_.RotateGlobal2LocalFrame(robot_.X_r_1d,robot_.X_loc_r_1d);
  robot_.RotateGlobal2LocalFrame(robot_.X_r_2d,robot_.X_loc_r_2d);

  // PD Controllers
  PDControllers();

  // FF Controllers
  FFControllers();

  // Set robot velocity (considering maximum limits)
  Eigen::Vector3d V_r_tmp;
  if (config_.ff_ctrl_on)
    V_r_tmp = robot_.V_pd_r + robot_.V_ff_r;
  else
    V_r_tmp = robot_.V_pd_r;

  robot_.SetRobotVelocity(V_r_tmp, &config_);

  // Publish reference velocity
  _cmd_vel.linear.x = robot_.V_r(0);
  _cmd_vel.linear.y = robot_.V_r(1);
  _cmd_vel.angular.z = robot_.V_r(2);

  return true;
}

void OmnidirectionalLocalPlanner::FFControllers() {
  for (int i=0; i<3; i++) {
    robot_.V_ff_r(i) =
        robot_.X_r_2d(i) * config_.ff_ctrl_tau[i] / config_.ff_ctrl_kp[i] +
        robot_.X_r_1d(i) / config_.ff_ctrl_kp[i];
    robot_.V_ff_r(i) = robot_.V_ff_r(i) * config_.conv_u2t;
  }
}

void OmnidirectionalLocalPlanner::PDControllers() {
  robot_.X_loc_e_prev = robot_.X_loc_e;

  for (int i=0; i<3; i++) {
    if (i != 3) {
      robot_.X_loc_e(i) = robot_.X_loc_r(i) - robot_.X_loc(i);
      robot_.X_loc_e_1d(i) =
          (robot_.X_loc_e(i) - robot_.X_loc_e_prev(i)) * config_.ctrl_freq;
    } else {
      robot_.X_loc_e(i) =
          NormalizeAngle(robot_.X_loc_r(i) - robot_.X_loc(i));
      robot_.X_loc_e_1d(i) =
          NormalizeAngle(robot_.X_loc_e(i) - robot_.X_loc_e_prev(i)) *
          config_.ctrl_freq;
    }

    robot_.V_pd_r(i) =
        config_.pd_ctrl_kc[i] *
        ( robot_.X_loc_e(i) + config_.pd_ctrl_td[i] * robot_.X_loc_e_1d(i));
  }
}

bool OmnidirectionalLocalPlanner::GetCurrentGoal(double& _x, double& _y,
                                                 double& _th) {
  return global_plan_.GetCurrentGoal(_x,_y,_th);
}

void OmnidirectionalLocalPlanner::GetTrajectoryPath(nav_msgs::Path& _path) {
  std::vector<geometry_msgs::PoseStamped> path;
  global_plan_.GetTrajectoryPath(path);

  _path.poses.resize(path.size());
  _path.header.frame_id = path[0].header.frame_id;
  _path.header.stamp = path[0].header.stamp;

  for (size_t i=0; i<path.size(); i++)
    _path.poses[i] = path[i];
}

void OmnidirectionalLocalPlanner::GetFutureTrajectoryPath(
    nav_msgs::Path& _path) {
  std::vector<geometry_msgs::PoseStamped> path;
  global_plan_.GetFutureTrajectoryPath(path,costmap_ros_);

  _path.poses.resize(path.size());
  _path.header.frame_id = path[0].header.frame_id;
  _path.header.stamp = path[0].header.stamp;

  for (size_t i=0; i<path.size(); i++)
    _path.poses[i] = path[i];
}

void OmnidirectionalLocalPlanner::GetRobotPoses(
    geometry_msgs::PoseStamped& _pos, geometry_msgs::PoseStamped& _pos_r,
    geometry_msgs::PoseStamped& _pos_r_1d,
    geometry_msgs::PoseStamped& _pos_r_2d, geometry_msgs::PoseStamped& _pos_loc,
    geometry_msgs::PoseStamped& _pos_loc_r,
    geometry_msgs::PoseStamped& _pos_loc_r_1d,
    geometry_msgs::PoseStamped& _pos_loc_r_2d,
    geometry_msgs::PoseStamped& _pos_loc_e,
    geometry_msgs::PoseStamped& _pos_loc_e_1d) {
  ros::Time t = ros::Time::now();
  std::string frame_id = costmap_ros_->getGlobalFrameID();

  ConvertEigenVector3d2PoseStampedMsg(robot_.X         ,_pos         ,
                                      frame_id,t);
  ConvertEigenVector3d2PoseStampedMsg(robot_.X_r       ,_pos_r       ,
                                      frame_id,t);
  ConvertEigenVector3d2PoseStampedMsg(robot_.X_r_1d    ,_pos_r_1d    ,
                                      frame_id,t);
  ConvertEigenVector3d2PoseStampedMsg(robot_.X_r_2d    ,_pos_r_2d    ,
                                      frame_id,t);
  ConvertEigenVector3d2PoseStampedMsg(robot_.X_loc     ,_pos_loc     ,
                                      frame_id,t);
  ConvertEigenVector3d2PoseStampedMsg(robot_.X_loc_r   ,_pos_loc_r   ,
                                      frame_id,t);
  ConvertEigenVector3d2PoseStampedMsg(robot_.X_loc_r_1d,_pos_loc_r_1d,
                                      frame_id,t);
  ConvertEigenVector3d2PoseStampedMsg(robot_.X_loc_r_2d,_pos_loc_r_2d,
                                      frame_id,t);
  ConvertEigenVector3d2PoseStampedMsg(robot_.X_loc_e   ,_pos_loc_e   ,
                                      frame_id,t);
  ConvertEigenVector3d2PoseStampedMsg(robot_.X_loc_e_1d,_pos_loc_e_1d,
                                      frame_id,t);
}

void OmnidirectionalLocalPlanner::ConvertEigenVector3d2PoseStampedMsg(
    Eigen::Vector3d &_vec, geometry_msgs::PoseStamped &_pose,
    std::string& _frame_id, ros::Time& _t) {

  _pose.header.frame_id = _frame_id;
  _pose.header.stamp    = _t;
  _pose.pose.position.x = _vec(0);
  _pose.pose.position.y = _vec(1);
  _pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0,0,_vec(2));
  tf2::convert(q, _pose.pose.orientation);
}

void OmnidirectionalLocalPlanner::GetRobotVelocities(
    geometry_msgs::Twist& _vel, geometry_msgs::Twist& _vel_r,
    geometry_msgs::Twist& _vel_pd_r, geometry_msgs::Twist& _vel_ff_r) {
  // Robot current velocity
  ConvertEigenVector3d2TwistMsg(robot_.V, _vel);

  // Robot reference velocity
  ConvertEigenVector3d2TwistMsg(robot_.V_r, _vel_r);

  // PD ctrl
  ConvertEigenVector3d2TwistMsg(robot_.V_pd_r, _vel_pd_r);

  // FF ctrl
  ConvertEigenVector3d2TwistMsg(robot_.V_ff_r, _vel_ff_r);
}

void OmnidirectionalLocalPlanner::ConvertEigenVector3d2TwistMsg(
    Eigen::Vector3d& _vec, geometry_msgs::Twist& _msg) {
  _msg.linear.x  = _vec(0);
  _msg.linear.y  = _vec(1);
  _msg.angular.z = _vec(2);
}

void OmnidirectionalLocalPlanner::UpdateRobot() {
  UpdateRobotPosition();
  UpdateRobotVelocity();
}

void OmnidirectionalLocalPlanner::UpdateRobotPosition() {
  // Get current pose
  costmap_ros_->getRobotPose(*current_pose_);

  // Update struct
  // - global position
  robot_.X(0) = current_pose_->pose.position.x;
  robot_.X(1) = current_pose_->pose.position.y;
  robot_.X(2) = tf2::getYaw(current_pose_->pose.orientation);
  // - local position
  robot_.RotateGlobal2LocalFrame(robot_.X, robot_.X_loc);
}

void OmnidirectionalLocalPlanner::UpdateRobotVelocity() {
  // Get velocity from odom topic (linear + twist)
  geometry_msgs::PoseStamped robot_vel;
  odom_helper_->getRobotVel(robot_vel);

  // Compute linear and angular velocity of the robot
  robot_.V(0) = robot_vel.pose.position.x;
  robot_.V(1) = robot_vel.pose.position.y;
  robot_.V(2) = tf2::getYaw(robot_vel.pose.orientation);
}

double OmnidirectionalLocalPlanner::UpdateTimeReference() {
  int index = global_plan_.GetIndex();
  double time = global_plan_.UpdateTime();
  double u_ref = time - index;
  while (u_ref >= 1) {
    index++;
    u_ref = time - index;
  }
  if (index != global_plan_.GetIndex()) {
    global_plan_.UpdateIndex(index);
    global_plan_.UpdateTrajectoryFutureBuffer(index);
  }

  return u_ref;
}

}  // namespace omnidirectional_local_planner
