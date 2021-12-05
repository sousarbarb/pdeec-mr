#ifndef SRC_OMNIDIRECTIONAL_LOCAL_PLANNER_H
#define SRC_OMNIDIRECTIONAL_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <cmath>

namespace omnidirectional_local_planner {

// Default values:
// - miscellaneous
const char kLoggingName[] = "OMNI_LOCAL_PLANNER";
const int kBufferFutureSize = 5;
const double kNomVelocityLin = 0.15; // m/s
// - acceleration limits
const double kMaxAcceleration[3] = { 0.25 , 0.25 , 0.785398163 };
// - velocity limits
const double kMaxVelocity[3] = { 0.25 , 0.25 , 0.785398163 };
// - goal tolerances
const double kXYGoalTolerance = 0.10; // m
const double kThGoalTolerance = 0.05; //rad
// - controller gains
const double kPDCtrlKc[3] = { 4.417214 , 4.382877 , 3.404729 };
const double kPDCtrlTd[3] = { 0.069690 , 0.067916 , 0.002368 };
const double kFFCtrlKp[3] = { 1 , 1 , 1 };
const double kFFCtrlTau[3] = { 0.129073 , 0.12807 , 0.099488 };

double NormalizeAngle(double _angle);

struct OmnidirectionalLocalPlannerConfig {
 public:
  double ctrl_freq = 0;
  int buffer_future_size = kBufferFutureSize;
  double conv_u2t = 0;
  bool ff_ctrl_on = true;
  double max_acc_lim[3] = { kMaxAcceleration[0] ,
                            kMaxAcceleration[1] ,
                            kMaxAcceleration[2] };
  double max_vel_lim[3] = { kMaxVelocity[0] ,
                            kMaxVelocity[1] ,
                            kMaxVelocity[2] };
  double goal_tol_xy = kXYGoalTolerance;
  double goal_tol_th = kThGoalTolerance;
  double pd_ctrl_kc[3] = { kPDCtrlKc[0] , kPDCtrlKc[1] , kPDCtrlKc[2] };
  double pd_ctrl_td[3] = { kPDCtrlTd[0] , kPDCtrlTd[1] , kPDCtrlTd[2] };
  double ff_ctrl_kp[3] = { kFFCtrlKp[0] , kFFCtrlKp[1] , kFFCtrlKp[2] };
  double ff_ctrl_tau[3] = { kFFCtrlTau[0] , kFFCtrlTau[1] , kFFCtrlTau[2] };

 public:
  void SetConvU2T(double _nom_vel, double _map_res) {
    conv_u2t = _nom_vel / (_map_res * std::sqrt(2));
  };
};

struct RobotState {
 public:
  Eigen::Vector3d X, X_r, X_r_1d, X_r_2d;
  Eigen::Vector3d X_loc, X_loc_r, X_loc_r_1d, X_loc_r_2d;
  Eigen::Vector3d X_loc_e, X_loc_e_prev, X_loc_e_1d;
  Eigen::Vector3d V, V_r;
  Eigen::Vector3d V_pd_r, V_ff_r;

 public:
  RobotState() = default;
  ~RobotState() = default;
  void RotateGlobal2LocalFrame(Eigen::Vector3d& _X_global,
                               Eigen::Vector3d& _X_local);
  void SetRobotVelocity(Eigen::Vector3d& _V,
                        OmnidirectionalLocalPlannerConfig* _config);
};

class Trajectory {
 private:
  std::vector<geometry_msgs::PoseStamped> plan_;

  size_t index_;
  double time_;

  OmnidirectionalLocalPlannerConfig* config_;

  Eigen::Matrix<double,Eigen::Dynamic,3> future_buffer_;
  Eigen::Matrix<double,Eigen::Dynamic,3> P_0_;
  Eigen::Matrix<double,Eigen::Dynamic,3> V_0_;
  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> L2_2d_mat_;
  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> L2_2d_coeff_;
  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> L2_1d_mat_;
  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> L2_1d_coeff_;
  Eigen::Matrix<double,Eigen::Dynamic,1>              U_vec_;
  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> U_mat_;

public:
  explicit Trajectory(OmnidirectionalLocalPlannerConfig* _config);
  void SetNewTrajectory(const std::vector<geometry_msgs::PoseStamped>& _plan);
  bool GetCurrentGoal(double& _x, double& _y, double& _th);
  int GetIndex();
  void GetTrajectoryPath(std::vector<geometry_msgs::PoseStamped>& _path);
  void GetFutureTrajectoryPath(std::vector<geometry_msgs::PoseStamped>& _path,
                               costmap_2d::Costmap2DROS* _costmap_ros);
  bool GetPoseVelAcc(double _u, Eigen::Vector3d& _pose,
                     Eigen::Vector3d& _vel, Eigen::Vector3d& _acc);
  int UpdateIndex();
  int UpdateIndex(int _index);
  double UpdateTime();
  void UpdateTrajectoryFutureBuffer(size_t _index_0);

 private:
  void InitializeMatrices();
  void ResizeTrajectoryFutureMatrices();
};

class OmnidirectionalLocalPlanner {
 public:
  OmnidirectionalLocalPlannerConfig config_;

 private:
  Trajectory global_plan_;

  base_local_planner::OdometryHelperRos* odom_helper_;
  base_local_planner::LocalPlannerUtil* planner_util_;
  RobotState robot_;

  costmap_2d::Costmap2DROS* costmap_ros_;
  geometry_msgs::PoseStamped* current_pose_;

 public:
  explicit OmnidirectionalLocalPlanner(
      OmnidirectionalLocalPlannerConfig _config,
      costmap_2d::Costmap2DROS* _costmap_ros,
      geometry_msgs::PoseStamped* _current_pose,
      base_local_planner::OdometryHelperRos* _odom_helper,
      base_local_planner::LocalPlannerUtil* _planner_util);
  bool GetCurrentGoal(double& _x, double& _y, double& _th);
  void GetTrajectoryPath(nav_msgs::Path& _path);
  void GetFutureTrajectoryPath(nav_msgs::Path& _path);
  void GetRobotPoses(geometry_msgs::PoseStamped& _pos,
                     geometry_msgs::PoseStamped& _pos_r,
                     geometry_msgs::PoseStamped& _pos_r_1d,
                     geometry_msgs::PoseStamped& _pos_r_2d,
                     geometry_msgs::PoseStamped& _pos_loc,
                     geometry_msgs::PoseStamped& _pos_loc_r,
                     geometry_msgs::PoseStamped& _pos_loc_r_1d,
                     geometry_msgs::PoseStamped& _pos_loc_r_2d,
                     geometry_msgs::PoseStamped& _pos_loc_e,
                     geometry_msgs::PoseStamped& _pos_loc_e_1d);
  void GetRobotVelocities(geometry_msgs::Twist& _vel,
                          geometry_msgs::Twist& _vel_r,
                          geometry_msgs::Twist& _vel_pd_r,
                          geometry_msgs::Twist& _vel_ff_r);
  bool SetPlan(const std::vector<geometry_msgs::PoseStamped>& _plan);
  bool SetRobotVelocity(geometry_msgs::Twist& _cmd_vel);
  void UpdateRobot();

 private:
  static void ConvertEigenVector3d2PoseStampedMsg(
      Eigen::Vector3d& _vec, geometry_msgs::PoseStamped& _pose,
      std::string& _frame_id, ros::Time& _t);
  static void ConvertEigenVector3d2TwistMsg(Eigen::Vector3d& _vec,
                                            geometry_msgs::Twist& _msg);
  void FFControllers();
  void PDControllers();
  void UpdateRobotPosition();
  void UpdateRobotVelocity();
  double UpdateTimeReference();
};

}  // namespace omnidirectional_local_planner

#endif //SRC_OMNIDIRECTIONAL_LOCAL_PLANNER_H
