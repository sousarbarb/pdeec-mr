#ifndef SRC_DRIVERHARDWARE5DPOROS_H
#define SRC_DRIVERHARDWARE5DPOROS_H

#include <ros/ros.h>
#include <itrci_hardware/motors_array_input.h>
#include <itrci_hardware/motors_array_output.h>
#include <std_srvs/Empty.h>

#include <QCoreApplication>

#include <mutex>
#include <thread>

#include "driver_hardware_5dpo/Robot5dpo.h"
#include "driver_hardware_5dpo/SerialCommunicationQt.h"

#define WATCHDOG_TIMER_MOTORS_SPEED_REF 0.2

namespace driver_hardware_5dpo {

class DriverHardware5dpoROS {
 private:
  ros::NodeHandle& node_handle_;
  ros::Rate loop_rate_ctrl_;

  ros::Publisher pub_motors_outputs_;
  ros::Subscriber sub_motors_speed_;
  ros::Time sample_time_;
  ros::Time sample_time_prev_;

  Robot5dpo robot_5dpo_;
  std::mutex robot_5dpo_mtx_;

  std::string serial_port_name_;
  SerialCommunicationQt* serial_communication_qt_;

 public:
  explicit DriverHardware5dpoROS(ros::NodeHandle& node_handle);
  virtual ~DriverHardware5dpoROS();
  void Execute();
 private:
  void PublishMotorsEncoders();
  void SubscribeMotorsSpeed(
      const itrci_hardware::motors_array_input& message);
  bool ReadParameters();
};

}  // namespace driver_hardware_5dpo

#endif //SRC_DRIVERHARDWARE5DPOROS_H
