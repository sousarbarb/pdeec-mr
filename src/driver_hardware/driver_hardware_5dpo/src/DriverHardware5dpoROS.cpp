#include "driver_hardware_5dpo/DriverHardware5dpoROS.h"

namespace driver_hardware_5dpo {

DriverHardware5dpoROS::DriverHardware5dpoROS(ros::NodeHandle& node_handle)
    : node_handle_(node_handle) , loop_rate_ctrl_(CTRL_FREQUENCY) {
  // Read parameters from ROS param server
  ReadParameters();

  // Serial port
  serial_communication_qt_ = new SerialCommunicationQt(
      &robot_5dpo_, &robot_5dpo_mtx_);
  serial_communication_qt_->ConnectSerialPort(serial_port_name_);

  // Publisher
  pub_motors_outputs_ =
      node_handle_.advertise<itrci_hardware::motors_array_output>(
          "motors_outputs",1);

  // Subscriber
  sub_motors_speed_ =
      node_handle_.subscribe(
          "motors_ref", 1,
          &DriverHardware5dpoROS::SubscribeMotorsSpeed, this);

  // Debug
  ROS_INFO("[DRIVER_HARDWARE_5DPO] Successfully launched node.");
}

DriverHardware5dpoROS::~DriverHardware5dpoROS() {
  delete serial_communication_qt_;
}

void DriverHardware5dpoROS::Execute() {
  while (ros::ok()) {
    // Send motors output
    if (sample_time_ - sample_time_prev_ >
        ros::Duration(WATCHDOG_TIMER_MOTORS_SPEED_REF))
      robot_5dpo_.StopMotors();

    // Publish data
    PublishMotorsEncoders();

    // Spin for callbacks
    ros::spinOnce();
    loop_rate_ctrl_.sleep();
  }
  QCoreApplication::exit();
}

void DriverHardware5dpoROS::PublishMotorsEncoders() {
  // Construct the message
  itrci_hardware::motors_array_output data_to_send;

  data_to_send.stamp = ros::Time::now();
  data_to_send.motors_output_array_data.resize(3);

  robot_5dpo_mtx_.lock();
  for (int i=0;i<3;i++) {
    data_to_send.motors_output_array_data[i].encoder_counter =
        robot_5dpo_.motors[i].enc_thicks;
    data_to_send.motors_output_array_data[i].enc_pulses_per_revolution =
        robot_5dpo_.motors[i].encoder_res *
        robot_5dpo_.motors[i].gear_reduction;
    data_to_send.motors_output_array_data[i].speed =
        robot_5dpo_.motors[i].w;
  }
  robot_5dpo_mtx_.unlock();

  // Publish data
  pub_motors_outputs_.publish(data_to_send);
}

void DriverHardware5dpoROS::SubscribeMotorsSpeed(
    const itrci_hardware::motors_array_input& message) {
  if (message.input_ref.size() >= 3) {
    // Update angular velocity references
    robot_5dpo_mtx_.lock();
    for (int i=0;i<3;i++)
      robot_5dpo_.motors[i].w_r = message.input_ref[i];
    robot_5dpo_mtx_.unlock();

    // Update the watchdog timer
    sample_time_prev_ = sample_time_;
    sample_time_      = ros::Time::now();

    // Debug
    ROS_DEBUG("[DRIVER_HARDWARE_5DPO][SUB] w_r: %lf\t%lf\t%lf",
              robot_5dpo_.motors[0].w_r,
              robot_5dpo_.motors[1].w_r,
              robot_5dpo_.motors[2].w_r);
  }
}

bool DriverHardware5dpoROS::ReadParameters() {
  // Get parameters from ROS parameter server
  // - motors:
  for (auto & motor : robot_5dpo_.motors) {
    node_handle_.param<float>(
        "enc_pulses_per_revolution",
        motor.encoder_res, 1024.0);
    node_handle_.param<float>(
        "gear_reduction",
        motor.gear_reduction, 12.0);
  }

  // - name of the serial port
  node_handle_.param<std::string>(
      "serial_port_name",
      serial_port_name_, "/dev/ttyACM0");

  return true;
}

}  // namespace driver_hardware_5dpo