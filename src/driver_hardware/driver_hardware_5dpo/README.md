# driver_hardware_5dpo

**Version 0.3**

This folder contains a ROS package relative to the drive for the 5DPO MSL
robot. The drive is required for communicating with the robot and have 
available all its different functions.

The serial communication is handle by Qt. This approach facilitates the use of
serial ports with the necessary robustness to prevent failures.

**With this version, it is possible to do:**

- Subscribe motors angular speed reference 
- Publish encoders data (encoders + wheels angular speed)
- Read encoders, emergency, reset button, ball sensor, battery level, IMU, 
  capacitator level, reset, and rollers
- Send motors speed
- Communicate with Arduino using Qt 

**The next version will add these features:**

No further developments are foreseen for this ROS package.

## ROS

This node is developed based on the principles presented in
(https://github.com/leggedrobotics/ros_best_practices). The ROS libraries
are only inserted in the files `main.cpp` e `DriverHardware5dpoROS.cpp/h`.
This approach tries to implement an easy way to upgrade between different
versions of the ROS platform.

**Current version:**

- Ubuntu 18.04 LTS
- Ros Melodic

### Dependencies

- driver_qextserialport
- itrci_hardware ([inesctec_robotics_custom_interfaces_stack/itrci_hardware](https://gitlab.inesctec.pt/CRIIS/inesctec_robotics_custom_interfaces_stack/-/tree/master/itrci_hardware))
- Qt4
- roscpp
- rospy
- serial_communication_channels (already in the repository
  [5dpo_msl_ros/driver_hardware](https://git.fe.up.pt/5dpo/5dpo_msl_ros/driver_hardware))
- std_msgs

### Parameters

- enc_pulses_per_revolution (`float`): resolution of the encoder (pulses/rot)
- gear_reduction (`float`): reduction ratio of the transmissions 
  (\[gear_reduction:1\])
- serial_port_name (`std::string`): name of the serial port (e.g., 
  `/dev/ttyACM0`)

### Subscribes

- motors_speed_ref ([`motors_array_input.msg`](https://gitlab.inesctec.pt/CRIIS/inesctec_robotics_custom_interfaces_stack/-/blob/master/itrci_hardware/msg/motors_array_input.msg))

### Publishes

- motors_outputs ([`motors_array_output.msg`](https://gitlab.inesctec.pt/CRIIS/inesctec_robotics_custom_interfaces_stack/-/blob/master/itrci_hardware/msg/motors_array_output.msg))

### Services

None.

### Actions

None.

## Contacts

If you have any questions or you want to know more about this work, please
contact one of the contributors of this package:

- Héber Miguel Sobreira (heber.m.sobreira@inesctec.pt)
- Ricardo B. Sousa (up201503004@fe.up.pt, ricardo.b.sousa@inesctec.pt,
  sousa.ricardob@outlook.com)