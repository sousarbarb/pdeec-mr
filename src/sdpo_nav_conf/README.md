# sdpo_nav_conf

**Version 1.0**

This repository implements the launch files required for the 5DPO Navigation 
Stack. The system implemented is based on the INESC TEC Robotics Navigation 
Stack that it allows you to have different configurations implemented and 
selecting just one based on your environment variables.

**With this version, it is possible to do:**

- Launch the node omnijoy

**The next version will add these features:**

- Launch the node localization_perfect_match
- Launch the node omni_controller (to be developed)

**Version 0**

- Launch the configuration feup0
- Launch the node driver_hardware_5dpo and its parameters through an YAML file
- Launch the node odometry_processor and its parameters through an YAML file

## Configurations

The main purpose for the support

### Setup

First, you need to setup your environment variables. These variables allows 
you to name your robot as well as selecting the proper configuration that 
you want to use.

```shell
# Robot id
export ROBOT_ID=sdpo  # (default: unnamed_robot)
# Configuration
export CONF=feup0     # (default: feup0)
```

Then, you can launch the navigation stack for the 5DPO's robot using the 
following commands (remember that it is required an installation of this 
stack or the use of the code from the 5dpo_msl_ros subgroup):

```shell
# Compile your catkin workspace, if necessary
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
# Launch
roslaunch sdpo_nav_conf wake_up_almighty_5dpo.launch
```

### feup0

- Hardware
  - driver_hardware_5dpo
  - serial_communication_channels
  - odometry_processor

## Contacts

If you have any questions or you want to know more about this work, please
contact one of the contributors of this package:

- Ricardo B. Sousa (up201503004@fe.up.pt, ricardo.b.sousa@inesctec.pt,
  sousa.ricardob@outlook.com)
