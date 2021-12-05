#include <ros/ros.h>

#include <QCoreApplication>
#include <thread>

#include "driver_hardware_5dpo/DriverHardware5dpoROS.h"

int main(int argc, char *argv[]){
  QCoreApplication* qt_app = new QCoreApplication(argc,argv);

  ros::init(argc,argv,"driver_hardware_5dpo");
  ros::NodeHandle node_handle;

  driver_hardware_5dpo::DriverHardware5dpoROS driver_hw_5dpo(node_handle);

  std::thread ros_ctrl = std::thread(
      &driver_hardware_5dpo::DriverHardware5dpoROS::Execute, &driver_hw_5dpo);

  qt_app->exec();

  ros_ctrl.join();

  delete qt_app;

  return 0;
}