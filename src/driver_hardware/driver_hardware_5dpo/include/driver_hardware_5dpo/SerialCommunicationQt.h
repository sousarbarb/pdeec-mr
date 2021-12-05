#ifndef SRC_SERIALCOMMUNICATIONQT_H
#define SRC_SERIALCOMMUNICATIONQT_H

#include <ros/ros.h>
#include <driver_qextserialport/qextserialport.h>

#include <QCoreApplication>
#include <QObject>
#include <QTimer>
#include <iostream>
#include <thread>
#include <mutex>

#include "driver_hardware_5dpo/Robot5dpo.h"
#include "driver_hardware_5dpo/SerialCommunicationChannelsConfig.h"

namespace driver_hardware_5dpo {

const auto kSerialPortBaudRate = BAUD115200;
const auto kSerialPortDataBits = DATA_8;
const auto kSerialPortStopBits = STOP_1;
const auto kSerialPortParity = PAR_NONE;
const auto kSerialPortFlowCtrl = FLOW_OFF;

class SerialCommunicationQt : public QObject {
Q_OBJECT
 private:
  Robot5dpo* robot_5dpo_;
  std::mutex* robot_5dpo_mtx_;

  QextSerialPort* serial_port_;
  std::string serial_port_name_;

  DataCommunicationChannels* data_channels_;

 public:
  explicit SerialCommunicationQt(Robot5dpo* _robot_5dpo = nullptr,
                                 std::mutex* _robot_5dpo_mtx = nullptr);
  ~SerialCommunicationQt() override;
  bool ConnectSerialPort(
      std::string _serial_port_name = std::string("/dev/ttyACM0"));
 private:
  void SendData();
 private Q_SLOTS:
  void SerialDataCallback();
};

}  // namespace driver_hardware_5dpo


#endif //SRC_SERIALCOMMUNICATIONQT_H
