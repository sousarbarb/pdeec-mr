#include "driver_hardware_5dpo/SerialCommunicationQt.h"

namespace driver_hardware_5dpo {

SerialCommunicationQt::SerialCommunicationQt(Robot5dpo* _robot_5dpo,
                                             std::mutex* _robot_5dpo_mtx)
    : QObject(nullptr) , robot_5dpo_(_robot_5dpo) ,
      robot_5dpo_mtx_(_robot_5dpo_mtx) {
  // Init channels
  data_channels_ = InitCommunications();

  // Serial port
  serial_port_ = new QextSerialPort;

  // Serial callback
  connect(serial_port_,SIGNAL(readyRead()),
          this,SLOT(SerialDataCallback()));
}

SerialCommunicationQt::~SerialCommunicationQt() {
  // Close serial port
  if (serial_port_->isOpen())
    serial_port_->close();
  delete serial_port_;
}

bool SerialCommunicationQt::ConnectSerialPort(std::string _serial_port_name) {
  // Initialize
  serial_port_->close();
  serial_port_name_ = std::move(_serial_port_name);

  // Serial port configurations
  serial_port_->setPortName(QString::fromStdString(serial_port_name_));
  serial_port_->setBaudRate(kSerialPortBaudRate);
  serial_port_->setDataBits(kSerialPortDataBits);
  serial_port_->setStopBits(kSerialPortStopBits);
  serial_port_->setParity(kSerialPortParity);
  serial_port_->setFlowControl(kSerialPortFlowCtrl);

  // Open serial port
  serial_port_->open(QIODevice::ReadWrite);
  if (serial_port_->isOpen()) {
    serial_port_->flush();
    ROS_INFO("[DRIVER_HARDWARE_5DPO] Serial port (%s) open",
             serial_port_name_.c_str());
    return true;
  } else {
    ros::requestShutdown();
    ROS_FATAL("[DRIVER_HARDWARE_5DPO] Could not open serial port (%s)",
              serial_port_name_.c_str());
    return false;
  }
}

void SerialCommunicationQt::SendData() {
  std::ostringstream str_debug;

  // Update channels
  robot_5dpo_mtx_->lock();
  data_channels_->channel_G = robot_5dpo_->motors[0].w_r;
  data_channels_->channel_H = robot_5dpo_->motors[1].w_r;
  data_channels_->channel_I = robot_5dpo_->motors[2].w_r;
  robot_5dpo_mtx_->unlock();

  // Get channels strings
  std::string s_G = SendChannel('G');
  std::string s_H = SendChannel('H');
  std::string s_I = SendChannel('I');

  // Send serial data
  serial_port_->write(s_G.c_str());
  serial_port_->write(s_H.c_str());
  serial_port_->write(s_I.c_str());

  // Debug
  str_debug << s_G << s_H << s_I;
  ROS_DEBUG("[DRIVER_HARDWARE_5DPO][TX] %s",str_debug.str().c_str());
}

void SerialCommunicationQt::SerialDataCallback() {
  // Check if the serial port is ok
  if (!serial_port_->isOpen()) {
    ros::requestShutdown();
    ROS_FATAL("[DRIVER_HARDWARE_5DPO] Serial port (%s) not available",
              serial_port_name_.c_str());
    return;
  }

  // Check if ROS is ok
  if (!ros::ok()) {
    ros::requestShutdown();
    return;
  }

  // Read data
  QByteArray data_received = serial_port_->readAll();
  int num_bytes = data_received.size();
  char data_byte, channel;
  std::ostringstream str_debug;

  for (int i=0; i<num_bytes; i++) {
    data_byte = data_received[i];
    channel = ProcessChannelsSerialData(data_byte);

    // Debug
    str_debug << data_byte;

    // Channels
    if (channel > 0) {
      switch (channel) {
        case 'g':  // Encoder Wheel 0
          robot_5dpo_mtx_->lock();
          robot_5dpo_->motors[0].UpdateEncThicks(data_channels_->channel_g);
          robot_5dpo_mtx_->unlock();
          break;

        case 'h':  // Encoder Wheel 1
          robot_5dpo_mtx_->lock();
          robot_5dpo_->motors[1].UpdateEncThicks(data_channels_->channel_h);
          robot_5dpo_mtx_->unlock();
          break;

        case 'i':  // Encoder Wheel 2 + SendData (synchronization byte)
          robot_5dpo_mtx_->lock();
          robot_5dpo_->motors[2].UpdateEncThicks(data_channels_->channel_i);
          robot_5dpo_mtx_->unlock();
          SendData();
          break;

        case 'j':  // Sample timestamps computed by Arduino Mega
          robot_5dpo_mtx_->lock();
          robot_5dpo_->motors[0].UpdateSampleTime(data_channels_->channel_j);
          robot_5dpo_->motors[1].UpdateSampleTime(data_channels_->channel_j);
          robot_5dpo_->motors[2].UpdateSampleTime(data_channels_->channel_j);
          robot_5dpo_mtx_->unlock();
          break;

        case 'k':  // Emergency button
          robot_5dpo_mtx_->lock();
          if (1 == data_channels_->channel_k)
            robot_5dpo_->emergency = true;
          else
            robot_5dpo_->emergency = false;
          robot_5dpo_mtx_->unlock();
          break;

        case 'l':  // Ball sensor
          robot_5dpo_mtx_->lock();
          if (1 == data_channels_->channel_l)
            robot_5dpo_->ball_sensor = true;
          else
            robot_5dpo_->ball_sensor = false;
          robot_5dpo_mtx_->unlock();
          break;

        case 'm':  // Battery level
          robot_5dpo_mtx_->lock();
          robot_5dpo_->UpdateBatteryLevel(data_channels_->channel_m);
          robot_5dpo_mtx_->unlock();
          break;

        case 'n':  // Compass X + Y axis + compass angle
          robot_5dpo_mtx_->lock();
          robot_5dpo_->compass.x = data_channels_->channel_n >> 16;
          robot_5dpo_->compass.y = data_channels_->channel_n & (0x0000FFFF);
          robot_5dpo_->compass.angle = std::atan2(robot_5dpo_->compass.y,
                                                  robot_5dpo_->compass.x);
          robot_5dpo_mtx_->unlock();
          break;

        case 'o':  // Compass Z axis + Accelerometer Z axis
          robot_5dpo_mtx_->lock();
          robot_5dpo_->compass.z = data_channels_->channel_o >> 16;
          robot_5dpo_->accelerometer.z = data_channels_->channel_o &
                                         (0x0000FFFF);
          robot_5dpo_mtx_->unlock();
          break;

        case 'p':  // Accelerometer X + Y axis
          robot_5dpo_mtx_->lock();
          robot_5dpo_->accelerometer.x = data_channels_->channel_p >> 16;
          robot_5dpo_->accelerometer.y = data_channels_->channel_p &
                                         (0x0000FFFF);
          robot_5dpo_mtx_->unlock();
          break;

        case 'q':  // Kicker capacitator level
          robot_5dpo_mtx_->lock();
          robot_5dpo_->kicker.UpdateCapacitatorLevel(data_channels_->channel_q);
          robot_5dpo_mtx_->unlock();
          break;

        case 'r':  // Reset
          robot_5dpo_mtx_->lock();
          if (1 == data_channels_->channel_r)
            robot_5dpo_->reset = true;
          else
            robot_5dpo_->reset = false;
          robot_5dpo_mtx_->unlock();
          break;

        case 's':  // Rollers current
          robot_5dpo_mtx_->lock();
          robot_5dpo_->roller_motors[ROLLER_LEFT].UpdateRollerMotorCurrent(
              (int16_t) (data_channels_->channel_s >> 16));
          robot_5dpo_->roller_motors[ROLLER_RIGHT].UpdateRollerMotorCurrent(
              (int16_t) (data_channels_->channel_s & (0x0000FFFF)));
          robot_5dpo_mtx_->unlock();
          break;

        case 't':  // Rollers angular velocity
          robot_5dpo_mtx_->lock();
          robot_5dpo_->roller_motors[ROLLER_LEFT].UpdateRollerMotorW(
              (int16_t) (data_channels_->channel_t >> 16));
          robot_5dpo_->roller_motors[ROLLER_RIGHT].UpdateRollerMotorW(
              (int16_t) (data_channels_->channel_t & (0x0000FFFF)));
          robot_5dpo_mtx_->unlock();
          break;

        case 'u':
          robot_5dpo_mtx_->lock();
          robot_5dpo_mtx_->unlock();
          break;

        case 'v':
          robot_5dpo_mtx_->lock();
          robot_5dpo_mtx_->unlock();
          break;

        case 'w':
          robot_5dpo_mtx_->lock();
          robot_5dpo_mtx_->unlock();
          break;

        case 'x':
          robot_5dpo_mtx_->lock();
          robot_5dpo_mtx_->unlock();
          break;

        case 'y':
          robot_5dpo_mtx_->lock();
          robot_5dpo_mtx_->unlock();
          break;

        case 'z':
          robot_5dpo_mtx_->lock();
          robot_5dpo_mtx_->unlock();
          break;

        default:
          break;
      }
    }
  }

  // Debug
  ROS_DEBUG("[DRIVER_HARDWARE_5DPO][RX] %s",str_debug.str().c_str());
}

}  // namespace driver_hardware_5dpo
