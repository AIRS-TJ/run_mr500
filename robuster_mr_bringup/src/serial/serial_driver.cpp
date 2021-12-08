#include "serial_node.h"
#include "command.h"
#include "utily.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#define TIMEOUT 2

RobusterDriver::RobusterDriver() {

  ros::NodeHandle private_node("~");

  private_node.param<std::string>("port", port_name_, std::string("/dev/ttyS0"));
  private_node.param<int>("baud_rate", baud_rate_, 115200);
  private_node.param<int>("control_rate", control_rate_, 10);
  private_node.param<std::string>("odom_frame", odom_frame_, std::string("odom"));
  private_node.param<std::string>("base_frame", base_frame_, std::string("base_link"));

  private_node.param<bool>("use_odom_trans", use_odom_trans_, false);

  pthread_mutex_init(&mutex_,NULL);
  pthread_condattr_setclock(&attr_, CLOCK_MONOTONIC);
  pthread_cond_init(&cond_, &attr_);

  last_send_time_ = getSysTime(); // initialize 

  serial_ptr_ = nullptr;
  disable_permission_ = false;

  init();
}

RobusterDriver::~RobusterDriver() {
  /// TODO: close port
  closeDriver();
}

void RobusterDriver::closeDriver() {
  ROS_INFO("Close robuster driver.");
  int  retry_cnt = 0;

  // 清除灯光状态
  if (cur_dev_state_.front_light != 0 || cur_dev_state_.rear_light != 0) {
    msg_que_.push(Command::setLightStatus(0, 0));
  }

  // 退出时解除控制，取消报警
  disable_permission_ = true;
  while (cur_dev_state_.control_mode != 0x00 && retry_cnt < 10) { // 不可用 0x01 进行判断,避免遥控开启时退出程序报警
    ROS_INFO("Stop control when exiting, cancel alarm");
    msg_que_.push(Command::setDeviceStatus(0x00, 0x0f));
    
    while (!msg_que_.empty()) { // 循环等待发送完成
      usleep(100*1000); 
    }
    retry_cnt++;
    usleep(500*1000);
  }

  loop_running_ = false;
  usleep(10*1000);
  if (!serial_ptr_ && serial_ptr_->isOpen()) {
    serial_ptr_->close();
    serial_ptr_ = nullptr;
  }
}

int RobusterDriver::waitForResponse(int millisecond) {
  struct timespec outtime;
  clock_gettime(CLOCK_MONOTONIC, &outtime);

  outtime.tv_sec += millisecond / 1000;
  outtime.tv_nsec += (millisecond % 1000) * 1000000;
  pthread_mutex_lock(&mutex_);
  int rtn = pthread_cond_timedwait(&cond_, &mutex_, &outtime);
  if (rtn == ETIMEDOUT) {
    ROS_INFO("Timeout.");
  } 
  pthread_mutex_unlock(&mutex_);

  return rtn;
}

void RobusterDriver::emitCondSignal() {
  pthread_mutex_lock(&mutex_);
  pthread_cond_signal(&cond_);
  pthread_mutex_unlock(&mutex_);
}

bool RobusterDriver::init() {

  cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 5, &RobusterDriver::cmdCallback, this);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);

  // device_status: Compatible with the old version
  dev_status_pub_ = nh_.advertise<robuster_mr_msgs::DeviceStatus>("device_status", 10);
  dev_state_pub_ = nh_.advertise<robuster_mr_msgs::DeviceState>("device_state", 10);
  battery_state_pub_ = nh_.advertise<robuster_mr_msgs::BatteryState>("battery_state", 10);
  // aberrant_pub_ = nh_.advertise<>("aberrant", 10);
  light_state_pub_ = nh_.advertise<robuster_mr_msgs::LightState>("light_state", 10);
  motor_state_pub_ = nh_.advertise<robuster_mr_msgs::MotorState>("motor_state", 10);

  get_devinfo_srv_ = nh_.advertiseService("/robuster/get_devInfo", &RobusterDriver::getDevInfoCallback, this);
  light_control_srv_ = nh_.advertiseService("/robuster/light_control", &RobusterDriver::lightControlCallback, this);
  err_dis_srv_ = nh_.advertiseService("/robuster/disable_error", &RobusterDriver::setErrorMask, this);
  new_light_control_ = false;

  loop_running_ = true;

  boost::thread recv_thread(boost::bind(&RobusterDriver::spin, this));
  boost::thread send_thread(boost::bind(&RobusterDriver::sendControlCmd, this));

  // no use now. Stay here, just to be compatible with the old version
  send_dev_status_timer_ = nh_.createTimer(ros::Duration(0.25), &RobusterDriver::sendDevInfo, this);
  publish_data_timer_ = nh_.createTimer(ros::Duration(1/30.0), &RobusterDriver::publishData, this);
}

void RobusterDriver::showData(const char *data, int size, const char *prefix) {
  if (NULL == data || !size) {
    return ;
  }

  if (prefix) {
    printf("%s ", prefix);
  }

  printf("Data(%dBytes): ", size);
  for (int i = 0; i < size; i++) {
    printf("%#x ", (uint8_t)data[i]);
  }
  printf("\n");
}

void RobusterDriver::paraseData(char *data, int size) {
  // showData(data, size);

  if (size < 6) {
    ROS_WARN("Incomplete data segment. Drop.");
    return ;
  } 

  if (data[0] != 0x7f) { // 地址域当前固定为 0x7f
    return ;
  }

  uint16_t data_len = (uint16_t)(*((uint16_t *)&data[2]));
  // ROS_INFO("data_len: %d , size: %d", data_len, size);
  if (data_len + 6 > size) {
    ROS_WARN("Incomplete data length. Drop");
    return ;
  }

  switch (static_cast<uint8_t>(data[1]))
  {
    case DEVICE_STATUS:
      /* handle device status */
      handleDevStatusData(data, size);
      break;
    case ERROR_STATUS:
      /* handle error status */
      handleErrStatusData(data, size);
      break;
    case LIGHT_STATUS:
      handleLightStatusData(data, size);
    case ODOMETRY_DATA:
      /* handle odometry message */
      handleOdomData(data, size);
      break;
    case VEL_STATUS:
      handleVelData(data, size);
      break;
    case MOTOR_TICK_STATUS:
      handleMotorTickStatus(data, size);
      break;
    case MOTOR_RPM_STATUS:
      handleMotorRPMStatus(data, size);
      break;
    case MOTOR_V_STATUS:
      handleMotorVoltageStatus(data, size);
      break;
    case MOTOR_A_STATUS:
      handleMotorCurrentStatus(data, size);
      break;
    case MOTOR_T_STATUS:
      handleMotorTemperatureStatus(data, size);
      break;
    case MOTOR_E_STATUS:
      handleMotorErrorStatus(data, size);
      break;
    /* battery data */
    case BATTERY_STATUS: 
      handleBatteryStatusData(data, size);
      break;
    case BATTERY_CAP_STATUS:
      handleBatteryCapStatusData(data, size);
      break;
    case BATTERY_VA_STATUS:
      handleBatteryVAStatusData(data, size);
      break;
    default:
      break;
  }
}

void RobusterDriver::spin() {
  while(ros::ok() && loop_running_) {
    /******************************************
     * Checking Connection
     * If connection break,than reconnect
     *******************************************/
    if (!serial_ptr_ || !serial_ptr_->isOpen()) {
      try {
        serial_ptr_ = SerialPtr(new serial::Serial(port_name_, baud_rate_, serial::Timeout::simpleTimeout(TIMEOUT)));
        ROS_INFO("Open port(%s) success.", port_name_.c_str());

        msg_que_.push(Command::setDeviceStatus(0x01, 0x0f)); 

      } catch (serial::IOException& e) {
        ROS_ERROR("Open port(%s) failed.Retry", port_name_.c_str());
        ros::Duration(3).sleep();
        continue;
      }
    } else {
      try {
        if (serial_ptr_ && serial_ptr_->waitReadable()) {
          int size = serial_ptr_->read(buffer, sizeof(buffer));
          if (size <= 0) {
            ROS_INFO("read data from serial failed.");
          } else {
            //ROS_INFO("Recveived %dBytes datas", size);
            paraseData((char *)buffer, size);
          }
        }
      } catch (serial::IOException& e) {
        ROS_INFO("Serial read failed. %s", e.what());
        serial_ptr_->close();
      }
    }
  }
}

void RobusterDriver::sendControlCmd() {
  while(loop_running_) {
      vector<uint8_t> msg;

    /// 确保已获取权限. TODO: 解决串口通信问题后,可去除,防止遥控控制时循环发送获取权限指令
    if (cur_dev_state_.control_mode != 0x01 && !disable_permission_) { // 0x01: 即驱动控制权限; 0x02: 表示遥控权限
      msg = Command::setDeviceStatus(0x01, 0x0f); 
      std::queue<vector<uint8_t>> empty;
      std::swap(msg_que_, empty); // 防止指令堆积
    } else {
      if (!msg_que_.empty()) { // 控制命令
        msg = msg_que_.front();
        msg_que_.pop();
      } else {
        // ROS_INFO("Time duration %lf", (ros::Time::now() - last_twist_time_).toSec());
        if ((ros::Time::now() - last_twist_time_).toSec() <= 0.5) {
          msg = Command::setVelocityControl(current_twist_.linear.x, current_twist_.angular.z);
        } else { // 控制超时，停止运动
          msg = Command::setVelocityControl(0.0, 0.0);
        }
      }
    }

    if (!sendCommand(msg)) {
      ROS_WARN("Write serial failed.");
    }

    ros::Duration(1.0/control_rate_).sleep();
  }
}

void RobusterDriver::cmdCallback(const geometry_msgs::Twist::ConstPtr& vel) {
  twist_mutex_.lock();
  last_twist_time_ = ros::Time::now();
  current_twist_ = *(vel.get());
  twist_mutex_.unlock();
}

bool RobusterDriver::sendCommand(const vector<uint8_t> command, bool verb) {
  if (verb) {
    showData((const char*)&command[0], command.size());
  }

  if (serial_ptr_ && serial_ptr_->isOpen()) {
    boost::lock_guard<boost::mutex> lock(write_mutex_);
    try {
      size_t size = serial_ptr_->write(command);
      if (size < command.size()) {
        ROS_ERROR("Send cmd failed.Cmd size: %ldBytes, but only send %ldBytes", command.size(), size);
        return false;
      } 
      // ROS_INFO("Write cmd success. %ld - %ld", size, command.size());
      // serial_ptr_->flush();
    } catch (serial::IOException& e) {
      ROS_INFO("Serial write data failed. %s", e.what());
      return false;
    } 
    return true;
  } else {
    return false;
  }
}

void RobusterDriver::sendDevInfo(const ros::TimerEvent&) {
  // cur_dev_status_
  dev_status_mutex_.lock();
  dev_status_pub_.publish(cur_dev_status_);
  dev_status_mutex_.unlock();
}

// must, or serial segment fault
void RobusterDriver::run() {
  ros::spin();
}

// no use. Compatible with older versions
bool RobusterDriver::getDevInfoCallback(robuster_mr_msgs::DevPermission::Request &req, 
     robuster_mr_msgs::DevPermission::Response &res) {

  return true;
}

bool RobusterDriver::lightControlCallback(robuster_mr_msgs::LightControl::Request &req, 
    robuster_mr_msgs::LightControl::Response &res) {
  // ROS_INFO("Set light status: front(%d), rear(%d)", req.front, req.rear);
  res.status = 1;

  msg_que_.push(Command::setLightStatus(req.front, req.rear));
  if (1) {
    usleep(20 * 1000);
    new_light_control_ = true;

    if (waitForResponse(1000) < 0) {
      res.status = 1;
      res.status_msg = "Set light value timeouted.";
    } else {
      res.status = 0;
      res.status_msg = "success.";
    }
  }

  res.front = cur_light_state_.front_light.value;
  res.rear = cur_light_state_.rear_light.value;

  return true;
}

bool RobusterDriver::setErrorMask(robuster_mr_msgs::ErrorMask::Request &req, 
    robuster_mr_msgs::ErrorMask::Response &res)
{
  res.status = 1;
  uint8_t mask[8] = {0xff}; // 默认全使能
  mask[0] = req.motor;
  mask[1] = req.abnormal;
  mask[2] = req.bumper;
  mask[3] = req.communicate;
  ROS_INFO("ErrorMask: motor(%d), abnormal(%d), bumper(%d), comm(%d)", mask[0], mask[1], mask[2], mask[3]);

  msg_que_.push(Command::disableError(mask, 8));
  if (1) {
    usleep(2500);
    new_error_mask_ = true;

    if (waitForResponse(1000)) {
      res.status = 1;
      res.status_msg = "Set error mask timeouted.";
    } else {
      res.status = 0;
      res.status_msg = "success.";
    }
  }
  return true;
}

void RobusterDriver::handleOdomData(const char *data, const int len) {
  // showData(data, len);

  ros::Time cur_time = ros::Time::now();

  int32_t x, y, th, vel_x, vel_th;
  memcpy(&x, (const void *)&data[4], sizeof(int32_t));
  memcpy(&y, (const void *)&data[8], sizeof(int32_t));
  memcpy(&th, (const void *)&data[12], sizeof(int32_t));
  memcpy(&vel_x, (const void *)&data[16], sizeof(int32_t));
  memcpy(&vel_th, (const void *)&data[20], sizeof(int32_t));
  // ROS_INFO("Odom value: %ld, %ld, %ld", x, y, th);
  // ROS_INFO("Odom value: %lf, %lf, %lf", x/1000.0, y/1000.0, th/1000.0);
  // ROS_INFO("vel value: %lf, %lf", vel_x/1000.0, vel_th/1000.0);

  cur_dev_state_.linear_velocity = vel_x/1000.0;
  cur_dev_state_.angular_velocity = vel_th/1000.0;

  geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(th/1000.0);

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = cur_time;
  odom_trans.header.frame_id = odom_frame_;
  odom_trans.child_frame_id = base_frame_;

  odom_trans.transform.translation.x = x/1000.0;
  odom_trans.transform.translation.y = y/1000.0;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = orientation;

  /* Don't need odom transform for 3D SLAM */
  if (use_odom_trans_) {
    odom_broadcaster_.sendTransform(odom_trans);
  }

  nav_msgs::Odometry odom;
  odom.header.stamp = cur_time;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;

  //set the position
  odom.pose.pose.position.x = x/1000.0;
  odom.pose.pose.position.y = y/1000.0;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = orientation;

  //set the velocity
  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = 0.0;

  odom.pose.covariance[0]  = 0.1;   
  odom.pose.covariance[7]  = 0.1;	
  odom.pose.covariance[35] = 0.2;   

  odom.pose.covariance[14] = 1e10; 	
  odom.pose.covariance[21] = 1e10; 	
  odom.pose.covariance[28] = 1e10; 

  //publish the message
  odom_pub_.publish(odom);
}

void RobusterDriver::handleOdomOrientationData(const char *data, const int len) {

}

void RobusterDriver::handleVelData(const char *data, const int len) {

}

void RobusterDriver::handleMotorStatus(const char *data, const int len) {
  // Deprecated
  int32_t rpm = (int)(*((int *)&data[4]));
  int32_t position = (int)(*((int *)&data[8]));
}

void RobusterDriver::handleMotorVAStatus(const char *data, const int len) {
  // Deprecated
}

void RobusterDriver::handleMotorTickStatus(const char *data, const int len) {
  for (int i = 0; i < MOTOR_COUNTS; i++) {
    memcpy(&cur_motor_state_.motor_states[i].tick, (const void *)&data[4 * i], sizeof(int32_t));
  }
}

void RobusterDriver::handleMotorRPMStatus(const char *data, const int len) {

  for (int i = 0; i < MOTOR_COUNTS; i++) {
    memcpy(&cur_motor_state_.motor_states[i].rpm, (const void *)&data[4 * i], sizeof(int32_t));
  }
}

void RobusterDriver::handleMotorVoltageStatus(const char *data, const int len) {
  uint32_t voltage = 0;
  for (int i = 0; i < MOTOR_COUNTS; i++) {
    memcpy(&voltage, (const void *)&data[2 * i], sizeof(uint32_t));
    cur_motor_state_.motor_states[i].voltage = voltage / 100.0;
  }
}

void RobusterDriver::handleMotorCurrentStatus(const char *data, const int len) {
  int32_t current = 0;
  for (int i = 0; i < MOTOR_COUNTS; i++) {
    memcpy(&current, (const void *)&data[4 * i], sizeof(int32_t));
    cur_motor_state_.motor_states[i].voltage = current / 100.0;
  }
}

void RobusterDriver::handleMotorTemperatureStatus(const char *data, const int len) {
  /// TODO:
}

void RobusterDriver::handleMotorErrorStatus(const char *data, const int len) {
  int fault_value = 0;
  for (int i = 0; i < MOTOR_COUNTS; i++) {
    
    memcpy(&fault_value, (const void *)&data[4 * i], sizeof(int16_t));
    // if motor fault happened, clean the state of this motor
    if (fault_value > 0) {
      memset(&cur_motor_state_, 0, sizeof(cur_motor_state_));
    } 
    
    cur_motor_state_.motor_states[i].fault = fault_value;
  }
}

void RobusterDriver::handleDevStatusData(const char *data, const int len) {
  boost::lock_guard<boost::mutex> lock(dev_status_mutex_);
  // cur_dev_status_
  int8_t control_mode;
  memcpy(&control_mode, (const void *)&data[4], sizeof(control_mode));
  cur_dev_status_.controlState = control_mode;
  cur_dev_state_.control_mode = control_mode;
}

void RobusterDriver::handleErrStatusData(const char *data, const int len) {
  // showData(data, len);
  boost::lock_guard<boost::mutex> lock(dev_status_mutex_);
  if (len < 14) {
    ROS_ERROR("Unvailed serial data.");
    return;
  }

  cur_dev_status_.errorCode = 0;
  cur_dev_status_.errorCode |= data[4];       // motor_state
  cur_dev_status_.errorCode |= data[6] << 4;  // bumper state
  cur_dev_status_.errorCode |= data[7] << 6;  // communication state

  cur_dev_state_.e_stop = data[5] & 0x01; // emergency stop
  cur_dev_state_.front_bumper = data[6] & 0x01;
  cur_dev_state_.rear_bumper = data[6] & 0x02;

}

void RobusterDriver::handleErrorMaskData(const char *data, const int len) {
  // showData(data, len);
  boost::lock_guard<boost::mutex> lock(light_mutex_);

  if (len < 14) {
    ROS_ERROR("Unvailed error mask msg.");
    return;
  }

  if (new_error_mask_) {
    ROS_INFO("Emit cond signal for setErrorMask");
    emitCondSignal();
    new_light_control_ = false;
  }
}

void RobusterDriver::handleLightStatusData(const char *data, const int len) {
  // showData(data, len);
  boost::lock_guard<boost::mutex> lock(light_mutex_);

  if (len < 14) {
    ROS_ERROR("Unvailed light state msg.");
    return;
  }

  int front_light_value, rear_light_value;
  memcpy(&front_light_value, (const void *)&data[4], sizeof(front_light_value));
  memcpy(&rear_light_value, (const void *)&data[8], sizeof(rear_light_value));

  cur_light_state_.front_light.value = static_cast<char>(front_light_value);
  cur_light_state_.rear_light.value = static_cast<char>(rear_light_value);

  cur_dev_state_.front_light = static_cast<int16_t>(front_light_value);
  cur_dev_state_.rear_light = static_cast<int16_t>(rear_light_value);

  if (new_light_control_) {
    ROS_INFO("Emit cond signal for light control.(%d, %d)", front_light_value, rear_light_value);
    emitCondSignal();
    new_light_control_ = false;
  }
}

void RobusterDriver::handleBatteryStatusData(const char *data, const int len) {
  // showData(data, len);
  unsigned int battery_percentage = 0;
  unsigned int battery_temeprature = 0;
  memcpy(&battery_percentage, (const void *)&data[4], sizeof(battery_percentage));
  memcpy(&battery_temeprature, (const void *)&data[8], sizeof(battery_temeprature));
  if (battery_percentage <= 100) {
    battery_mutex_.lock();
    cur_dev_status_.powerExtra = (char)battery_percentage;

    cur_battery_state_.battery_percentage = battery_percentage;
    cur_battery_state_.battery_temperature = battery_temeprature / 10.0;

    cur_dev_state_.battery_percentage = battery_percentage;
    battery_mutex_.unlock();
  } else {
    ROS_WARN("Invalid power percentage value(%d)", battery_percentage);
  }
}

void RobusterDriver::handleBatteryCapStatusData(const char *data, const int len) {
  
}

void RobusterDriver::handleBatteryVAStatusData(const char *data, const int len) {
  // showData(data, len);
  unsigned int voltage = 0;
  int current = 0;  // 负数表示放电
  memcpy(&voltage, (const void *)&data[4], sizeof(voltage));
  memcpy(&current, (const void *)&data[8], sizeof(current));
  // ROS_INFO("Get battery VA: %d, %d", voltage, current);
  battery_mutex_.lock();
  if (voltage <= VOLTAGE_MAX) {
    cur_battery_state_.voltage = voltage / 100.0;
  } else {
    ROS_WARN("Invalid voltage value(%d)", voltage);
  }

  if (current <= CURRENT_MAX) {
    cur_battery_state_.current = current / 100.0;
    cur_dev_state_.battery_current = current / 100.0;
  } else {
    ROS_WARN("Invalid current value(%d)", current);
  }
  
  battery_mutex_.unlock();
}

void RobusterDriver::publishData(const ros::TimerEvent&) {
  /// TODO: publish while value changed
  if (battery_state_pub_.getNumSubscribers() != 0) {
    battery_state_pub_.publish(cur_battery_state_);
  }

  if (light_state_pub_.getNumSubscribers() != 0) {
    light_state_pub_.publish(cur_light_state_);
  }

  if (dev_state_pub_.getNumSubscribers() != 0) {
    dev_state_pub_.publish(cur_dev_state_);
  }

  if (motor_state_pub_.getNumSubscribers() != 0){
    motor_state_pub_.publish(cur_motor_state_);
  }
}
