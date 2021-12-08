/**************************************************************************
 * Copyright (C), 2020-2025, Robot++ Co., Ltd.
 * Author: Damon
 * version: 0.1
 * Date: 2020-12-03
 * 
 * Description: Serial Driver
 **************************************************************************/

#ifndef __SERIAL_NODE_H__

#include <serial/serial.h>
#include <string>
#include <queue>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <pthread.h>
#include <errno.h>

#include "robuster_mr_msgs/DevPermission.h"
#include "robuster_mr_msgs/LightControl.h"
#include "robuster_mr_msgs/DeviceStatus.h"  // compatible older versions
#include "robuster_mr_msgs/DeviceState.h"
#include "robuster_mr_msgs/BatteryState.h"
#include "robuster_mr_msgs/LightState.h"
#include "robuster_mr_msgs/MotorState.h"
#include "robuster_mr_msgs/ErrorMask.h"

using namespace std;

class RobusterDriver {
  typedef boost::shared_ptr<serial::Serial> SerialPtr;

 public:
  RobusterDriver();
  ~RobusterDriver();

  int waitForResponse(int millisecond);
  void emitCondSignal();

  void closeDriver();

  bool init();

  void spin();

  void sendControlCmd();

  void run();

  // No use now. Compatible with older versions
  bool getDevInfoCallback(robuster_mr_msgs::DevPermission::Request &req, 
                          robuster_mr_msgs::DevPermission::Response &res);

  bool lightControlCallback(robuster_mr_msgs::LightControl::Request &req, 
                            robuster_mr_msgs::LightControl::Response &res);

  bool setErrorMask(robuster_mr_msgs::ErrorMask::Request &req,
                    robuster_mr_msgs::ErrorMask::Response &res);

 private:
  bool sendCommand(const vector<uint8_t> command, bool verb = false);

  void showData(const char *data, int size, const char *prefix = nullptr);
  void paraseData(char *data, int size);

  void cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd);

  // Timer callback
  void sendControlCmdCb(const ros::TimerEvent&);

  void sendDevInfo(const ros::TimerEvent&);

  void publishData(const ros::TimerEvent&);

  /** 底盘上报的里程信息. 包括 dx 和 dy
   *    \protocol: 0x7f 0x9e 0x0012 0x00 0x00 ...... 0xffff
   *      data[4]: (4Bytes) dx
   *      data[8]: (4Bytes) dy
   *      data[12]: (2Bytes) θ
   *      data[14]: (4Bytes) 线速度
   *      data[18]: (4Bytes) 角速度
   */
  void handleOdomData(const char *data, const int len);

  /** 底盘上报的里程信息. 包括方向
   *    \protocol: 0x7f 0x9d 0x0008 0x01 0x03 0x06 0x04 0x00 0x00 0x00 0x00 0xffff
   *      data[4]: (4Bytes) 方向角
   *      data[8]: (4Bytes) 保留
   */
  void handleOdomOrientationData(const char *data, const int len);

  /** 底盘上报的速度信息. 包括线速度和角速度
   *    \protocol: 0x7f 0x9f 0x0008 0x01 0x03 0x06 0x04 0x00 0x00 0x00 0x00 0xffff
   *      data[4]: (4Bytes) 线速度
   *      data[8]: (4Bytes) 角速度
   */
  void handleVelData(const char *data, const int len);

  /** 电机状态. 包括转速和位置 (Deprecated)
   *    \protocol: 0x7f 0x9[0-3] 0x0008 0x01 0x03 0x06 0x04 0x00 0x00 0x00 0x00 0xffff
   *      data[1]: 0x90 - 左前电机
   *               0x91 - 右前电机
   *               0x92 - 左后电机
   *               0x93 - 右后电机
   *      data[4]: (4Bytes) 转速
   *      data[8]: (4Bytes) 位置
   */
  void handleMotorStatus(const char *data, const int len);

  /** 电机电压及电流数据 (Deprecated)
   *    \protocol: 0x7f 0xa[0-3] 0x0008 0x01 0x03 0x06 0x04 0x00 0x00 0x00 0x00 0xffff
   *      data[1]: 0xa0 - 左前电机
   *               0xa1 - 右前电机
   *               0xa2 - 左后电机
   *               0xa3 - 右后电机
   *      data[4]: (4Bytes) 电压
   *      data[8]: (4Bytes) 电流
   */
  void handleMotorVAStatus(const char *data, const int len);

  /** 电机位置(即: 脉冲)
   *    \protocol: 0x7f 0x90 0x0010 0x01 0x03 0x06 0x04 0x00 0x00 0x00 0x00 0xffff
   *      data[4]: (4Bytes) 左前电机脉冲
   *      data[8]: (4Bytes) 右前电机脉冲
   *      data[12]: (4Bytes) 左后电机脉冲
   *      data[16]: (4Bytes) 右后电机脉冲
   */
  void handleMotorTickStatus(const char *data, const int len);

  /** 电机转速
   *    \protocol: 0x7f 0x91 0x0010 0x01 0x03 0x06 0x04 0x00 0x00 0x00 0x00 0xffff
   *      data[4]: (4Bytes) 左前电机转速
   *      data[8]: (4Bytes) 右前电机转速
   *      data[12]: (4Bytes) 左后电机转速
   *      data[16]: (4Bytes) 右后电机转速
   */
  void handleMotorRPMStatus(const char *data, const int len);

  /** 电机电流
   *    \protocol: 0x7f 0xa0 0x0008 0x01 0x03 0x06 0x04 0x00 0x00 0x00 0x00 0xffff
   *      data[4]: (2Bytes) 左前电机电流
   *      data[6]: (2Bytes) 右前电机电流
   *      data[8]: (2Bytes) 左后电机电流
   *      data[10]: (2Bytes) 右后电机电流
   */
  void handleMotorVoltageStatus(const char *data, const int len);

  /** 电机电压
   *    \protocol: 0x7f 0xa1 0x0008 0x01 0x03 0x06 0x04 0x00 0x00 0x00 0x00 0xffff
   *      data[4]: (2Bytes) 左前电机电压
   *      data[6]: (2Bytes) 右前电机电压
   *      data[8]: (2Bytes) 左后电机电压
   *      data[10]: (2Bytes) 右后电机电压
   */
  void handleMotorCurrentStatus(const char *data, const int len);

  /** 电机温度
   *    \protocol: 0x7f 0xa2 0x0008 0x01 0x03 0x06 0x04 0x00 0x00 0x00 0x00 0xffff
   *      data[4]: (2Bytes) 左前电机温度
   *      data[6]: (2Bytes) 右前电机电压
   *      data[8]: (2Bytes) 左后电机电压
   *      data[10]: (2Bytes) 右后电机电压
   */
  void handleMotorTemperatureStatus(const char *data, const int len);

  /** 电机异常
   *    \protocol: 0x7f 0xa3 0x0008 0x01 0x03 0x06 0x04 0x00 0x00 0x00 0x00 0xffff
   *      data[4]: (2Bytes) 左前电机异常
   *      data[6]: (2Bytes) 右前电机异常
   *      data[8]: (2Bytes) 左后电机异常
   *      data[10]: (2Bytes) 右后电机异常
   */
  void handleMotorErrorStatus(const char *data, const int len);

  /** 设备状态
   *    \protocol: 0x7f 0xf4 0x0008 0x01 0x03 0x06 0x04 0x00 0x00 0x00 0x00 0xffff
   *      data[4]: (1Byte) 控制状态. 0 - IDLE, 1 - pc, 2 - joy
   *      data[5]: (1Byte) 电源
   *      data[6]: (1Byte) 动力
   *      data[7]: (1Byte) 速比, 即档位: 1、2、3
   *      data[8-12]: 保留
   */
  void handleDevStatusData(const char *data, const int len);

  /** 错误状态
   *    \protocol: 0x7f 0xf0 0x0008 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0xffff
   *      data[4]: (1Byte) 电机. 弃用(电机状态已移至 0xa3 协议)
   *          - bit 0: 左前电机状态(1: 故障, 0: 正常)
   *          - bit 1: 右前电机状态(1: 故障, 0: 正常)
   *          - bit 2: 左后电机状态(1: 故障, 0: 正常)
   *          - bit 3: 右后电机状态(1: 故障, 0: 正常)
   *      data[5]: (1Byte) 系统异常
   *          - bit 0: 急停(1: 触发, 0: 故障)
   *      data[6]: (1Byte) 限位
   *          - bit 0: 前限位状态(1: 触发, 0: 释放)
   *          - bit 1: 后限位状态(1: 触发, 0: 释放)
   *      data[7]: (1Byte) 通信
   *          - bit 0: 通信状态(1: 故障, 0: 正常). 电机通信异常
   *          - bit 1: 遥控状态(1: 故障, 0: 正常)
   *          - bit 2: 动力总线
   *          - bit 3: 扩展总线
   *          - bit 4: 电池
   *          - bit 5: 显示
   *      data[8-11]: (4Byte) 保留
   */
  void handleErrStatusData(const char *data, const int len);

  /** 错误状态
   *    \protocol: 0x7f 0xf5 0x0008 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0xffff
   *      data[4]: (4Byte) 前灯亮度
   *      data[8]: (4Byte) 后灯亮度
   */
  void handleLightStatusData(const char *data, const int len);

  void handleErrorMaskData(const char *data, const int len);

  /** 电池状态.包括电量百分比和电池温度 
   *    \protocol: 0x7f 0xef 0x0008 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0xffff
   *      data[4]: (4Bytes) 剩余容量
   *      data[8]: (4Bytes) 总容量
   */
  void handleBatteryStatusData(const char *data, const int len);

  /* 电池容量.包括剩余容量和标称容量 */
  void handleBatteryCapStatusData(const char *data, const int len);
  /* 电池电压及电流状态.包括电池电压和电池电流 */
  void handleBatteryVAStatusData(const char *data, const int len);

 private:
  ros::NodeHandle nh_;

  ros::Subscriber cmd_sub_;

  tf::TransformBroadcaster odom_broadcaster_;
  ros::Publisher odom_pub_;

  ros::ServiceServer get_devinfo_srv_;

  ros::Timer send_dev_status_timer_; // no use. Compatible with the old version
  ros::Timer publish_data_timer_;

  string odom_frame_;
  string base_frame_;

  bool loop_running_;

  SerialPtr serial_ptr_;
  string port_name_;
  int baud_rate_;
  int control_rate_;

  bool use_odom_trans_;

  boost::mutex write_mutex_;

  // twist info
  boost::mutex twist_mutex_;
  geometry_msgs::Twist current_twist_;
  ros::Time last_twist_time_;
  
  uint8_t buffer[32];

  // device status. Compatible with the old version
  ros::Publisher dev_status_pub_;
  boost::mutex dev_status_mutex_;
  robuster_mr_msgs::DeviceStatus cur_dev_status_;

  // device state. Instand of device status
  ros::Publisher dev_state_pub_;
  boost::mutex dev_state_mutex_;
  robuster_mr_msgs::DeviceState cur_dev_state_;

  // battery state
  ros::Publisher battery_state_pub_;  
  boost::mutex battery_mutex_;
  robuster_mr_msgs::BatteryState cur_battery_state_;

  // light control and state
  ros::ServiceServer light_control_srv_;
  bool new_light_control_;
  ros::ServiceServer err_dis_srv_;
  bool new_error_mask_;
  ros::Publisher light_state_pub_;
  boost::mutex light_mutex_;
  robuster_mr_msgs::LightState cur_light_state_;

  
  
  // motor state
  ros::Publisher motor_state_pub_;
  boost::mutex motor_mutex_;
  robuster_mr_msgs::MotorState cur_motor_state_;

  long int last_send_time_;

  /// TODO:
  pthread_mutex_t     mutex_;
  pthread_condattr_t  attr_;
  pthread_cond_t      cond_;

  queue<vector<uint8_t>> msg_que_;

  bool disable_permission_;
};

#endif