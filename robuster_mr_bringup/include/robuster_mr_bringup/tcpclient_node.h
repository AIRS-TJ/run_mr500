#ifndef __TCPCLIENT_NODE_H__
#define __TCPCLIENT_NODE_H__



#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "robuster_mr_msgs/sonarData.h"
#include "robuster_mr_msgs/DeviceStatus.h"
#include "robuster_mr_msgs/encoderData.h"
#include "robuster_mr_msgs/controlCmdData.h"
#include "robuster_mr_msgs/getDevInfo.h"
#include "robuster_mr_msgs/setPermission.h"
#include "robuster_mr_msgs/resetAlarm.h"
#include "robuster_mr_msgs/LightControl.h"
#include "robuster_mr_msgs/enableUltrasound.h"
#include "robuster_mr_msgs/resetImu.h"

#include <semaphore.h>

#include "tcpclient.h"


namespace mr_tool {
class TcpClient
{
public:
	TcpClient();
	~TcpClient();
	
	void run();
private:
	ros::NodeHandle nh_;

	ros::Publisher imu_pub_;
	ros::Publisher encoder_pub_;
	ros::Publisher sonar_pub_;
	ros::Publisher car_pub_;

	ros::Publisher odom_pub_;

	ros::Subscriber cmd_sub_;
	ros::Subscriber encoder_init_sub_;

	ros::ServiceServer control_cmd_sub_;
	ros::ServiceServer get_devInfo_;
	ros::ServiceServer set_permission_;
	ros::ServiceServer reset_alarm_;
	ros::ServiceServer light_control_;
	ros::ServiceServer enable_ultrasound_;
	ros::ServiceServer reset_imu_;

	CTCPClient ctcpclient_;	

	sem_t done_;
	pthread_cond_t serverCond_;
	pthread_mutex_t serverMutex_;

	boost::mutex cmd_mutex_;
	double wheelSpace_, wheelDiam_,wheelRate_;
	bool tfUsed_;
	std::string odomFrame_, baseFrame_, imuFrame_;
	double vx_, th_;


	char recvdata_[1024] = {0};
	char recvdata_temp_[1024] = {0};
	unsigned char recvdata_u_[1024] = {0};

	char senddata_[1024] = {0};
	unsigned char senddata_u_[1024] = {0};

	int sendlenth_ = 0;
	unsigned char result_ = 0x10;

	void cmdcallback(const geometry_msgs::Twist::ConstPtr& cmd);
	bool controlcmdcallback(unsigned char father, unsigned char child, unsigned char *data);
	bool sendDataCombined(unsigned char father,unsigned char child,unsigned char *data);

	bool get_devInfocallback(robuster_mr_msgs::getDevInfo::Request &req, robuster_mr_msgs::getDevInfo::Response &res);
	bool set_permissioncallback(robuster_mr_msgs::setPermission::Request &req, robuster_mr_msgs::setPermission::Response &res);
	bool reset_alarmcallback(robuster_mr_msgs::resetAlarm::Request &req, robuster_mr_msgs::resetAlarm::Response &res);
	bool light_controlcallback(robuster_mr_msgs::LightControl::Request &req, robuster_mr_msgs::LightControl::Response &res);
	bool enable_ultrasoundcallback(robuster_mr_msgs::enableUltrasound::Request &req, robuster_mr_msgs::enableUltrasound::Response &res);
	bool reset_imucallback(robuster_mr_msgs::resetImu::Request &req, robuster_mr_msgs::resetImu::Response &res);

	void encodercallback(const robuster_mr_msgs::encoderData::ConstPtr& encoderMsg);

	unsigned short crc16(unsigned char *data, int size, bool invert);

	bool dataAnalysis(unsigned char *data,int size);
	bool dataSelect(unsigned char *data,int size);

	void recvdata();
	void senddata();

	void get_abstime_wait(int microseconds, struct timespec *abstime);
};

}




#endif

