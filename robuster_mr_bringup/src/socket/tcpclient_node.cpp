#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>


#include <boost/bind.hpp>
#include <boost/thread.hpp>


#include "tcpclient_node.h"

#define pi 3.1415926
//转换为左轮转速
#define vel_to_rpm_left(liner_vel_x,angular_rad_z) ((liner_vel_x * 60.0 - angular_rad_z * 60 * wheelSpace_ / 2) / pi / wheelDiam_)
//转换为右轮转速
#define vel_to_rpm_right(liner_vel_x,angular_rad_z) ((liner_vel_x * 60.0 + angular_rad_z * 60 * wheelSpace_ / 2) / pi / wheelDiam_)

//转换为角速度
#define rpm_to_liner_vel_th(CurRpmL,CurRpmR) (((CurRpmR - CurRpmL) / 2.0 / 60.0 * pi * wheelDiam_) / wheelSpace_ * 2)
//转换为线速度
#define rpm_to_liner_vel_x(CurRpmL, CurRpmR) ((CurRpmL + CurRpmR) / 2.0 / 60.0 * pi * wheelDiam_)

//空间开始
namespace mr_tool {

//构造函数
TcpClient::TcpClient()
{
	ros::NodeHandle private_node("~");

	private_node.param("odom_frame", odomFrame_, std::string("odom"));
  	private_node.param("base_link_frame", baseFrame_, std::string("base_link"));
	private_node.param("imu_link_frame", imuFrame_, std::string("imu_link"));

	private_node.param("wheelSpace", wheelSpace_, 0.5);
  	private_node.param("wheelDiam", wheelDiam_, 0.2);
	private_node.param("wheelRate", wheelRate_, 32.0);
	
	private_node.param("tfUsed", tfUsed_, false);
}

//析构函数
TcpClient::~TcpClient()
{
	
}

//速度回调函数
void TcpClient::cmdcallback(const geometry_msgs::Twist::ConstPtr& cmd)
{
	cmd_mutex_.lock();
	double vx_ = cmd->linear.x;
	double th_ = cmd->angular.z;

	short leftRpm = (short)vel_to_rpm_left(vx_,th_) * wheelRate_;
	short rightRpm = (short)vel_to_rpm_right(vx_,th_) * wheelRate_;

//	ROS_INFO("leftRpm is %f  rightRpm is %f",(double)leftRpm / 32,(double)rightRpm / 32);

	unsigned char temp[4] = {0x0,0x0,0x0,0x0};

	temp[0] = *((unsigned char *)&leftRpm + 1);
	temp[1] = *((unsigned char *)&leftRpm);

	temp[2] = *((unsigned char *)&rightRpm + 1);
	temp[3] = *((unsigned char *)&rightRpm);

//	memcpy(temp,(unsigned char *)&leftRpmTemp,2);
//	memcpy(&temp[2],(unsigned char *)&rightRpmTemp,2);
	sendDataCombined(0x02,0x01,temp);

	cmd_mutex_.unlock();
}

//查询设备信息
bool TcpClient::get_devInfocallback(robuster_mr_msgs::getDevInfo::Request &req, robuster_mr_msgs::getDevInfo::Response &res)
{
	unsigned char temp[2] = {0};
	controlcmdcallback(0x02, 0x06, temp);

	//等待反馈
	struct timespec outtime;
	get_abstime_wait(600,&outtime);
	pthread_mutex_lock(&serverMutex_);
	int ret = pthread_cond_timedwait(&serverCond_, &serverMutex_, &outtime);
	pthread_mutex_unlock(&serverMutex_);

	ROS_INFO("get_devInfocallback pthread_cond_timedwait: %d",ret);
	res.result = ret;
	return 1;
}

//获取控制权限
bool TcpClient::set_permissioncallback(robuster_mr_msgs::setPermission::Request &req, robuster_mr_msgs::setPermission::Response &res)
{
	unsigned char temp[2];

	temp[0] = req.permission;
	controlcmdcallback(0x01, 0x02, temp);

	//等待反馈
	struct timespec outtime;
	get_abstime_wait(600,&outtime);
	pthread_mutex_lock(&serverMutex_);
	int ret = pthread_cond_timedwait(&serverCond_, &serverMutex_, &outtime);
	pthread_mutex_unlock(&serverMutex_);

	ROS_INFO("set_permissioncallback pthread_cond_timedwait: %d",ret);
	res.result = ret;
	return 1;
}

//解除防撞报警
bool TcpClient::reset_alarmcallback(robuster_mr_msgs::resetAlarm::Request &req, robuster_mr_msgs::resetAlarm::Response &res)
{
	unsigned char temp[2];

	temp[0] = req.permission;
	controlcmdcallback(0x01, 0x03, temp);

	//等待反馈
	struct timespec outtime;
	get_abstime_wait(600,&outtime);
	pthread_mutex_lock(&serverMutex_);
	int ret = pthread_cond_timedwait(&serverCond_, &serverMutex_, &outtime);
	pthread_mutex_unlock(&serverMutex_);

	ROS_INFO("reset_alarmcallback pthread_cond_timedwait: %d",ret);
	res.result = ret;
	return 1;

}

//开关灯
bool TcpClient::light_controlcallback(robuster_mr_msgs::LightControl::Request &req, robuster_mr_msgs::LightControl::Response &res)
{
	unsigned char temp[2];

	temp[0] = req.front;
	temp[1] = req.rear;

	controlcmdcallback(0x02, 0x03, temp);

	//等待反馈
	struct timespec outtime;
	get_abstime_wait(600,&outtime);
	pthread_mutex_lock(&serverMutex_);
	int ret = pthread_cond_timedwait(&serverCond_, &serverMutex_, &outtime);
	pthread_mutex_unlock(&serverMutex_);

	ROS_INFO("light_controlcallback pthread_cond_timedwait: %d",ret);
	res.result = ret;
	return 1;
}

//设置超声波开关
bool TcpClient::enable_ultrasoundcallback(robuster_mr_msgs::enableUltrasound::Request &req, robuster_mr_msgs::enableUltrasound::Response &res)
{
	unsigned char temp[2];

	temp[0] = req.value;
	controlcmdcallback(0x02, 0x0b, temp);

	//等待反馈
	struct timespec outtime;
	get_abstime_wait(600,&outtime);
	pthread_mutex_lock(&serverMutex_);
	int ret = pthread_cond_timedwait(&serverCond_, &serverMutex_, &outtime);
	pthread_mutex_unlock(&serverMutex_);

	ROS_INFO("enable_ultrasoundcallback pthread_cond_timedwait: %d",ret);
	res.result = ret;
	return 1;
}

//IMU 复位
bool TcpClient::reset_imucallback(robuster_mr_msgs::resetImu::Request &req, robuster_mr_msgs::resetImu::Response &res)
{
	unsigned char temp[2];

	temp[0] = req.value;
	controlcmdcallback(0x02, 0x0c, temp);

	//等待反馈
	struct timespec outtime;
	get_abstime_wait(600,&outtime);
	pthread_mutex_lock(&serverMutex_);
	int ret = pthread_cond_timedwait(&serverCond_, &serverMutex_, &outtime);
	pthread_mutex_unlock(&serverMutex_);

	ROS_INFO("reset_imucallback pthread_cond_timedwait: %d",ret);
	res.result = ret;
	return 1;
}

//控制命令回调函数
bool TcpClient::controlcmdcallback(unsigned char father, unsigned char child, unsigned char *data)
{
	cmd_mutex_.lock();

	sendDataCombined(father,child,data);

	cmd_mutex_.unlock();

}

//下发数据组合
bool TcpClient::sendDataCombined(unsigned char father,unsigned char child,unsigned char *data)
{
	senddata_u_[0] = 0xF4;
	senddata_u_[1] = 0xF5;

	senddata_u_[2] = 0x00;
	senddata_u_[3] = 0x00;
	senddata_u_[4] = 0x00;
	senddata_u_[5] = 0x00;

	if(father == 0x01)
	{
		switch (child)
		{
			case 0x01:
			{
				senddata_u_[6] = 0x01;
				senddata_u_[7] = 0x01;
				senddata_u_[8] = 0x01;
				senddata_u_[9] = data[0];
				unsigned short res = crc16(&senddata_u_[2],8,1);
				senddata_u_[10] = (res & 0xFF00) >> 8;
				senddata_u_[11] = (res & 0x00FF);
				sendlenth_ = 12;

				break;
			}

			case 0x02:
			{
				senddata_u_[6] = 0x01;
				senddata_u_[7] = 0x02;
				senddata_u_[8] = 0x01;
				senddata_u_[9] = data[0];
				unsigned short res = crc16(&senddata_u_[2],8,1);
				senddata_u_[10] = (res & 0xFF00) >> 8;
				senddata_u_[11] = (res & 0x00FF);
				sendlenth_ = 12;

				break;
			}

			case 0x03:
			{
				senddata_u_[6] = 0x01;
				senddata_u_[7] = 0x03;
				senddata_u_[8] = 0x01;
				senddata_u_[9] = data[0];
				unsigned short res = crc16(&senddata_u_[2],8,1);
				senddata_u_[10] = (res & 0xFF00) >> 8;
				senddata_u_[11] = (res & 0x00FF);
				sendlenth_ = 12;

				break;
			}

			default:
				break;
		}
	}

	if(father == 0x02)
	{
		switch (child)
		{
			//速度
			case 0x01:
			{
				senddata_u_[6] = 0x02;
				senddata_u_[7] = 0x01;
				senddata_u_[8] = 0x05;
				senddata_u_[9] = 0x00;
				memcpy(&senddata_u_[10],data,4);
				unsigned short res = crc16(&senddata_u_[2],12,1);
				senddata_u_[14] = (res & 0xFF00) >> 8;
				senddata_u_[15] = (res & 0x00FF);
				sendlenth_ = 16;
				break;
			}

			//车灯
			case 0x03:
			{
				senddata_u_[6] = 0x02;
				senddata_u_[7] = 0x03;
				senddata_u_[8] = 0x02;
				senddata_u_[9] = data[0];
				senddata_u_[10] = data[1];
				unsigned short res = crc16(&senddata_u_[2],9,1);
				senddata_u_[11] = (res & 0xFF00) >> 8;
				senddata_u_[12] = (res & 0x00FF);
				sendlenth_ = 13;
				break;
			}

			//设备信息
			case 0x06:
			{
				senddata_u_[6] = 0x02;
				senddata_u_[7] = 0x06;
				senddata_u_[8] = 0x00;
				unsigned short res = crc16(&senddata_u_[2],7,1);
				senddata_u_[9] = (res & 0xFF00) >> 8;
				senddata_u_[10] = (res & 0x00FF);
				sendlenth_ = 11;
				break;
			}

			//ip地址
			case 0x07:
			{
				senddata_u_[6] = 0x02;
				senddata_u_[7] = 0x07;
				senddata_u_[8] = 0x06;
				memcpy(&senddata_u_[9],data,4);
				memcpy(&senddata_u_[13],data,2);
				unsigned short res = crc16(&senddata_u_[2],13,1);
				senddata_u_[14] = (res & 0xFF00) >> 8;
				senddata_u_[15] = (res & 0x00FF);
				sendlenth_ = 16;
				break;
			}

			//电源开关
			case 0x08:
			{
				senddata_u_[6] = 0x02;
				senddata_u_[7] = 0x08;
				senddata_u_[8] = 0x01;
				senddata_u_[9] = data[0];
				unsigned short res = crc16(&senddata_u_[2],8,1);
				senddata_u_[10] = (res & 0xFF00) >> 8;
				senddata_u_[11] = (res & 0x00FF);
				sendlenth_ = 12;

				break;
			}

			//超声波开关
			case 0x0b:
			{
				senddata_u_[6] = 0x02;
				senddata_u_[7] = 0x0b;
				senddata_u_[8] = 0x01;
				senddata_u_[9] = data[0];
				unsigned short res = crc16(&senddata_u_[2],8,1);
				senddata_u_[10] = (res & 0xFF00) >> 8;
				senddata_u_[11] = (res & 0x00FF);
				sendlenth_ = 12;

				break;
			}

			//imu复位
			case 0x0c:
			{
				senddata_u_[6] = 0x02;
				senddata_u_[7] = 0x0c;
				senddata_u_[8] = 0x01;
				senddata_u_[9] = data[0];
				unsigned short res = crc16(&senddata_u_[2],8,1);
				senddata_u_[10] = (res & 0xFF00) >> 8;
				senddata_u_[11] = (res & 0x00FF);
				sendlenth_ = 12;

				break;
			}
					
			default:
				break;
		}
	}
}

//获得校验码  发送invert=1  接收invert=0
unsigned short TcpClient::crc16(unsigned char *data, int size, bool invert)
{
	unsigned short a = 0xFFFF;
	unsigned short b = 0xA001;
	for(int i = 0; i < size; i++)
	{
		a ^= data[i];
		for(int i = 0; i < 8; i ++)
		{
			unsigned short last = a % 2;
			a >>= 1;
			if(last == 1)
				a ^= b;
		}
	}
	if(invert == 1)
		return ((a & 0xFF00) | (a & 0x00FF));
	else
		return ((a & 0x00FF) | (a & 0xFF00));
}

//解析数据
bool TcpClient::dataAnalysis(unsigned char *data,int size)
{
	static int extraNcount = 0;
	static unsigned char extra[1024];

	static int first = 0;

	int temI;

	if(size <= 0)
		return 0;

	//接收的第一真数据判断真头
	if(extraNcount == 0)
	{
		first = 1;
		for(temI = 0; temI < size - 1; temI ++)
		{
			if(data[temI] == 0xF4 && data[temI + 1] == 0xF5)
			{
				data += temI;
				size -= temI;
				break;
			}
				
		}
		//
		if((temI == size - 1) && data[size - 1] == 0xF4)
		{
			extra[0] = 0xF4; 
			extraNcount ++;
			return 0;
		}

		if((temI == size - 1) && data[size - 1] != 0xF4)
		{
			first = 0;
			return 0; 
		}
			
	}

	if(extraNcount != 0)
	{
		if(extraNcount < 9)
		{
			if(size > 8 - extraNcount)
			{
				int length = data[8 - extraNcount];
				if(size >= (length + 11 - extraNcount))
				{
					memcpy(&extra[extraNcount],data,length + 11 - extraNcount);

					//除头除尾，获得校验码
					unsigned short res = crc16(&extra[2],length + 7, 0);

					if(res == *(unsigned short *)&extra[9 + length])
					{
						//这是一帧正确数据
						//ROS_INFO("<9");
						dataSelect(extra,11 + length);
					}
					else
					{
						ROS_INFO("<9 error");
					}
	
					data += 9 - extraNcount + length + 2;
					size -= 9 - extraNcount + length + 2;
					extraNcount = 0;
						
					int checkExtra = size;
					int temI;
					for(temI = 0; temI < checkExtra - 1; temI ++)
					{
						if(data[temI] == 0xF4 && data[temI + 1] == 0xF5)
						{
							data += temI;
							size -= temI;
							break;
						}
					}	

					if((temI == size - 1) && data[size - 1] == 0xF4)
					{
						extra[0] = 0xF4; 
						extraNcount ++;
						return 0;
					}

					if((temI == size - 1) && data[size - 1] != 0xF4)
					{
						size = 0;
						return 0; 
					}				
				}	
			}
			if(extraNcount != 0)
			{
				memcpy(&extra[extraNcount],data,size);
				extraNcount += size;
				return 0;
			}
		}
		else
		{
			int length = extra[8];
			if(size >= (length - (extraNcount - 9) + 2))
			{
				memcpy(&extra[extraNcount],data,(length - (extraNcount - 9) + 2));

				//除头除尾，获得校验码
				unsigned short res = crc16(&extra[2],length + 7, 0);

				if(res == *(unsigned short *)&extra[9 + length])
				{
					//这是一帧正确数据
					//ROS_INFO(">=9");
					dataSelect(extra,extraNcount + (length - (extraNcount - 9) + 2));
				}
				else
				{
					ROS_INFO(">=9 error");
				}
			
				data += length - (extraNcount - 9) + 2;
				size -= length - (extraNcount - 9) + 2;
				extraNcount = 0;

				int checkExtra = size;
				int temI;
				for(temI = 0; temI < checkExtra - 1; temI ++)
				{
					if(data[temI] == 0xF4 && data[temI + 1] == 0xF5)
					{
						data += temI;
						size -= temI;
						break;
					}
				}
				if((temI == size - 1) && data[size - 1] == 0xF4)
				{
					extra[0] = 0xF4; 
					extraNcount ++;
					return 0;
				}

				if((temI == size - 1) && data[size - 1] != 0xF4)
				{
					size = 0;
					return 0; 
				}
			}
			else
			{
				memcpy(&extra[extraNcount],data,size);
				extraNcount += size;
				return 0;
			}
			
		}
	}

	if(size == 0) return 0;

	if(size >= 12)
	{
		int extraSize = 0;
		int length = 0;
		do
		{
			length = data[8];

			//不足一帧数据，直接退出循环
			if(size < 11 + length)
			{
				break;
			}

			//除头除尾，获得校验码
			unsigned short res = crc16(&data[2],length + 7, 0);

			if(res == *(unsigned short *)&data[9 + length])
			{
				//这是一帧正确数据
				//ROS_INFO("111");
				dataSelect(data,9 + length + 2);
			}
			else
			{
				for(int i = 0; i < length + 11; i ++)
				{
					printf("%X ",(unsigned char)data[i]);
				}
				printf("\r\n");
				ROS_INFO("%d  %d error",size,length);
			}

			size = size - 11 - length;
			data += (11 + length);

			int checkExtra = size;
			int temI;
			for(temI = 0; temI < checkExtra - 1; temI ++)
			{
				if(data[temI] == 0xF4 && data[temI + 1] == 0xF5)
				{
					data += temI;
					size -= temI;
					break;
				}
			}
			if((temI == size - 1) && data[size - 1] == 0xF4)
			{
				extra[0] = 0xF4; 
				extraNcount ++;
				return 0;
			}

			if((temI == size - 1) && data[size - 1] != 0xF4)
			{
				size = 0;
				return 0; 
			}
			

		} while (size >= 12);
//		size = extraSize;
	}		

	memcpy(extra,data,size);
	extraNcount += size;
	return 0;
}


//接收数据分类
bool TcpClient::dataSelect(unsigned char *data,int size)
{
	int i = 0;

	static int leftLast = 0;

	if(data[6] == 0xA1)
	{
		switch (data[7])
		{
			//imu数据
			case 0x01:
			{
				sensor_msgs::Imu imuData;
				imuData.header.stamp = ros::Time::now();
				imuData.header.frame_id = imuFrame_;

				
				//线加速度
				imuData.linear_acceleration.x = (double)(*((int *)&data[9])) / 10000; 
				imuData.linear_acceleration.y = (double)(*((int *)&data[13])) / 10000;
				imuData.linear_acceleration.z = (double)(*((int *)&data[17])) / 10000;
				//角速度
				imuData.angular_velocity.x = (double)(*((int *)&data[21])) / 10000; 
				imuData.angular_velocity.y = (double)(*((int *)&data[25])) / 10000;
				imuData.angular_velocity.z = (double)(*((int *)&data[29])) / 10000;

				//四元数位姿
				imuData.orientation.x = (double)(*((int *)&data[49])) / 10000;
				imuData.orientation.y = (double)(*((int *)&data[53])) / 10000;
				imuData.orientation.z = (double)(*((int *)&data[57])) / 10000;
				imuData.orientation.w = (double)(*((int *)&data[45])) / 10000;

				//publish the message
				imu_pub_.publish(imuData);
				break;
			}
				
			//编码器数据
			case 0x02:
			{
				robuster_mr_msgs::encoderData msg;
				msg.leftTick = *((int *)&data[9]);
				msg.rightTick = *((int *)&data[13]);
				msg.leftEncoderCount = *((int *)&data[17]);
				msg.rightEncoderCount = *((int *)&data[21]);

//				ROS_INFO("last: %d  now: %d   delta: %d",leftLast,msg.leftEncoderCount,msg.leftEncoderCount - leftLast);
				leftLast = msg.leftEncoderCount;

				encoder_pub_.publish(msg);
				break;
			}
			
			//超声波数据
			case 0x03:
			{
				robuster_mr_msgs::sonarData msg;
				msg.first = *((short *)&data[9]);
				msg.second = *((short *)&data[11]);
				msg.third = *((short *)&data[13]);
				msg.fourth = *((short *)&data[15]);

				sonar_pub_.publish(msg);
				break;
			}
		}
	}

	if(data[6] == 0xA0)
	{
		switch (data[7])
		{
			//小车设备信息数据
			case 0x01:
			{
				robuster_mr_msgs::DeviceStatus msg;

				msg.errorCode = *((char *)&data[9]);
				msg.controlState = *((char *)&data[10]);
				msg.light = *((char *)&data[11]);
				msg.holdCar = *((char *)&data[12]);
				msg.powerExtra = *((char *)&data[13]);

				short temp = 0;
				*((unsigned char *)&temp) = data[15];
				*((unsigned char *)&temp + 1) = data[14];

//				msg.temperaTure = *((short *)&data[14]);

				msg.temperaTure = temp;
									
				msg.privdePower = *((char *)&data[16]);

				car_pub_.publish(msg);
				break;
			}
				
			//设备信息数据
			case 0x02:
			{	
				pthread_mutex_lock(&serverMutex_);
				pthread_cond_signal(&serverCond_);
				pthread_mutex_unlock(&serverMutex_);

				ROS_INFO("shebei result: %d",result_);
				break;
			}
					
			//心跳信息数据
			case 0x03:
			{
				for(int i = 0; i < size; i ++)
				{
//					printf("%X ",(unsigned char)data[i]);
				}
//				printf("\r\n");	
				break;
			}
			
			//回复控制指令信息数据
			case 0x04:
			{
				result_ = data[11];

				pthread_mutex_lock(&serverMutex_);
				pthread_cond_signal(&serverCond_);
				pthread_mutex_unlock(&serverMutex_);

				ROS_INFO("control result: %d",result_);
				break;
			}
				
			//回复设备编号数据
			case 0x05:
			{
				pthread_mutex_lock(&serverMutex_);
				pthread_cond_signal(&serverCond_);
				pthread_mutex_unlock(&serverMutex_);

				ROS_INFO("bianhao result: %d",result_);
				break;
			}
			
			//回复设置关闭上位机数据
			case 0x06:
				/* code */
				break;

			default :
				break;
		}
	}
}


void TcpClient::encodercallback(const robuster_mr_msgs::encoderData::ConstPtr& encoderMsg)
{
	static double x = 0.0;
	static double y = 0.0;
	static double th = 0.0;

	static double lastYaw = 0;

	static tf::TransformBroadcaster odom_broadcaster;

	static ros::Time last_time = ros::Time::now();
	ros::Time current_time;


	int leftSpeedTick = encoderMsg->leftTick;
	int rightSpeedTick = encoderMsg->rightTick;

	double leftRpm = (double)leftSpeedTick * 60.0 / 65536.0 / wheelRate_;
	double rightRpm = (double)rightSpeedTick * 60.0 / 65536.0 / wheelRate_;

//	ROS_INFO("leftRpm is %f  rightRpm is %f",leftRpm,rightRpm);

	double vx = rpm_to_liner_vel_x(leftRpm,rightRpm);
	double vth = rpm_to_liner_vel_th(leftRpm,rightRpm);

	current_time = ros::Time::now();

	//compute odometry in a typical way given the velocities of the robot
	double dt = (current_time - last_time).toSec();

//	ROS_INFO("deltaYaw: %f  %f  %f",deltaYaw, deltaYaw / dt, dt);

	double delta_x = (vx * cos(th)) * dt;
	double delta_y = (vx * sin(th)) * dt;
	double delta_th = vth * dt;

	x += delta_x;
	y += delta_y;
	th += delta_th;

	//转换为四元素
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = odomFrame_;
	odom_trans.child_frame_id = baseFrame_;

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	//是否发布tf转换
	if(tfUsed_)
		odom_broadcaster.sendTransform(odom_trans);

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = odomFrame_;

	//set the position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	//set the velocity
	odom.child_frame_id = baseFrame_;
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = 0.0;
	odom.twist.twist.angular.z = vth;

	odom.pose.covariance[0]  = 0.1;   	// x的协方差 
	odom.pose.covariance[7]  = 0.1;		// y的协方差
	odom.pose.covariance[35] = 0.2;   	//theta的协方差

	odom.pose.covariance[14] = 1e10; 	// set a non-zero covariance on unused    theta x axis
	odom.pose.covariance[21] = 1e10; 	// dimensions (z, pitch and roll); this   theta y  axis
	odom.pose.covariance[28] = 1e10; 	// is a requirement of robot_pose_ekf     theta z axis

	//publish the message
	odom_pub_.publish(odom);

	last_time = current_time;
}

//接收数据线程
void TcpClient::recvdata()
{
	ros::Rate loop(110);

	while(ros::ok())
	{
		int res = ctcpclient_.Receive(recvdata_temp_,1024,false);
		if(res <= 0)
		{
			sleep(1);
		}
		else
		{
			dataAnalysis((unsigned char *)recvdata_temp_,res);
		}
		
		for(int i = 0; i < res; i ++)
		{
//			printf("%X ",(unsigned char)recvdata_temp_[i]);
		}
		
//		printf("\r\n");
		loop.sleep();
	}
}

//发送数据线程
void TcpClient::senddata()
{
	unsigned char cmd[10] = {0};

	unsigned char heart[] = {0xf4,0xf5,0x0,0x0,0x0,0x0,0x01,0x01,0x01,0x0,0xa7,0x11};
	unsigned char control[] = {0xf4,0xf5,0x00,0x00,0x00,0x00,0x01,0x02,0x01,0x00,0xa7,0xe1};

	ros::Rate loop(30);

	ctcpclient_.Send((char *)heart,sizeof(heart));
	loop.sleep();
	ctcpclient_.Send((char *)control,sizeof(control));

	int count = 4;
	while(ros::ok())
	{
		loop.sleep();

		bool res = ctcpclient_.Send((char *)heart,sizeof(heart));
		if(res == 0)
		{
			sleep(1);
			ctcpclient_.Connect("192.168.1.196","9999");
		}

		loop.sleep();
		if(sendlenth_ != 0)
		{
			cmd_mutex_.lock();
			bool res = ctcpclient_.Send((char *)senddata_u_,sendlenth_);
			cmd_mutex_.unlock();
			for(int i = 0; i < sendlenth_; i ++)
			{
//				printf("%X ",(unsigned char)senddata_u_[i]);
			}
//			printf("\r\n");
			sendlenth_ = 0;
		}

	}
}

void TcpClient::get_abstime_wait(int microseconds, struct timespec *abstime)
{
  struct timeval tv;
  long long absmsec;
  gettimeofday(&tv, NULL);
  absmsec = tv.tv_sec * 1000ll + tv.tv_usec / 1000ll;
  absmsec += microseconds;

  abstime->tv_sec = absmsec / 1000ll;
  abstime->tv_nsec = absmsec % 1000ll * 1000000ll;
}

void TcpClient::run()
{
	//连接服务器
	bool res = ctcpclient_.Connect("192.168.1.196","9999");
	if(res)
	{
		ROS_INFO("connect success");
	}
	else
	{
		while(1)
		{
			ROS_WARN("connect failed!!!  repeat......");
			sleep(2);
			bool res = ctcpclient_.Connect("192.168.1.196","9999");
			if(res)
			{
				ROS_INFO("connect success");
				break;
			}
		}		
	}
	
	imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu_data", 5);
	sonar_pub_ = nh_.advertise<robuster_mr_msgs::sonarData>("/ultrasound",5);
	encoder_pub_ = nh_.advertise<robuster_mr_msgs::encoderData>("encoder_init_data",5);
	car_pub_ = nh_.advertise<robuster_mr_msgs::DeviceStatus>("/device_status",5);
	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 5);

	cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("smooth_cmd_vel", 5, &TcpClient::cmdcallback, this);
	encoder_init_sub_ = nh_.subscribe<robuster_mr_msgs::encoderData>("encoder_init_data", 5, &TcpClient::encodercallback, this);

	//查询设备信息
	get_devInfo_ = nh_.advertiseService("/robuster/get_devInfo", &TcpClient::get_devInfocallback,this);
	//获取控制权限
	set_permission_ = nh_.advertiseService("/robuster/set_permission", &TcpClient::set_permissioncallback,this);
	//解除防撞报警
	reset_alarm_ = nh_.advertiseService("/robuster/reset_alarm", &TcpClient::reset_alarmcallback,this);
	//开关灯
	light_control_ = nh_.advertiseService("/robuster/light_control", &TcpClient::light_controlcallback,this);
	//设置超声波开关
	enable_ultrasound_ = nh_.advertiseService("/robuster/enable_ultrasound", &TcpClient::enable_ultrasoundcallback,this);
	//IMU 复位
	reset_imu_ = nh_.advertiseService("/robuster/reset_imu", &TcpClient::reset_imucallback,this);

	//初始化信号量
	sem_init(&done_, 0, 0);

	pthread_cond_init(&serverCond_,NULL);
	pthread_mutex_init(&serverMutex_,NULL);


	//创建接收发送数据线程
	boost::thread recvdata_thread(boost::bind(&TcpClient::recvdata, this));
	boost::thread senddata_thread(boost::bind(&TcpClient::senddata, this));

	ros::spin();
}

}//空间结束




//主函数
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "robuster_mr_bringup_node");

	mr_tool::TcpClient tcpclient;
	tcpclient.run();

	return 0;
}
