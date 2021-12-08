# run_mr500

## 1. 连接mr500

### 1.1. 打开mr500电源
按电源按钮打开mr500电源，报警的时候按复位按钮

### 1.2. 连接wifi
wifi名：Terra_MR500;密码：terra123

(注意：此网不能上网，要在路由器上插网线才能上网)

### 1.3. vnc连接
在电脑上打开终端用ssh连接

    $ ssh szzq@192.168.1.102
    
密码是1

进入主控后再：$ vnc

注意：要让主控的屏幕显示，vnc命令才能成功

主控打开vnc后，再打开我电脑端或者手机端的vnc软件与mr500主控连接
192.168.1.102::12099
或者
192.168.1.102::12098

## 2. 运行

mr500的密码是：1

### 2.1. 编译
    $ cd j_ws/
    $ catkin_make

### 2.2. imu
    $ roslaunch xsens_driver xsens_driver.launch

### 2.3. 打开多线雷达

    $ roslaunch rslidar_pointcloud rs_lidar_16.launch

### 2.4. 里程计

    $ roslaunch robuster_mr_bringup mr500_bringup.launch

### 2.5. 用指令控制小车

把手柄最左侧的拨杆、拨到最上为指令控制，拨到最下为遥控手柄控制

    $ roslaunch robuster_mr_bringup mr500_bringup.launch
    $ rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.3}}'

### 2.6. 融合imu和轮式里程计

    $ roslaunch robot_pose_ekf robot_pose_ekf.launch
    $ rosrun ekf_odom ekf_odom

## 3. GPS实验
### 3.1. 打开gps
    $ rosrun nmea_navsat_driver nmea_serial_driver _port:=/dev/ttyUSB1 _baund:=4800
    
### 3.1. 户外gps实验简化运行

    $ roslaunch nmea_navsat_driver other.launch
    $ roslaunch nmea_navsat_driver demo.launch
或者
    $ cd ~/j_ws/src/run_mr500/
    $ sudo chmod 777 run.sh
    $ ./run.sh
