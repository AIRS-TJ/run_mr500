#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

  ros::Subscriber odom_sub;
  ros::Publisher odom_pub;

 void odom_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
 {
     ROS_INFO("i hear robot_pose_ekf/odom");
     ros::Time current_time = ros::Time::now();
    tf::TransformBroadcaster odom_broadcaster;

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = msg->pose.pose.position.x;
    odom_trans.transform.translation.y = msg->pose.pose.position.y;
    odom_trans.transform.translation.z = msg->pose.pose.position.z;
    odom_trans.transform.rotation = msg->pose.pose.orientation;

    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    //set the position
    odom.pose.pose.position= msg->pose.pose.position;
    odom.pose.pose.orientation = msg->pose.pose.orientation;
    odom.pose.covariance=msg->pose.covariance;
 
    //publish the message
    odom_pub.publish(odom);
 }
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "hedge_odom");
  ros::NodeHandle n;
 

  odom_pub = n.advertise<nav_msgs::Odometry>("tf_odom", 50);
  odom_sub = n.subscribe("/robot_pose_ekf/odom_combined", 1000, odom_Callback);

 
    ros::spin();     
    return 0;

}