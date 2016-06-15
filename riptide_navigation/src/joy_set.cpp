#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>

ros::Publisher ps;
ros::Publisher ts;
ros::Subscriber js;
double posex_last=0;
double posey_last=0;
double posez_last=0;

void callback(const nav_msgs::Odometry::ConstPtr& current_data, const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::PoseStamped pose_set;
  geometry_msgs::TwistStamped twist_set;
  
  ros::Time time = ros::Time::now();
  pose_set.header.stamp = time;
  twist_set.header.stamp= time;

  pose_set.pose.position.x = .005 * (joy->axes[3]) + posex_last;
  pose_set.pose.position.y = .005 * (joy->axes[2]) + posey_last;
  pose_set.pose.position.z = .005 * (-1 * joy->axes[8] + joy->axes[10]) + posez_last;
  pose_set.pose.orientation.x = .75 * joy->axes[0];
  pose_set.pose.orientation.y = .75 * joy->axes[1];
  pose_set.pose.orientation.z = .75 * (joy->axes[13]-joy->axes[12]);

  posex_last = pose_set.pose.position.x;
  posey_last = pose_set.pose.position.y;
  posez_last = pose_set.pose.position.z;

  twist_set.twist.linear.x = .001 * (pose_set.pose.position.x-current_data->pose.pose.position.x);
  twist_set.twist.linear.y = .001 * (pose_set.pose.position.y-current_data->pose.pose.position.y);
  twist_set.twist.linear.z = .001 * (pose_set.pose.position.z-current_data->pose.pose.position.z);
  twist_set.twist.angular.x = .001 * (pose_set.pose.orientation.x-current_data->pose.pose.orientation.x);
  twist_set.twist.angular.y = .1 * (pose_set.pose.orientation.y-current_data->pose.pose.orientation.y);
  twist_set.twist.angular.z = .1 * (pose_set.pose.orientation.z-current_data->pose.pose.orientation.z);

  ps.publish(pose_set);
  ts.publish(twist_set);
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_set");
  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::Joy> joy_sub(nh, "joy", 1);
  message_filters::Subscriber<nav_msgs::Odometry> kalmanout_sub(nh, "/odometry/filtered", 1);

  ps = nh.advertise<geometry_msgs::PoseStamped>("pose_set_pt", 1);
  ts = nh.advertise<geometry_msgs::TwistStamped>("twist_set_pt",1);

 typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,sensor_msgs::Joy> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), kalmanout_sub, joy_sub);  
  sync.registerCallback(boost::bind(&callback,_1,_2));
  
  ros::spin();
}
