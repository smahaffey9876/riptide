#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/PoseStamped.h"

class Pose
{
  private:
    ros::NodeHandle nh;
    ros::Publisher p;
    ros::Subscriber js;
    geometry_msgs::PoseStamped pose_set;

  public:
    Pose();
    void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);
    void loop();
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_set");
  Pose pose;
  pose.loop();
}

Pose::Pose()
{
  js = nh.subscribe<sensor_msgs::Joy>("joy", 1, &Pose::joy_callback, this);
  p = nh.advertise<geometry_msgs::PoseStamped>("pose_set_pt", 1);
}

void Pose::joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  ros::Time time = ros::Time::now();
  pose_set.header.stamp = time;
  pose_set.pose.position.x = 5 * joy->axes[3];
  pose_set.pose.position.y = 5 * joy->axes[2];
  pose_set.pose.position.z = 5 * (-1 * joy->axes[8] + joy->axes[9]);
  pose_set.pose.orientation.x = 2 * joy->axes[0];
  pose_set.pose.orientation.y = 2 * joy->axes[1];
  pose_set.pose.orientation.z = 2 * (joy->axes[13]-joy->axes[12]);

  p.publish(pose_set);
  
}

void Pose::loop()
{
  ros::Rate rate(30);
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}
