#include "geometry_msgs/Vector3Stamped.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
ros::init(argc,argv, "fake_data");

ros::NodeHandle n;

ros::Publisher fake = n.advertise<geometry_msgs::Vector3Stamped>("angular_set_pt",10);

ros::Rate loop_rate(1);

while (ros::ok())

{

geometry_msgs::Vector3Stamped accel_set;

  ros::Time time = ros::Time::now();
  accel_set.header.stamp = time;

accel_set.vector.x=0;
accel_set.vector.y=0;
accel_set.vector.z=0;


fake.publish(accel_set);

ros::spinOnce();

loop_rate.sleep();
}

return 0;
}

