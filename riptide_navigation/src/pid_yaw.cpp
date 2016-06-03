#include "ros/ros.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/AccelStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Vector3.h"
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <control_toolbox/pid.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <riptide_navigation/pidConfig.h>
#include <boost/asio.hpp>
#include <imu_3dm_gx4/FilterOutput.h>
#include "tf/transform_datatypes.h"

double p,i,d,im,dm, mag_z;
bool first_reconfig= true;
ros::Publisher a_e;

void dyn_callback(riptide_navigation::pidConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f %f %f %f %f",
            config.p, config.i,
            config.d,
            config.im,
            config.dm,
	    config.mag_z);
if (first_reconfig)
	{
	first_reconfig=false;
	return;
	}
p=config.p;
i=config.i;
d=config.d;
im=config.im;
dm=config.dm;
mag_z=config.mag_z;
}

void callback(const imu_3dm_gx4::FilterOutput::ConstPtr& current_orientation, const geometry_msgs::Vector3Stamped::ConstPtr& orientation_set)
{

  ros::Time time;
  ros::Duration time_diff;
  ros::Time last_time;
  geometry_msgs::Vector3 orientation_des, rpy; 
  geometry_msgs::Vector3Stamped accel_yaw;
  double currentz;

  ros::Time timestamp = ros::Time::now();
  accel_yaw.header.stamp = timestamp;

  orientation_des.z = orientation_set->vector.z;


  tf::Quaternion q(current_orientation->orientation.x,current_orientation->orientation.y,current_orientation->orientation.z,current_orientation->orientation.w);
  tf::Matrix3x3 m(q);
  double roll,pitch,yaw;
  
  m.getRPY(roll,pitch,yaw);
  ROS_INFO("roll: %f pitch: %f yaw: %f",roll,pitch,yaw);
  currentz = yaw-mag_z;

  control_toolbox::Pid pid;

  pid.initPid(p,i,d,im,dm);

  time = ros::Time::now();
  time_diff = time-last_time;
  accel_yaw.vector.z=pid.control_toolbox::Pid::computeCommand((orientation_des.z-currentz),time_diff);

  pid.control_toolbox::Pid::getCurrentCmd();
  last_time = time;

  ROS_INFO("PID Accel Results -> %f",accel_yaw.vector.z);

a_e.publish(accel_yaw);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "pid_yaw");
  ros::NodeHandle nh;
  ros::NodeHandle node_priv("~");

  node_priv.param<double>("p",p,1.0);
  node_priv.param<double>("i",i,1.0);
  node_priv.param<double>("d",d,1.0);
  node_priv.param<double>("im",im,.3);
  node_priv.param<double>("dm",dm,-.3);

  message_filters::Subscriber<imu_3dm_gx4::FilterOutput> imu_sub(nh, "/state/filter",1);
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> accel_sub(nh, "angular_set_pt",1);

  dynamic_reconfigure::Server<riptide_navigation::pidConfig> server;
  dynamic_reconfigure::Server<riptide_navigation::pidConfig>::CallbackType f;
  f = boost::bind(&dyn_callback, _1, _2);
  server.setCallback(f);


  typedef message_filters::sync_policies::ApproximateTime<imu_3dm_gx4::FilterOutput, geometry_msgs::Vector3Stamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), imu_sub, accel_sub);  
    sync.registerCallback(boost::bind(&callback,_1,_2));

    a_e = nh.advertise<geometry_msgs::Vector3Stamped>("accel_error_yaw", 1);
  ros::spin();

}
