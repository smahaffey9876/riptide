#include "ros/ros.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/AccelStamped.h"
#include <std_msgs/Float64.h>
#include "geometry_msgs/Vector3Stamped.h"
#include <sensor_msgs/Imu.h>
#include <control_toolbox/pid.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <riptide_navigation/pidConfig.h>
#include <boost/asio.hpp>
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Transform.h"
#include <imu_3dm_gx4/FilterOutput.h>
#include <tf/transform_listener.h>


double p,i,d,im,dm,g,mag_x,mag_y,mag_z;
bool first_reconfig= true;
ros::Publisher a_e;
tf::StampedTransform transform;

void dyn_callback(riptide_navigation::pidConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f %f %f %f %f %f %f %f",
            config.p, config.i,
            config.d,
            config.im,
            config.dm,
	    config.g,
            config.mag_x,
	    config.mag_y,
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
g=config.g;
mag_x=config.mag_x;
mag_y=config.mag_y;
mag_z=config.mag_z;
}

void callback(const sensor_msgs::Imu::ConstPtr& current_accel, const geometry_msgs::AccelStamped::ConstPtr& accel_set,const geometry_msgs::Vector3Stamped::ConstPtr& accel_roll,const geometry_msgs::Vector3Stamped::ConstPtr& accel_pitch,const geometry_msgs::Vector3Stamped::ConstPtr& accel_yaw,const imu_3dm_gx4::FilterOutput::ConstPtr& current_orientation)
{

  geometry_msgs::Vector3 accel_des, rpy, gravity_cal;
  geometry_msgs::Accel accel_thrusters;
  ros::Time time;
  ros::Duration time_diff;
  ros::Time last_time;
  double currentx=0;
  double currenty=0;
  double currentz=0;

  accel_des.x = accel_set->accel.linear.x;
  accel_des.y = accel_set->accel.linear.y;
  accel_des.z = accel_set->accel.linear.z;


  tf::Quaternion q(current_orientation->orientation.x,current_orientation->orientation.y,current_orientation->orientation.z,current_orientation->orientation.w);
  tf::Matrix3x3 m(q);
  double r,p,y,roll,pitch,yaw;
  m.getRPY(r,p,y);
  ROS_INFO("roll: %f pitch: %f yaw: %f",r,p,y);
  roll = r-mag_x;
  pitch = p-mag_y;
  yaw = y-mag_z;
  //tf::Vector3 gravity_w, gravity_c;
  //gravity_w = tf::Vector3(0,0,-9.80665);
  //gravity_c=transform*gravity_w;
  
  //vector3TFToMsg(gravity_c, gravity_cal);

  currentx = current_accel->linear_acceleration.x;
  currenty = current_accel->linear_acceleration.y;
  currentz = current_accel->linear_acceleration.z;
  ROS_INFO("currentx: %f currenty: %f currentz: %f",currentx,currenty,currentz);

  control_toolbox::Pid pid;

  pid.initPid(p,i,d,im,dm);

  time = ros::Time::now();
  time_diff = time-last_time;
  accel_thrusters.linear.x=pid.control_toolbox::Pid::computeCommand((accel_des.x-currentx),time_diff);

  accel_thrusters.linear.y=pid.control_toolbox::Pid::computeCommand((accel_des.y-currenty),time_diff);

  accel_thrusters.linear.z=pid.control_toolbox::Pid::computeCommand((accel_des.z-currentz),time_diff);

  pid.control_toolbox::Pid::getCurrentCmd();
  last_time = time;

  accel_thrusters.angular.x=accel_roll->vector.x;
  accel_thrusters.angular.y=accel_pitch->vector.y;
  accel_thrusters.angular.z=accel_yaw->vector.z;

  ROS_INFO("PID Accel Results -> X:%f   Y:%f   Z:%f",accel_thrusters.linear.x,accel_thrusters.linear.y,accel_thrusters.linear.z);
  
  a_e.publish(accel_thrusters);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "pid_linear");
  ros::NodeHandle nh;
  ros::NodeHandle node_priv("~");

  //node_priv.param<double>("p",p,1.0);
  //node_priv.param<double>("i",i,1.0);
  //node_priv.param<double>("d",d,1.0);
  //node_priv.param<double>("im",im,.3);
  //node_priv.param<double>("dm",dm,-.3);

  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/state/imu",1);
  message_filters::Subscriber<geometry_msgs::AccelStamped> accel_sub(nh, "accel_set_pt",1);
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> roll_sub(nh,"accel_error_roll",1);
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> pitch_sub(nh,"accel_error_pitch",1);
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> yaw_sub(nh,"accel_error_yaw",1);
  message_filters::Subscriber<imu_3dm_gx4::FilterOutput> imuang (nh, "/state/filter",1);

  dynamic_reconfigure::Server<riptide_navigation::pidConfig> server;
  dynamic_reconfigure::Server<riptide_navigation::pidConfig>::CallbackType f;
  f = boost::bind(&dyn_callback, _1, _2);
  server.setCallback(f);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, geometry_msgs::AccelStamped,geometry_msgs::Vector3Stamped,geometry_msgs::Vector3Stamped,geometry_msgs::Vector3Stamped,imu_3dm_gx4::FilterOutput> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), imu_sub, accel_sub, roll_sub, pitch_sub, yaw_sub, imuang);  
    sync.registerCallback(boost::bind(&callback,_1,_2,_3,_4,_5,_6));

    a_e = nh.advertise<geometry_msgs::Accel>("accel_error", 1);
  
 // tf::TransformListener listener;  
 
 // ros::Rate rate(20.0);
 //while (nh.ok()){
 // try{
 // listener.lookupTransform("/map","/base_link",ros::Time(0),transform);
 // ROS_INFO("Listener is Working");
 // }

 // catch (tf::TransformException &ex){
 // ROS_ERROR ("%s", ex.what());
 // }
 
 ros::spin();
 //rate.sleep();

}
