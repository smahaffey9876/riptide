#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <control_toolbox/pid.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <riptide_navigation/pidConfig.h>
#include <boost/asio.hpp>
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Transform.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>

double px,ix,dx,imx,dmx,py,iy,dy,imy,dmy,pz,iz,dz,imz,dmz,pr,ir,dr,imr,dmr,pp,ip,dp,imp,dmp,pyaw,iyaw,dyaw,imyaw,dmyaw;
bool first_reconfig= true;
ros::Publisher p_e;
tf::StampedTransform transform;

void dyn_callback(riptide_navigation::pidConfig &config, uint32_t level) {

if (first_reconfig)
	{
	first_reconfig=false;
	return;
	}
px=config.px;
ix=config.ix;
dx=config.dx;
imx=config.imx;
dmx=config.dmx;
py=config.py;
iy=config.iy;
dy=config.dy;
imy=config.imy;
dmy=config.dmy;
pz=config.pz;
iz=config.iz;
dz=config.dz;
imz=config.imz;
dmz=config.dmz;
pr=config.pr;
ir=config.ir;
dr=config.dr;
imr=config.imr;
dmr=config.dmr;
pp=config.pp;
ip=config.ip;
dp=config.dp;
imp=config.imp;
dmp=config.dmp;
pyaw=config.pyaw;
iyaw=config.iyaw;
dyaw=config.dyaw;
imyaw=config.imyaw;
dmyaw=config.dmyaw;


}

void callback(const nav_msgs::Odometry::ConstPtr& current_pose, const geometry_msgs::PoseStamped::ConstPtr& pose_set)
{

  geometry_msgs::Vector3 position_des, position_current, orientation_des, orientation_current;
  geometry_msgs::PoseStamped pose_error;
  ros::Time time;
  ros::Duration time_diff;
  ros::Time last_time;
  
  position_des.x = pose_set->pose.position.x;
  position_des.y = pose_set->pose.position.y;
  position_des.z = pose_set->pose.position.z;
  ROS_INFO("set: %f",position_des.x);
  position_current.x = current_pose->pose.pose.position.x;
  position_current.y = current_pose->pose.pose.position.y;
  position_current.z = current_pose->pose.pose.position.z;
  ROS_INFO("current: %f",position_current.x);
  control_toolbox::Pid pidx;
//, pidy, pidz, pidr, pidp, pidyaw;

  pidx.initPid(px,ix,dx,imx,dmx);
  //pidy.initPid(py,iy,dy,imy,dmy);
  //pidz.initPid(pz,iz,dz,imz,dmz);
  //pidr.initPid(pr,ir,dr,imr,dmr);
  //pidp.initPid(pp,ip,dp,imp,dmp);
  //pidy.initPid(pyaw,iyaw,dyaw,imyaw,dmyaw);

  time = ros::Time::now();
  time_diff = time-last_time;
  pose_error.header.stamp = time;
  
  pose_error.pose.position.x=pidx.control_toolbox::Pid::computeCommand((position_des.x-position_current.x),time_diff);
  //pose_error.pose.position.y=pidy.control_toolbox::Pid::computeCommand((pose_set->pose.position.y-current_pose->pose.pose.position.y),time_diff);
  //pose_error.pose.position.z=pidz.control_toolbox::Pid::computeCommand((pose_set->pose.position.z-current_pose->pose.pose.position.z),time_diff);
ROS_INFO("pid: %f",pose_error.pose.position.x);
  //pose_error.pose.orientation.x=pidr.control_toolbox::Pid::computeCommand((pose_set->pose.position.x-current_pose->pose.pose.orientation.x),time_diff);
  //pose_error.pose.orientation.y=pidp.control_toolbox::Pid::computeCommand((pose_set->pose.position.y-current_pose->pose.pose.orientation.y),time_diff);
  //pose_error.pose.orientation.z=pidyaw.control_toolbox::Pid::computeCommand((pose_set->pose.position.z-current_pose->pose.pose.orientation.z),time_diff);
  
  pidx.control_toolbox::Pid::getCurrentCmd();
  //pidy.control_toolbox::Pid::getCurrentCmd();
  //pidz.control_toolbox::Pid::getCurrentCmd();
  //pidr.control_toolbox::Pid::getCurrentCmd();
  //pidp.control_toolbox::Pid::getCurrentCmd();
  //pidyaw.control_toolbox::Pid::getCurrentCmd();

  last_time = time;

  p_e.publish(pose_error);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "position_controller");
  ros::NodeHandle nh;
  ros::NodeHandle node_priv("~");

  message_filters::Subscriber<nav_msgs::Odometry> kalmanout_sub(nh, "/odometry/filtered",1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> poseset_sub(nh, "pose_set_pt",1);
 
  dynamic_reconfigure::Server<riptide_navigation::pidConfig> server;
  dynamic_reconfigure::Server<riptide_navigation::pidConfig>::CallbackType f;
  f = boost::bind(&dyn_callback, _1, _2);
  server.setCallback(f);

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,geometry_msgs::PoseStamped> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), kalmanout_sub, poseset_sub);  
  sync.registerCallback(boost::bind(&callback,_1,_2));

  p_e = nh.advertise<geometry_msgs::PoseStamped>("pose_error", 1);
  tf::TransformListener listener;  
 
  ros::Rate rate(20.0);
  
  while (nh.ok()){
  try{
  listener.lookupTransform("/map","/base_link",ros::Time(0),transform);
  }
  catch (tf::TransformException &ex){
  ROS_ERROR ("%s", ex.what());
  }
  ros::spin();
  rate.sleep();
  }
}
