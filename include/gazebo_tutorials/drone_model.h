#ifndef _DRONE_MODEL_H_
#define _DRONE_MODEL_H_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class DroneControlPlugin : public gazebo::ModelPlugin {
private:
  gazebo::physics::ModelPtr model;
  gazebo::event::ConnectionPtr update_connection;

  tf::Vector3 linear_velocity_command;
  tf::Vector3 angular_velocity_command;

  // subscribers
  ros::Subscriber cmd_vel_sub;
  tf::TransformListener* listener;
  ros::Timer state_estimate_timer;

  // publishers
  ros::Publisher state_estimate_pub;
  
  // callbacks
  void cmd_vel_callback(geometry_msgs::TwistStamped twist);
  void state_estimate_timer_callback(const ros::TimerEvent& e);
  
public:
  void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

  void on_update();
};

GZ_REGISTER_MODEL_PLUGIN(DroneControlPlugin)

#endif
