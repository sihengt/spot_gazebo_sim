#include <gazebo_tutorials/drone_model.h>

void DroneControlPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf){
  if (!ros::isInitialized()){
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
		     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  std::string robot_namespace = sdf->Get<std::string>("robotNamespace");
  
  this->model = parent;
  this->update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&DroneControlPlugin::on_update, this));

  ros::NodeHandle nh;
  
  // init subscribers
  cmd_vel_sub = nh.subscribe(robot_namespace + "/cmd_vel", 10, &DroneControlPlugin::cmd_vel_callback, this);
  listener = new tf::TransformListener();
  state_estimate_timer = nh.createTimer(ros::Duration(1./50.), &DroneControlPlugin::state_estimate_timer_callback, this);

  // init publishers
  state_estimate_pub = nh.advertise<nav_msgs::Odometry>(robot_namespace + "/state_estimate", 10);
}

void DroneControlPlugin::on_update(){
  //this->model->GetLinks()[0]->AddForce(ignition::math::Vector3d(0.0, 0, 70.));
  this->model->SetLinearVel(ignition::math::Vector3d(linear_velocity_command.x(),
						      linear_velocity_command.y(),
						      linear_velocity_command.z()));
  this->model->SetAngularVel(ignition::math::Vector3d(angular_velocity_command.x(),
						      angular_velocity_command.y(),
						      angular_velocity_command.z()));
}

void DroneControlPlugin::cmd_vel_callback(geometry_msgs::TwistStamped twist){
  try{
    tf::StampedTransform transform;
    listener->waitForTransform("world", twist.header.frame_id, twist.header.stamp, ros::Duration(0.1));
    listener->lookupTransform("world", twist.header.frame_id, twist.header.stamp, transform);
    transform.setOrigin(tf::Vector3(0, 0, 0));

    tf::Vector3 linear_velocity_vector(twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z);
    tf::Vector3 angular_velocity_vector(0, 0, twist.twist.angular.z);
    
    linear_velocity_command = transform*linear_velocity_vector;
    angular_velocity_command = transform*angular_velocity_vector;
  }
  catch(const tf::TransformException& ex){
    ROS_ERROR_STREAM("TransformException in cmd_vel_callback: " << ex.what());
  }
}

void DroneControlPlugin::state_estimate_timer_callback(const ros::TimerEvent& e){
  nav_msgs::Odometry odom;

  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "world";
  odom.child_frame_id = "world";

  ignition::math::Pose3d pose = model->WorldPose();
  ignition::math::Vector3d linear_vel = model->WorldLinearVel();
  ignition::math::Vector3d angular_vel = model->WorldAngularVel();
  
  odom.pose.pose.position.x = pose.Pos().X();
  odom.pose.pose.position.y = pose.Pos().Y();
  odom.pose.pose.position.z = pose.Pos().Z();
  odom.pose.pose.orientation.x = pose.Rot().X();
  odom.pose.pose.orientation.y = pose.Rot().Y();
  odom.pose.pose.orientation.z = pose.Rot().Z();
  odom.pose.pose.orientation.w = pose.Rot().W();
  odom.twist.twist.linear.x = linear_vel.X();
  odom.twist.twist.linear.y = linear_vel.Y();
  odom.twist.twist.linear.z = linear_vel.Z();
  odom.twist.twist.angular.x = angular_vel.X();
  odom.twist.twist.angular.y = angular_vel.Y();
  odom.twist.twist.angular.z = angular_vel.Z();
  
  state_estimate_pub.publish(odom);
}
