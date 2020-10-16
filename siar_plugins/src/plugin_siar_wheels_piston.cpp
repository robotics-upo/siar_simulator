#include <algorithm>
#include <assert.h>

#include <siar_plugins/plugin_siar_wheels_piston.h>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <gazebo/gazebo_config.h>
#include "siar_driver/SiarStatus.h"

namespace gazebo {

  enum {
    RIGHT,
    LEFT,
  };
  
  GazeboRosWheelsPiston::GazeboRosWheelsPiston() {
  }
 
  // Destructor
  GazeboRosWheelsPiston::~GazeboRosWheelsPiston() {
    delete rosnode_;
    delete transform_broadcaster_;
    delete inter_va;
    delete inter_vr;
    
  }

  // Load the controller
  void GazeboRosWheelsPiston::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

    this->parent = _parent;
    this->world = _parent->GetWorld();

    this->robot_namespace_ = "";
    if (!_sdf->HasElement("robotNamespace")) {
      ROS_INFO("GazeboRosWheelsPiston Plugin missing <robotNamespace>, defaults to \"%s\"", 
          this->robot_namespace_.c_str());
    } else {
      this->robot_namespace_ = 
        _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }

    //this->left_joint_names_ = "left_joint";
    if (!_sdf->HasElement("leftJoints")) {
      gzthrow("Have to specify space separated left side joint names via <leftJoints> tag!");
    } else {
      std::string joint_string = _sdf->GetElement("leftJoints")->Get<std::string>();
      boost::split( joint_names_[LEFT], joint_string, boost::is_any_of(" ") );
    }

    //this->right_joint_names_ = "right_joint";
    if (!_sdf->HasElement("rightJoints")) {
      gzthrow("Have to specify space separated right side joint names via <rightJoints> tag!");
    } else {
      std::string joint_string = _sdf->GetElement("rightJoints")->Get<std::string>();
      boost::split( joint_names_[RIGHT], joint_string, boost::is_any_of(" ") );
    }

    this->init_wheel_separation_ = 0.62;
    if (!_sdf->HasElement("initWheelSeparation")) {
      ROS_WARN("GazeboRosWheelsPiston Plugin (ns = %s) missing <initWheelSeparation>, defaults to %f",
          this->robot_namespace_.c_str(), this->init_wheel_separation_);
    } else {
      this->init_wheel_separation_ = 
        _sdf->GetElement("initWheelSeparation")->Get<double>();
    }

    this->wheel_diameter_ = 0.25;
    if (!_sdf->HasElement("wheelDiameter")) {
      ROS_WARN("GazeboRosWheelsPiston Plugin (ns = %s) missing <wheelDiameter>, defaults to %f",
          this->robot_namespace_.c_str(), this->wheel_diameter_);
    } else {
      this->wheel_diameter_ = _sdf->GetElement("wheelDiameter")->Get<double>();
    }

    this->torque = 15.0;
    if (!_sdf->HasElement("torque")) {
      ROS_WARN("GazeboRosWheelsPiston Plugin (ns = %s) missing <torque>, defaults to %f",
          this->robot_namespace_.c_str(), this->torque);
    } else {
      this->torque = _sdf->GetElement("torque")->Get<double>();
    }

    this->command_topic_ = "cmd_vel";
    if (!_sdf->HasElement("commandTopic")) {
      ROS_WARN("GazeboRosWheelsPiston Plugin (ns = %s) missing <commandTopic>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->command_topic_.c_str());
    } else {
      this->command_topic_ = _sdf->GetElement("commandTopic")->Get<std::string>();
    }

    this->odometry_topic_ = "odom";
    if (!_sdf->HasElement("odometryTopic")) {
      ROS_WARN("GazeboRosWheelsPiston Plugin (ns = %s) missing <odometryTopic>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->odometry_topic_.c_str());
    } else {
      this->odometry_topic_ = _sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    this->odometry_frame_ = "odom";
    if (!_sdf->HasElement("odometryFrame")) {
      ROS_WARN("GazeboRosWheelsPiston Plugin (ns = %s) missing <odometryFrame>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->odometry_frame_.c_str());
    } else {
      this->odometry_frame_ = _sdf->GetElement("odometryFrame")->Get<std::string>();
    }

    this->robot_base_frame_ = "base_footprint";
    if (!_sdf->HasElement("robotBaseFrame")) {
      ROS_WARN("GazeboRosWheelsPiston Plugin (ns = %s) missing <robotBaseFrame>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->robot_base_frame_.c_str());
    } else {
      this->robot_base_frame_ = _sdf->GetElement("robotBaseFrame")->Get<std::string>();
    }

    this->update_rate_ = 100.0;
    if (!_sdf->HasElement("updateRate")) {
      ROS_WARN("GazeboRosWheelsPiston Plugin (ns = %s) missing <updateRate>, defaults to %f",
          this->robot_namespace_.c_str(), this->update_rate_);
    } else {
      this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
    }


    this->publish_odometry_tf_ = true;
    if (!_sdf->HasElement("publishOdometryTf")) {
      ROS_WARN("GazeboRosWheelsPiston Plugin (ns = %s) missing <publishOdometryTf>, defaults to %s",
               this->robot_namespace_.c_str(), this->publish_odometry_tf_ ? "true" : "false");
    } else {
      this->publish_odometry_tf_ = _sdf->GetElement("publishOdometryTf")->Get<bool>();
    }

    this->publish_odometry_msg_ = true;
    if (!_sdf->HasElement("publishOdometryMsg")) {
      ROS_WARN("GazeboRosWheelsPiston Plugin (ns = %s) missing <publishOdometryMsg>, defaults to %s",
               this->robot_namespace_.c_str(), this->publish_odometry_msg_ ? "true" : "false");
    } else {
      this->publish_odometry_msg_ = _sdf->GetElement("publishOdometryMsg")->Get<bool>();
    }

    

    // Initialize update rate stuff
    if (this->update_rate_ > 0.0) {
      this->update_period_ = 1.0 / this->update_rate_;
    } else {
      this->update_period_ = 0.0;
    }
    last_update_time_ = this->world->SimTime();

    // Initialize velocity stuff
    wheel_speed_[RIGHT] = 0;
    wheel_speed_[LEFT] = 0;

    x_ = 0;
    rot_ = 0;
    alive_ = true;

    for (size_t side = 0; side < 2; ++side){
      for (size_t i = 0; i < joint_names_[side].size(); ++i){
        joints_[side].push_back(this->parent->GetJoint(joint_names_[side][i]));
        if (!joints_[side][i]){
          char error[200];
          snprintf(error, 200,
                   "GazeboRosWheelsPiston Plugin (ns = %s) couldn't get hinge joint named \"%s\"",
                   this->robot_namespace_.c_str(), joint_names_[side][i].c_str());
          gzthrow(error);
        }
#if (GAZEBO_MAJOR_VERSION > 4)
        joints_[side][i]->SetEffortLimit(0, torque);
#else
        joints_[side][i]->SetMaxForce(0, torque);
#endif
      }
    }


    // Get the SIAR relevant links & joint
    l_c_wheel_ = this->parent->GetLink("wheel_left_1");
    r_c_wheel_ = this->parent->GetLink("wheel_right_1");
    electronic_box_ = this->parent->GetLink("box_battery");
    
    tf_frame_name_.push_back("asusXtion_topMiddle_link");
    tf_frame_name_.push_back("asusXtion_frontMiddle_link");
    tf_frame_name_.push_back("asusXtion_frontRight_link");
    tf_frame_name_.push_back("asusXtion_frontLeft_link");
    tf_frame_name_.push_back("asusXtion_backMiddle_link");
    tf_frame_name_.push_back("asusXtion_backRight_link");
    tf_frame_name_.push_back("asusXtion_backLeft_link");
    
    for (const std::string& name : tf_frame_name_) { // access by const reference
    
      frame_camera_.push_back(this->parent->GetLink(name));
    }
    
    this -> piston_main_1_ = this->parent->GetJoint("move_piston_1_1");
    this -> piston_main_2_ = this->parent->GetJoint("move_piston_1_2");
    this -> hinge_arm_right_1_1_ = this->parent->GetJoint("hinge_arm_right_1_1");
    this -> hinge_arm_right_1_2_ = this->parent->GetJoint("hinge_arm_right_1_2");
    this -> hinge_arm_left_1_1_ = this->parent->GetJoint("hinge_arm_left_1_1");
    this -> hinge_arm_left_1_2_ = this->parent->GetJoint("hinge_arm_left_1_2");
    this -> axis_wheel_right_1_ = this->parent->GetJoint("move_axis_wheel_right_1");
    this -> axis_wheel_right_2_ = this->parent->GetJoint("move_axis_wheel_right_2");
    this -> axis_wheel_right_3_ = this->parent->GetJoint("move_axis_wheel_right_3");
    this -> axis_wheel_left_1_ = this->parent->GetJoint("move_axis_wheel_left_1");
    this -> axis_wheel_left_2_ = this->parent->GetJoint("move_axis_wheel_left_2");
    this -> axis_wheel_left_3_ = this->parent->GetJoint("move_axis_wheel_left_3");
    
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    rosnode_ = new ros::NodeHandle(this->robot_namespace_);
    
    // Load parameter to calculate interpolation
    
    if (!rosnode_->getParam("cmd_vel_file", cmd_vel_file)) {
      cmd_vel_file = "cmd_vel_file";
    }
    if (!rosnode_->getParam("vr_file", vr_file)) {
      vr_file = "vr_file";
    }
    if (!rosnode_->getParam("va_file", va_file)) {
      va_file = "va_file";
   }
    inter_vr = new functions::LinearInterpolator(cmd_vel_file, vr_file);
    inter_va = new functions::LinearInterpolator(cmd_vel_file, va_file);
    
    

    //ROS_INFO("Starting GazeboRosWheelsPiston Plugin (ns = %s). Cmd_file= %s. iNTERPOL: %d!", this->robot_namespace_.c_str(), cmd_vel_file.c_str(), (int)inter_va->size());

    tf_prefix_ = tf::getPrefixParam(*rosnode_);
    transform_broadcaster_ = new tf::TransformBroadcaster();

    
    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
   ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
          boost::bind(&GazeboRosWheelsPiston::cmdVelCallback, this, _1),
          ros::VoidPtr(), &queue_);
    cmd_vel_subscriber_ = rosnode_->subscribe(so);
    
        // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
          so = ros::SubscribeOptions::create<std_msgs::Float32>("vel_state", 1,
          boost::bind(&GazeboRosWheelsPiston::velStateCallback, this, _1),
          ros::VoidPtr(), &queue_);
    vel_state_subscriber_ = rosnode_->subscribe(so);

    // SIAR: create elec pos subscriber
          so = ros::SubscribeOptions::create<std_msgs::Float32>("width_pos", 1,
          boost::bind(&GazeboRosWheelsPiston::elecPosCallback, this, _1),
          ros::VoidPtr(), &queue_);
    move_Piston_subscriber_= rosnode_->subscribe(so);
    
    // SIAR: create arm_bool subscriber
          so = ros::SubscribeOptions::create<std_msgs::Bool>("arm_bool", 1,
          boost::bind(&GazeboRosWheelsPiston::armCentralPosCallback, this, _1),
          ros::VoidPtr(), &queue_);
    arm_central_subscriber_= rosnode_->subscribe(so);
       

    // INitialize publishers
    odometry_publisher_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
    width_publisher_ = rosnode_->advertise<std_msgs::Float32>("width", 1);
    pos_electronicBox_publisher_ = rosnode_->advertise<geometry_msgs::Vector3>("pos_ElectronicBox", 1);
    pos_centerMidWheels_publisher_ = rosnode_->advertise<geometry_msgs::Vector3>("pos_centerMidWheels_", 1);
    pos_vecBoxWheel_publisher_ = rosnode_->advertise<geometry_msgs::Vector3>("pos_vecBoxWheel_", 1);
    pos_vecUnitOrient_publisher_ = rosnode_->advertise<geometry_msgs::Vector3>("pos_vecUnitOrient_", 1);
    dis_box_centralaxis_publisher_= rosnode_->advertise<std_msgs::Float32>("dis_box_centralaxis_", 1);
    elec_pos_publisher_= rosnode_->advertise<std_msgs::Float32>("elec_pos", 1);
    siar_status_publisher_= rosnode_->advertise<siar_driver::SiarStatus>("siar_status",1);
    tf_base_link_publisher_= rosnode_->advertise<geometry_msgs::Vector3>("tf_base_link",1);
    

    // start custom queue for diff drive
    this->callback_queue_thread_ = 
      boost::thread(boost::bind(&GazeboRosWheelsPiston::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ = 
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosWheelsPiston::UpdateChild, this));

    // Initial position electronic
    move_Piston_cmd_=1;
    move_Piston_aux_=1;
    elec_pos_cmd_ = 0;
    arm_central_cmd_=false;
    vel_state_cmd_=0;
  }

  
  // Update the controller
  void GazeboRosWheelsPiston::UpdateChild() {
    common::Time current_time = this->world->SimTime();
    double seconds_since_last_update = 
      (current_time - last_update_time_).Double();
    if (seconds_since_last_update > update_period_) {

      if (this->publish_odometry_tf_ || this->publish_odometry_msg_){
        publishOdometry(seconds_since_last_update);
      }

      //Publish the SiarStatus
      
      siar_driver::SiarStatus msg;
      msg.width = width_;
      msg.electronics_x = (-1*elec_pos_cmd_);
      siar_status_publisher_.publish(msg);
      
      //Calculate Value of distance between Wheels
      updateWidth();
      std_msgs::Float32 width_msg;
      width_msg.data = width_;
      width_publisher_.publish(width_msg);

      //Obtain Value of the distance between Center Robot and Electronic Box
      updateElecPos();
      
      tfBaseLink();
     
      this->pid_hinge_arm_right_left = common::PID(100, 5.0, 5.0);
      this-> parent ->GetJointController()->SetPositionPID(this->hinge_arm_right_1_1_->GetScopedName(), this->pid_hinge_arm_right_left);
      this-> parent ->GetJointController()->SetPositionPID(this->hinge_arm_right_1_2_->GetScopedName(), this->pid_hinge_arm_right_left);
      this-> parent ->GetJointController()->SetPositionPID(this->hinge_arm_left_1_1_->GetScopedName(), this->pid_hinge_arm_right_left);
      this-> parent ->GetJointController()->SetPositionPID(this->hinge_arm_left_1_2_->GetScopedName(), this->pid_hinge_arm_right_left);
      this-> parent ->GetJointController()->SetPositionTarget(this->hinge_arm_right_1_1_->GetScopedName(), -1.2* elec_pos_cmd_);
      this-> parent ->GetJointController()->SetPositionTarget(this->hinge_arm_right_1_2_->GetScopedName(), -1.2* elec_pos_cmd_);
      this-> parent ->GetJointController()->SetPositionTarget(this->hinge_arm_left_1_1_->GetScopedName(),  1.2* elec_pos_cmd_);
      this-> parent ->GetJointController()->SetPositionTarget(this->hinge_arm_left_1_2_->GetScopedName(),  1.2* elec_pos_cmd_);              
      
      // Here to limit the first option move_Piston_cmd_ = 1 like value, because it is given problem like initial value
        if ( (move_Piston_cmd_ == 0)  || move_Piston_aux_ == 0)  
	  {
	  elec_pos_cmd_ = move_Piston_cmd_ ;
	  move_Piston_aux_= 0;
	  }
 
      //Publish the position of electronic_box_
      std_msgs::Float32 elec_pos_msg;
      elec_pos_msg.data = (-1*elec_pos_cmd_);
      elec_pos_publisher_.publish(elec_pos_msg);
        
        
      // Update robot in case new velocities have been requested
      getWheelVelocities();

        for (size_t side = 0; side < 2; ++side){
            for (size_t i = 0; i < joints_[side].size(); ++i){
            joints_[side][i]->SetVelocity(0, wheel_speed_[side] / (0.5 * wheel_diameter_));
		    
            }
        }
     
      last_update_time_+= common::Time(update_period_);
    }
  }

  // Finalize the controller
  void GazeboRosWheelsPiston::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void GazeboRosWheelsPiston::getWheelVelocities() {
    boost::mutex::scoped_lock scoped_lock(lock);
  
    double coef_vr,coef_va;
      
    //Values interpolation to have same proportion of odomTopic and cmd_vel   
    coef_vr = inter_vr->interpolate(fabs(x_));
    coef_va = inter_va->interpolate(fabs(rot_));
    
    double vr = x_ * (coef_vr);
    double va = rot_ * (coef_va);
   
    wheel_speed_[LEFT] = 4.0*(2.0*vr - va * width_ / (2.0*0.125));
    wheel_speed_[RIGHT] = 4.0*(2.0*vr + va * width_ / (2.0*0.125));
  }

  void GazeboRosWheelsPiston::updateWidth(){
      
     ignition::math::Vector3d dis = l_c_wheel_->WorldCoGPose().Pos() - r_c_wheel_->WorldCoGPose().Pos();
     width_ = sqrt(((l_c_wheel_->WorldCoGPose().Pos().X() - r_c_wheel_->WorldCoGPose().Pos().X()) * (l_c_wheel_->WorldCoGPose().Pos().X() - r_c_wheel_->WorldCoGPose().Pos().X())) + ((l_c_wheel_->WorldCoGPose().Pos().Y() - r_c_wheel_->WorldCoGPose().Pos().Y()) * (l_c_wheel_->WorldCoGPose().Pos().Y() - r_c_wheel_->WorldCoGPose().Pos().Y()))) + 0.10001;

  }

  void GazeboRosWheelsPiston::updateElecPos(){
     ignition::math::Vector3d rl, rr,u,pe;
// convert velocity to child_frame_id (aka base_footprint)
    ignition::math::Pose3d pose = this->parent->WorldPose();
    float yaw = pose.Rot().Yaw();

     rb.X() = electronic_box_->WorldCoGPose().Pos().X();
     rb.Y() = electronic_box_->WorldCoGPose().Pos().Y();
     rb.Z() = electronic_box_->WorldCoGPose().Pos().Z();
     rl.X() = l_c_wheel_->WorldCoGPose().Pos().X();
     rl.Y() = l_c_wheel_->WorldCoGPose().Pos().Y();
     rl.Z() = l_c_wheel_->WorldCoGPose().Pos().Z() - 0.125;
     rr.X() = r_c_wheel_->WorldCoGPose().Pos().X();
     rr.Y() = r_c_wheel_->WorldCoGPose().Pos().Y();
     rr.Z() = r_c_wheel_->WorldCoGPose().Pos().Z() - 0.125;
     rm = (rl + rr) * 0.5;
     pe= rb - rm;
     u.X()= cos (yaw);
     u.Y()= sin (yaw);
     u.Z()= 0;

// HERE IS NECESARY TO CHECK HOW TO PRESENT THE FUNCTION DOT PRODUCT TO APPLY
     dis_box_centralaxis_ = (pe.X()*u.X()+pe.Y()*u.Y()+pe.Z()*u.Z())*10;
     
     //Get the position of Electronic Box
     geometry_msgs::Vector3 pos_electronicBox_msg;
     pos_electronicBox_msg.x = electronic_box_->WorldCoGPose().Pos().X();
     pos_electronicBox_msg.y = electronic_box_->WorldCoGPose().Pos().Y();
     pos_electronicBox_msg.z = 0;
     pos_electronicBox_publisher_.publish(pos_electronicBox_msg);
     
      
     //Get the Vector Central  Between Middle Wheels
     geometry_msgs::Vector3 pos_centerMidWheels_msg;
     pos_centerMidWheels_msg.x = rm.X();
     pos_centerMidWheels_msg.y = rm.Y();
     pos_centerMidWheels_msg.z = rm.Z();
     pos_centerMidWheels_publisher_.publish(pos_centerMidWheels_msg);
      
      
     //Get Vector Diference XY between center Electronic Box and Centor Central Middle Wheels
     geometry_msgs::Vector3 pos_vecBoxWheel_msg;
     pos_vecBoxWheel_msg.x = pe.X();
     pos_vecBoxWheel_msg.y = pe.Y();
     pos_vecBoxWheel_msg.z = pe.Z();
     pos_vecBoxWheel_publisher_.publish(pos_vecBoxWheel_msg);
     
     
     //Get unit Vector parrallel with orientation of Robot
     geometry_msgs::Vector3 pos_vecUnitOrient_msg;
     pos_vecUnitOrient_msg.x = u.X();
     pos_vecUnitOrient_msg.y = u.Y();
     pos_vecUnitOrient_msg.z = u.Z();
     pos_vecUnitOrient_publisher_.publish(pos_vecUnitOrient_msg);

     
     //Get Dot Product between Vector unit orientation and Vector Diference XY
     std_msgs::Float32 dis_box_centralaxis_msg;  
     dis_box_centralaxis_msg.data = (dis_box_centralaxis_);
     dis_box_centralaxis_publisher_.publish(dis_box_centralaxis_msg);
     
  }

  void GazeboRosWheelsPiston::cmdVelCallback(
      const geometry_msgs::Twist::ConstPtr& cmd_msg) {

    boost::mutex::scoped_lock scoped_lock(lock);
    x_ = cmd_msg->linear.x;
    rot_ = cmd_msg->angular.z;
    
    
  }

  void GazeboRosWheelsPiston::elecPosCallback (
      const std_msgs::Float32::ConstPtr& move_Piston_msg) {
    
    boost::mutex::scoped_lock scoped_lock(lock);
    move_Piston_cmd_ = move_Piston_msg->data;
   }
 
  void GazeboRosWheelsPiston::armCentralPosCallback (
      const std_msgs::Bool::ConstPtr& arm_central_msg) {
    
    boost::mutex::scoped_lock scoped_lock(lock);
    arm_central_cmd_ = arm_central_msg->data;
   }        
   
  void GazeboRosWheelsPiston::velStateCallback (
      const std_msgs::Float32::ConstPtr& vel_state_msg) {
    
    boost::mutex::scoped_lock scoped_lock(lock);
    vel_state_cmd_ = vel_state_msg->data;
   }  
  void GazeboRosWheelsPiston::QueueThread() {
    static const double timeout = 0.01;

    while (alive_ && rosnode_->ok()) {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  
  void GazeboRosWheelsPiston::tfBaseLink(void)
  {
    static tf::TransformBroadcaster br;
    
//     math::Vector3 cam_[frame_camera_.size()];
    ignition::math::Pose3d cam_[frame_camera_.size()];
    ignition::math::Pose3d tf_electronic_box;
    
    std::string frame_name_8 = "base_link",
		frame_name_9 = "box_electronics";
    
    for (unsigned int i = 0; i < frame_camera_.size() ; i++)
    {
      
      cam_[i] = frame_camera_[i] ->WorldCoGPose();
    }
	
    tf::Transform t_[frame_camera_.size()];
    //tf::Quaternion q_[frame_camera_.size()];
    
    for (int i = 0 ; i < tf_frame_name_.size() ; i++)
    {   
      t_[i].setOrigin( tf::Vector3(cam_[i].Pos().X(), cam_[i].Pos().Y(), cam_[i].Pos().Z()) );
      t_[i].setRotation(tf::Quaternion( cam_[i].Rot().X(), cam_[i].Rot().Y(), cam_[i].Rot().Z(),cam_[i].Rot().W()) );
      //q_[i].setRPY(0, 0, 0);
      //t_[i].setRotation(q_[i]);
      br.sendTransform(tf::StampedTransform(t_[i], ros::Time::now(), "world", tf_frame_name_[i]) ); 
    }
    
      tf_electronic_box = electronic_box_->WorldCoGPose();
     
    
      tf::Transform t_8,t_9;
      
      t_8.setOrigin( tf::Vector3(rm.X(), rm.Y(), rm.Z()) );
      t_9.setOrigin( tf::Vector3(tf_electronic_box.Pos().X(), tf_electronic_box.Pos().Y(), tf_electronic_box.Pos().Z()) );
            
      tf::Quaternion q_8;

      t_8.setRotation(tf::Quaternion(tf_electronic_box.Rot().X(), tf_electronic_box.Rot().Y(),tf_electronic_box.Rot().Z(),tf_electronic_box.Rot().W() ));
      t_9.setRotation (tf::Quaternion(tf_electronic_box.Rot().X(), tf_electronic_box.Rot().Y(),tf_electronic_box.Rot().Z(),tf_electronic_box.Rot().W() ));
      
      br.sendTransform(tf::StampedTransform(t_8, ros::Time::now(), "world", frame_name_8 ));
      br.sendTransform(tf::StampedTransform(t_9, ros::Time::now(), "world", frame_name_9 ));
      
  }
  
  
  void GazeboRosWheelsPiston::publishOdometry(double step_time) {
    ros::Time current_time = ros::Time::now();
    std::string odom_frame = 
      tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_footprint_frame = 
      tf::resolve(tf_prefix_, robot_base_frame_);

    // getting data for base_footprint to odom transform
    ignition::math::Pose3d pose = this->parent->WorldPose();

    tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
    tf::Vector3 vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

    tf::Transform base_footprint_to_odom(qt, vt);

    if (this->publish_odometry_tf_){
      transform_broadcaster_->sendTransform(
            tf::StampedTransform(base_footprint_to_odom, current_time,
                                 odom_frame, base_footprint_frame));
    }

    // publish odom topic
    odom_.pose.pose.position.x = pose.Pos().X();
    odom_.pose.pose.position.y = pose.Pos().Y();

    odom_.pose.pose.orientation.x = pose.Rot().X();
    odom_.pose.pose.orientation.y = pose.Rot().Y();
    odom_.pose.pose.orientation.z = pose.Rot().Z();
    odom_.pose.pose.orientation.w = pose.Rot().W();
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;

    // get velocity in /odom frame
    ignition::math::Vector3d linear;
    linear = this->parent->WorldLinearVel();;
    odom_.twist.twist.angular.z = this->parent->WorldAngularVel().Z();;

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = pose.Rot().Yaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
    odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    if (this->publish_odometry_msg_){
      odometry_publisher_.publish(odom_);
    }
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosWheelsPiston)
}
