#include <algorithm>
#include <assert.h>

#include <siar_plugins/plugin_siar_wheels_piston.h>

#include <gazebo/math/gzmath.hh>
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
  void GazeboRosWheelsPiston::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){

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

    // this->set_angle_elevation_arm_ = 0.62;
    // if (!_sdf->HasElement("setAngleElevationArm")) {
    //   ROS_WARN("GazeboRosWheelsPiston Plugin (ns = %s) missing <initWheelSeparation>, defaults to %f",
    //       this->robot_namespace_.c_str(), this->init_wheel_separation_);
    // } else {
    //   this->init_wheel_separation_ = 
    //     _sdf->GetElement("initWheelSeparation")->Get<double>();
    // }

    

    // Initialize update rate stuff
    if (this->update_rate_ > 0.0) {
      this->update_period_ = 1.0 / this->update_rate_;
    } else {
      this->update_period_ = 0.0;
    }
    last_update_time_ = this->world->GetSimTime();

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
    electronics_center = this->parent->GetLink("box_battery");
    thermal_camera_ = this->parent->GetLink("thermal_camera_arm");
    arm_siar_base_ = this->parent->GetLink("arm_siar_base");

    
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
    this -> axis_arm_1_ = this->parent->GetJoint("move_arm_1");
    // this -> axis_arm_2_1_ = this->parent->GetJoint("move_arm_2_1");
    // this -> axis_arm_2_2_ = this->parent->GetJoint("move_arm_2_2");
    this -> axis_arm_3_1_ = this->parent->GetJoint("move_arm_3_1");
    this -> axis_arm_3_2_ = this->parent->GetJoint("move_arm_3_2");

    
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
    
    // SIAR: create arm_bool subscriber (This change the width)
          so = ros::SubscribeOptions::create<std_msgs::Bool>("arm_bool", 1,
          boost::bind(&GazeboRosWheelsPiston::armCentralPosCallback, this, _1),
          ros::VoidPtr(), &queue_);
    arm_central_subscriber_= rosnode_->subscribe(so);

    // SIAR: create arm_mode to move the robotics Arm
          so = ros::SubscribeOptions::create<std_msgs::Bool>("arm_mode", 1,
          boost::bind(&GazeboRosWheelsPiston::moveArmPosCallback, this, _1),
          ros::VoidPtr(), &queue_);
    move_arm_subscriber_= rosnode_->subscribe(so);

    // SIAR: create arm_mode to move pan from robotics Arm
          so = ros::SubscribeOptions::create<std_msgs::Float32>("arm_pan", 1,
          boost::bind(&GazeboRosWheelsPiston::movePanArmCallback, this, _1),
          ros::VoidPtr(), &queue_);
    move_pan_arm_subscriber_= rosnode_->subscribe(so);

    // SIAR: create arm_mode to move tilt  from robotics Arm
          so = ros::SubscribeOptions::create<std_msgs::Float32>("arm_tilt", 1,
          boost::bind(&GazeboRosWheelsPiston::moveTiltArmCallback, this, _1),
          ros::VoidPtr(), &queue_);
    move_tilt_arm_subscriber_= rosnode_->subscribe(so);
       

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

    // Initial position 
    move_Piston_cmd_=1;
    move_Piston_aux_=1;
    elec_pos_cmd_ = 0;
    arm_central_cmd_=false;
    vel_state_cmd_=0;
    move_pan_arm_aux_ = 0;
    move_tilt_arm_aux_ = 1.7;
    move_elevation_arm_aux_ = 0;
    auxiliar = 0;
  }

  
  // Update the controller
  void GazeboRosWheelsPiston::UpdateChild() {
    common::Time current_time = this->world->GetSimTime();
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
      this->pid_hinge_arm = common::PID(0.1, 0.0, 0.02);
      this->pid_hinge_arm_2 = common::PID(0.1, 0.0, 0.02);
      //To control the width of SIAR
      this-> parent ->GetJointController()->SetPositionPID(this->hinge_arm_right_1_1_->GetScopedName(), this->pid_hinge_arm_right_left);
      this-> parent ->GetJointController()->SetPositionPID(this->hinge_arm_right_1_2_->GetScopedName(), this->pid_hinge_arm_right_left);
      this-> parent ->GetJointController()->SetPositionPID(this->hinge_arm_left_1_1_->GetScopedName(), this->pid_hinge_arm_right_left);
      this-> parent ->GetJointController()->SetPositionPID(this->hinge_arm_left_1_2_->GetScopedName(), this->pid_hinge_arm_right_left);
      this-> parent ->GetJointController()->SetPositionTarget(this->hinge_arm_right_1_1_->GetScopedName(), -1.2* elec_pos_cmd_);
      this-> parent ->GetJointController()->SetPositionTarget(this->hinge_arm_right_1_2_->GetScopedName(), -1.2* elec_pos_cmd_);
      this-> parent ->GetJointController()->SetPositionTarget(this->hinge_arm_left_1_1_->GetScopedName(),  1.2* elec_pos_cmd_);
      this-> parent ->GetJointController()->SetPositionTarget(this->hinge_arm_left_1_2_->GetScopedName(),  1.2* elec_pos_cmd_); 
   
      // Here to limit the first option move_Piston_cmd_ = 1 like value, because it is given problem like initial value
      if ( (move_Piston_cmd_ == 0)  || move_Piston_aux_ == 0)  {
	      elec_pos_cmd_ = move_Piston_cmd_ ;
	      move_Piston_aux_= 0;
	    }
 
      //Publish the position of electronics_center
      std_msgs::Float32 elec_pos_msg;
      elec_pos_msg.data = (-1*elec_pos_cmd_);
      elec_pos_publisher_.publish(elec_pos_msg);

        
      // Update robot in case new velocities have been requested or to control arm 
      if (move_arm_cmd_ == 0){
        getWheelVelocities();
        move_pan_arm_cmd_ = 0.0;
        move_tilt_arm_cmd_ = 1.7;
        // arm_pos_cmd_ = 1;
        ROS_INFO("The value of move_tilt_arm_cmd_ is: %f",move_tilt_arm_cmd_);

      }
      else{
        // arm_pos_cmd_ = 0.1;
        if (move_pan_arm_aux_ < 1.5707 && move_pan_arm_aux_ > -1.5707){
          move_pan_arm_aux_ = move_pan_arm_cmd_;
          // move_pan_arm_aux_ = move_pan_arm_aux_ + 0.01 * move_pan_arm_cmd_;
          this-> parent ->GetJointController()->SetPositionPID(this->axis_arm_1_->GetScopedName(), this->pid_hinge_arm);
          this-> parent ->GetJointController()->SetPositionTarget(this->axis_arm_1_->GetScopedName(),  move_pan_arm_aux_);
        // ROS_INFO("The value of move_pan_arm_aux_ is: %f",move_pan_arm_aux_);
        // ROS_INFO("The value of axis_arm_1_ is: %f",axis_arm_1_);

        }
        if (move_pan_arm_aux_ >= 1.55){
          move_pan_arm_aux_ = 1.549;}
        if (move_pan_arm_aux_ <= -1.55){
          move_pan_arm_aux_ = -1.549;}  

        if (move_tilt_arm_aux_ < 1.8 && move_tilt_arm_aux_ > -1.8){
          // move_tilt_arm_aux_ = move_tilt_arm_aux_ + 0.01 * move_tilt_arm_cmd_;
          if (auxiliar == 0) {
            move_tilt_arm_aux_ = 1.7;
          }
          if ((move_tilt_arm_cmd_ != 0.0) || (auxiliar ==  1)) {
            move_tilt_arm_aux_ = move_tilt_arm_cmd_;
            auxiliar = 1;
          }
          
          this-> parent ->GetJointController()->SetPositionPID(this->axis_arm_3_1_->GetScopedName(), this->pid_hinge_arm);
          this-> parent ->GetJointController()->SetPositionTarget(this->axis_arm_3_1_->GetScopedName(),  move_tilt_arm_aux_);
          this-> parent ->GetJointController()->SetPositionPID(this->axis_arm_3_2_->GetScopedName(), this->pid_hinge_arm);
          this-> parent ->GetJointController()->SetPositionTarget(this->axis_arm_3_2_->GetScopedName(),  move_tilt_arm_aux_);
          // ROS_INFO("The value of move_tilt_arm_cmd_ is: %f",move_tilt_arm_cmd_);
          // ROS_INFO("The value of move_tilt_arm_aux_ is: %f",move_tilt_arm_aux_);
          // ROS_INFO("The value of axis_arm_3_1_ is: %f",axis_arm_3_1_);
          // ROS_INFO("The value of axis_arm_3_2_ is: %f",axis_arm_3_2_);
        }
        if (move_tilt_arm_aux_ >= 1.8){
          move_tilt_arm_aux_ = 1.79;}
        if (move_tilt_arm_aux_ <= -1.8){
          move_tilt_arm_aux_ = -1.79;}  
      }

      
    
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
   
    wheel_speed_[LEFT] = 2*vr - va * width_ / (2.0*0.125);
    wheel_speed_[RIGHT] = 2*vr + va * width_ / (2.0*0.125);
  }

  void GazeboRosWheelsPiston::updateWidth(){
      
     math::Vector3 dis = l_c_wheel_->GetWorldCoGPose().pos - r_c_wheel_->GetWorldCoGPose().pos;
     width_ = sqrt(((l_c_wheel_->GetWorldCoGPose().pos.x - r_c_wheel_->GetWorldCoGPose().pos.x) * (l_c_wheel_->GetWorldCoGPose().pos.x - r_c_wheel_->GetWorldCoGPose().pos.x)) + ((l_c_wheel_->GetWorldCoGPose().pos.y - r_c_wheel_->GetWorldCoGPose().pos.y) * (l_c_wheel_->GetWorldCoGPose().pos.y - r_c_wheel_->GetWorldCoGPose().pos.y))) + 0.10001;

  }

  void GazeboRosWheelsPiston::updateElecPos(){
     math::Vector3 rl, rr,u,pe;
// convert velocity to child_frame_id (aka base_footprint)
    math::Pose pose = this->parent->GetWorldPose();
    float yaw = pose.rot.GetYaw();

     rb.x = electronics_center->GetWorldCoGPose().pos.x;
     rb.y = electronics_center->GetWorldCoGPose().pos.y;
     rb.z = electronics_center->GetWorldCoGPose().pos.z;
     rl.x = l_c_wheel_->GetWorldCoGPose().pos.x;
     rl.y = l_c_wheel_->GetWorldCoGPose().pos.y;
     rl.z = l_c_wheel_->GetWorldCoGPose().pos.z - 0.125;
     rr.x = r_c_wheel_->GetWorldCoGPose().pos.x;
     rr.y = r_c_wheel_->GetWorldCoGPose().pos.y;
     rr.z = r_c_wheel_->GetWorldCoGPose().pos.z - 0.125;  
     rm = (rl + rr) * 0.5;
     pe= rb - rm;
     u.x= cos (yaw);
     u.y= sin (yaw);
     u.z= 0;

// HERE IS NECESARY TO CHECK HOW TO PRESENT THE FUNCTION DOT PRODUCT TO APPLY
     dis_box_centralaxis_ = (pe.x*u.x+pe.y*u.y+pe.z*u.z)*10;
     
     //Get the position of Electronic Box
     geometry_msgs::Vector3 pos_electronicBox_msg;
     pos_electronicBox_msg.x = electronics_center->GetWorldCoGPose().pos.x;
     pos_electronicBox_msg.y = electronics_center->GetWorldCoGPose().pos.y;
     pos_electronicBox_msg.z = 0;
     pos_electronicBox_publisher_.publish(pos_electronicBox_msg);
     
      
     //Get the Vector Central  Between Middle Wheels
     geometry_msgs::Vector3 pos_centerMidWheels_msg;
     pos_centerMidWheels_msg.x = rm.x;
     pos_centerMidWheels_msg.y = rm.y;
     pos_centerMidWheels_msg.z = rm.z;
     pos_centerMidWheels_publisher_.publish(pos_centerMidWheels_msg);
      
      
     //Get Vector Diference XY between center Electronic Box and Centor Central Middle Wheels
     geometry_msgs::Vector3 pos_vecBoxWheel_msg;
     pos_vecBoxWheel_msg.x = pe.x;
     pos_vecBoxWheel_msg.y = pe.y;
     pos_vecBoxWheel_msg.z = pe.z;
     pos_vecBoxWheel_publisher_.publish(pos_vecBoxWheel_msg);
     
     
     //Get unit Vector parrallel with orientation of Robot
     geometry_msgs::Vector3 pos_vecUnitOrient_msg;
     pos_vecUnitOrient_msg.x = u.x;
     pos_vecUnitOrient_msg.y = u.y;
     pos_vecUnitOrient_msg.z = u.z;
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
  void GazeboRosWheelsPiston::moveArmPosCallback(
      const std_msgs::Bool::ConstPtr& move_arm_msg) {
    
    boost::mutex::scoped_lock scoped_lock(lock);
    move_arm_cmd_ = move_arm_msg->data;
   }

   void GazeboRosWheelsPiston::movePanArmCallback(
      const std_msgs::Float32::ConstPtr& move_pan_arm_msg) {
    
    boost::mutex::scoped_lock scoped_lock(lock);
    move_pan_arm_cmd_ = move_pan_arm_msg->data;
   } 

   void GazeboRosWheelsPiston::moveTiltArmCallback(
      const std_msgs::Float32::ConstPtr& move_tilt_arm_msg) {
    
    boost::mutex::scoped_lock scoped_lock(lock);
    move_tilt_arm_cmd_ = move_tilt_arm_msg->data;
   }    

  
  void GazeboRosWheelsPiston::tfBaseLink(void)
  {
    static tf::TransformBroadcaster br;
    
    math::Pose tf_thermal_camera;
    std::string frame_name_8 = "odom";

 
    tf_thermal_camera = thermal_camera_->GetRelativePose();
    

    tf::Transform t_t;
    t_t.setOrigin( tf::Vector3(tf_thermal_camera.pos.x, tf_thermal_camera.pos.y, tf_thermal_camera.pos.z) );
    t_t.setRotation(tf::Quaternion(tf_thermal_camera.rot.x,tf_thermal_camera.rot.y,tf_thermal_camera.rot.z,tf_thermal_camera.rot.z));

    br.sendTransform(tf::StampedTransform(t_t, ros::Time::now(), "siar_arm", "thermal_camera"));
  }
  
  
  void GazeboRosWheelsPiston::publishOdometry(double step_time) {
    ros::Time current_time = ros::Time::now();
    std::string odom_frame = tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_footprint_frame = tf::resolve(tf_prefix_, robot_base_frame_);

    // getting data for base_footprint to odom transform
    math::Pose pose = this->parent->GetWorldPose();

    tf::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
    tf::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);
    tf::Transform base_footprint_to_odom(qt, vt);

    if (this->publish_odometry_tf_){
      transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom, current_time,
                                                                  odom_frame, base_footprint_frame));
    }


    // publish odom topic
    odom_.pose.pose.position.x = pose.pos.x;
    odom_.pose.pose.position.y = pose.pos.y;

    odom_.pose.pose.orientation.x = pose.rot.x;
    odom_.pose.pose.orientation.y = pose.rot.y;
    odom_.pose.pose.orientation.z = pose.rot.z;
    odom_.pose.pose.orientation.w = pose.rot.w;
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;

    // get velocity in /odom frame
    math::Vector3 linear;
    linear = this->parent->GetWorldLinearVel();
    odom_.twist.twist.angular.z = this->parent->GetWorldAngularVel().z;

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = pose.rot.GetYaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.x + sinf(yaw) * linear.y;
    odom_.twist.twist.linear.y = cosf(yaw) * linear.y - sinf(yaw) * linear.x;

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    if (this->publish_odometry_msg_){
      odometry_publisher_.publish(odom_);
    }
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosWheelsPiston)
}
