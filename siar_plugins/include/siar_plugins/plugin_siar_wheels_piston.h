/*
 * Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/

/*
 * \file  gazebo_ros_diff_drive.h
 *
 * \brief A differential drive plugin for gazebo. Based on the diffdrive plugin 
 * developed for the erratic robot (see copyright notice above). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * $ Id: 06/21/2013 11:23:40 AM piyushk $
 */

/**
 * A diff drive plugin supporting multiple wheels per vehicle side. Based on
 * existing plugins as stated above this notice.
 */

/*
    Copyright (c) 2014, Stefan Kohlbrecher
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef DIFFDRIVE_PLUGIN_HH
#define DIFFDRIVE_PLUGIN_HH

#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

//Interpolate process
#include "functions/linear_interpolator.hpp"

namespace gazebo {

  class Joint;
  class Entity;

  class GazeboRosWheelsPiston : public ModelPlugin {

    public:
      GazeboRosWheelsPiston();
//       GazeboRosWheelsPiston(ros::NodeHandle& rosnode_p);
      ~GazeboRosWheelsPiston();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      common::PID pid_piston_main_1, pid_piston_main_2, pid_hinge_arm_right_left, pid_hinge_arm, pid_hinge_arm_2;
//       int argc; 
//       char** argv;

    protected:
      virtual void UpdateChild();
      virtual void FiniChild();

    private:
      void publishOdometry(double step_time);
      void getWheelVelocities();
      void updateWidth();
      void updateElecPos();
      void updatePosElement();
      void tfBaseLink();



      physics::WorldPtr world;
      physics::ModelPtr parent;
      event::ConnectionPtr update_connection_;

      std::vector<std::string> joint_names_[2];

      double init_wheel_separation_;
      double wheel_diameter_;
      double speed_factor_;
      double torque;
      double wheel_speed_[2];

      double width_, elec_pos_, dis_box_centralaxis_, Force_piston_, limit_angle_pan, limit_angle_tilt ;

      std::vector<physics::JointPtr> joints_[2];

      // SIAR links
      std::vector <physics::LinkPtr> frame_camera_;
      physics::LinkPtr l_c_wheel_, r_c_wheel_,electronic_box_,electronics_center, os1_sensor_, piston_, 
                        frame_camera_1_,frame_camera_2_,frame_camera_3_,frame_camera_4_,frame_camera_5_,
		                    frame_camera_6_,frame_camera_7_, arm_siar_base_, arm_link_1_1_, arm_link_1_2_,
                        arm_link_2_1_, arm_link_2_2_, arm_link_3_, frame_thermal_;
      //std::vector<physics::LinkPtr> frame_camera_; 
      physics::JointPtr piston_main_1_, piston_main_2_,hinge_arm_right_1_1_,hinge_arm_right_1_2_, hinge_arm_left_1_1_, hinge_arm_left_1_2_, 
			axis_wheel_right_1_,axis_wheel_right_2_,axis_wheel_right_3_,axis_wheel_left_1_,axis_wheel_left_2_,axis_wheel_left_3_,
      axis_arm_1_,axis_arm_2_1_,axis_arm_2_2_,axis_arm_3_1_, axis_arm_3_2_;
  

      // ROS STUFF
      ros::NodeHandle* rosnode_;
//       ros::NodeHandle& rosnode_p;
      ros::Publisher odometry_publisher_, width_publisher_, pos_electronicBox_publisher_,pos_centerMidWheels_publisher_,pos_vecBoxWheel_publisher_,
		                  pos_vecUnitOrient_publisher_,  dis_box_centralaxis_publisher_,elec_pos_publisher_,pos_piston_publisher_,siar_status_publisher_,
		                  tf_base_link_publisher_,test_velocity_publisher_;
      ros::Subscriber cmd_vel_subscriber_, move_Piston_subscriber_, arm_central_subscriber_,vel_state_subscriber_, 
                      move_arm_subscriber_,move_arm_as_subscriber_, move_pan_arm_subscriber_, move_tilt_arm_subscriber_;
      tf::TransformBroadcaster *transform_broadcaster_;
      nav_msgs::Odometry odom_;
      geometry_msgs::TransformStamped odom_trans;
      std::string tf_prefix_;

      boost::mutex lock;

      std::string robot_namespace_;
      std::string command_topic_;
      std::string odometry_topic_;
      std::string odometry_frame_;
      std::string robot_base_frame_;

      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      // DiffDrive stuff
      void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
      // Move Piston stuff
      void elecPosCallback(const std_msgs::Float32::ConstPtr& move_Piston_msg);
      // Move Pisto Center stuff
      void armCentralPosCallback(const std_msgs::Bool::ConstPtr& arm_central_msg);
      
      void velStateCallback (const std_msgs::Float32::ConstPtr& vel_state_msg);

      void moveArmPosCallback(const std_msgs::Bool::ConstPtr& move_arm_msg);

      void moveArmPosASCallback(const std_msgs::Bool::ConstPtr& move_arm_msg);

      void movePanArmCallback(const std_msgs::Float32::ConstPtr& move_pan_arm_msg);
      
      void moveTiltArmCallback(const std_msgs::Float32::ConstPtr& move_tilt_arm_msg);

      double x_;
      double rot_;
      double elec_pos_cmd_, move_Piston_cmd_,vel_state_cmd_,move_Piston_aux_,
              arm_pos_cmd_, move_pan_arm_cmd_, move_tilt_arm_cmd_,
              move_pan_arm_add_,move_tilt_arm_add_, move_elevation_arm_aux_;
      bool alive_, arm_central_cmd_,move_arm_cmd_, move_arm_as_cmd_, auxiliar_tilt;
      math::Vector3 rm, rb,frame_thermal; 
     
      // Update Rate
      double update_rate_;
      double update_period_;
      common::Time last_update_time_;

      bool publish_odometry_tf_;
      bool publish_odometry_msg_;
      std::vector <std::string> tf_frame_name_;
      
      // Find coefficients to multiply vr and va then to have the same proportion en cmd_vel and odomTopic
      functions::LinearInterpolator *interp_cmd_vel, *inter_vr, *inter_va;
      std::string cmd_vel_file, vr_file, va_file;

  };

}

#endif
