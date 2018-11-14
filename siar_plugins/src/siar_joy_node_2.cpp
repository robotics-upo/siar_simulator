/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"
#include <sstream> 

class TurtlebotTeleop
{
public:
  TurtlebotTeleop();
private:
  
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();
  void publish1();  
  void joyConvert();

  ros::NodeHandle ph_, nh_;

  int linear_, angular_, arm_, deadman_axis_, central_arm1_, central_arm2_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_,pos_pub_,arm_pub_,deadman_pressed_pub_, vel_state_pub_,auto_botton_pub_,mode_botton_pub_;
  ros::Subscriber joy_sub_;
 
  geometry_msgs::Twist last_published_;
  std_msgs::Bool central_arm, central_arm_msg_,deadman_pressed_msg_, deadman_pressed,mode_botton_,mode_botton_msg,aux_botton_;
  std_msgs::Float32  elec_pos_cmd, elec_pos_msg_,vel_state_cmd,vel_state_msg_;
  std_msgs::Int8 operation_mode;
  boost::mutex publish_mutex_;
  bool deadman_pressed_ , arm1_, arm2_,auto_botton_;
  bool zero_twist_published_;
  ros::Timer timer_;

};

TurtlebotTeleop::TurtlebotTeleop():
  ph_("~"),
  linear_(1.0),
  angular_(0),
  arm_(4),
  auto_botton_(3),
  deadman_axis_(4),
  central_arm1_(1),
  central_arm2_(2),
  l_scale_(0.95), //it was 0.75
  a_scale_(-6.0)
{
  ph_.param("axis_linear", linear_, linear_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("axis_arm", arm_, arm_);
  ph_.param("change_mode", auto_botton_,auto_botton_);
  ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
  ph_.param("axiscentral_arm1", central_arm1_, central_arm1_);
  ph_.param("axiscentral_arm2", central_arm2_, central_arm2_);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);

  deadman_pressed_ = false;
  zero_twist_published_ = false;
  
  

  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  vel_state_pub_ = ph_.advertise<std_msgs::Float32>("vel_state", 1, true);
  pos_pub_ = ph_.advertise<std_msgs::Float32>("move_Piston", 1, true);
  arm_pub_ = ph_.advertise<std_msgs::Bool>("arm_bool", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TurtlebotTeleop::joyCallback, this);
  deadman_pressed_pub_ =ph_.advertise<std_msgs::Bool>("deadman_pressed", 1, true);
  auto_botton_pub_ =ph_.advertise<std_msgs::Int8>("auto_botton", 1,true);
  mode_botton_pub_ =ph_.advertise<std_msgs::Bool>("mode_botton", 1, true);

  timer_ = nh_.createTimer(ros::Duration(0.05), boost::bind(&TurtlebotTeleop::publish,this));
}

void TurtlebotTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  geometry_msgs::Twist vel, pos;
  vel.angular.z = a_scale_*joy->axes[angular_];
  vel.linear.x = l_scale_*joy->axes[linear_];
  last_published_ = vel;
  vel_state_cmd.data = vel.angular.z + vel.linear.x;
  vel_state_msg_ = vel_state_cmd;
  
  deadman_pressed_ = joy->buttons[deadman_axis_];
  deadman_pressed.data = deadman_pressed_;
  deadman_pressed_msg_ = deadman_pressed;
  
  mode_botton_msg.data = joy -> buttons[auto_botton_];
  mode_botton_.data = mode_botton_msg.data;
  
  arm1_ = joy->buttons[central_arm1_];
  arm2_ = joy->buttons[central_arm2_];
  central_arm.data = arm1_ && arm2_;
  central_arm_msg_ = central_arm;
  
  elec_pos_cmd.data = joy->axes[arm_];
  elec_pos_msg_ = elec_pos_cmd;
}

void TurtlebotTeleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  
  if (deadman_pressed_)
  {
    vel_pub_.publish(last_published_);
    vel_state_pub_.publish(vel_state_msg_);
    pos_pub_.publish(elec_pos_msg_);
    arm_pub_.publish(central_arm_msg_); 
    deadman_pressed_pub_.publish(deadman_pressed_msg_); 
    auto_botton_pub_.publish(operation_mode);
    mode_botton_pub_.publish(mode_botton_);
	  
    if (mode_botton_.data && !aux_botton_.data)
    {
      aux_botton_.data = true;
      operation_mode.data = 1;
      auto_botton_pub_.publish(operation_mode);
      timer_;
    }
    else if (mode_botton_.data && aux_botton_.data)
    {
      aux_botton_.data = false;
      operation_mode.data = 0;
      auto_botton_pub_.publish(operation_mode);
      timer_; 
    }
    
      
    zero_twist_published_=false;
  }
  else if(!deadman_pressed_ && !zero_twist_published_)
  {
    vel_pub_.publish(*new geometry_msgs::Twist());

    zero_twist_published_=true;
  }
 /*  if (deadman_pressed_)
  {
      pos_pub_.publish(elec_pos_msg_);
    
    zero_twist_published_=false;
  }
 else if(!deadman_pressed_ && !zero_twist_published_)
  {
      pos_pub_.publish(*new std_msgs::Float32());

    zero_twist_published_=true;
  }*/
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "siar_teleop");
  TurtlebotTeleop siar_teleop;

  ros::spin();
}
