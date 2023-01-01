
/******************************************************************************
# openrst_control.h:   OpenRST ROS controller                                 #
# Copyright (c) 2022                                                          #
# Hasegawa Laboratory at Nagoya University                                    #
#                                                                             #
# Redistribution and use in source and binary forms, with or without          #
# modification, are permitted provided that the following conditions are met: #
#                                                                             #
#     - Redistributions of source code must retain the above copyright        #
#       notice, this list of conditions and the following disclaimer.         #
#     - Redistributions in binary form must reproduce the above copyright     #
#       notice, this list of conditions and the following disclaimer in the   #
#       documentation and/or other materials provided with the distribution.  #
#     - Neither the name of the Hasegawa Laboratory nor the                   #
#       names of its contributors may be used to endorse or promote products  #
#       derived from this software without specific prior written permission. #
#                                                                             #
# This program is free software: you can redistribute it and/or modify        #
# it under the terms of the GNU Lesser General Public License LGPL as         #
# published by the Free Software Foundation, either version 3 of the          #
# License, or (at your option) any later version.                             #
#                                                                             #
# This program is distributed in the hope that it will be useful,             #
# but WITHOUT ANY WARRANTY; without even the implied warranty of              #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                #
# GNU Lesser General Public License LGPL for more details.                    #
#                                                                             #
# You should have received a copy of the GNU Lesser General Public            #
# License LGPL along with this program.                                       #
# If not, see <http://www.gnu.org/licenses/>.                                 #
#                                                                             #
# #############################################################################
#                                                                             #
#   Author: Jacinto Colan, email: colan@robo.mein.nagoya-u.ac.jp              #
#                                                                             #
# ###########################################################################*/

#ifndef OPENRST_CONTROL_H
#define OPENRST_CONTROL_H

#include <openrst_control/openrst_hw.h>

// C
#include <pthread.h>
#include <pwd.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

// C++
#include <cmath>

// External
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <controller_manager/controller_manager.h>

// Internal
#include <openrst_control/openrst_request.h>
#include <openrst_control/openrst_status_code.h>
#include <rt_utils/rt_clock.h>
#include <usb1608/usb1608.h>
#include <usb3104/usb3104.h>
#include <usbquad08/usbquad08.h>
#include <mc_daq_ros/daq_cmd.h>

using namespace Eigen;
using namespace hardware_interface;
using namespace realtime_utils;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

namespace openrst_nu
{

  const double kDeg2Rad = (M_PI) / 180;
  const double kRad2Deg = 180 / (M_PI);

  class OpenRSTControl : public OpenRSTHW
  {
    enum
    {
      CALIB_STOP = -1,
      CALIB_READY,
      CALIB_M0_HIGH,
      CALIB_M0_LOW,
      SET_M0_ZERO,
      CALIB_M1_HIGH,
      CALIB_M2_HIGH,
      CALIB_M1_LOW,
      SET_M1_HIGH,
      CALIB_M2_LOW,
      SET_M2_ZERO,
      SET_M1_ZERO,
      CALIB_COMPLETED
    };

    struct ArmJointLimits
    {
      std::string name;
      bool has_position_limits = {false};
      double min_pos = {0.0};
      double max_pos = {0.0};
      bool has_velocity_limits = {false};
      double max_vel = {0.0};
      bool has_effort_limits = {false};
      double max_eff = {0.0};
    };

  public:
    //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Constructor
    OpenRSTControl(ros::NodeHandle &node_handle, bool *kill_this_node);
    ~OpenRSTControl();

    // Callbacks
    bool
    SrvOpenRSTCommandCb(openrst_control::openrst_request::Request &request,
                        openrst_control::openrst_request::Response &response);

    void SubUpdateSimJointStateCb(const sensor_msgs::JointState::ConstPtr &msg);
    void
    SubUpdateJointCommandCb(const std_msgs::Float64MultiArray::ConstPtr &msg);

    void SubUpdateAiStateCb(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void SubUpdateDiStateCb(const std_msgs::Int32MultiArray::ConstPtr &msg);
    void SubUpdateAoStateCb(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void SubUpdateEncStateCb(const std_msgs::Int32MultiArray::ConstPtr &msg);

    // Accessors

    // Mutators

    // Functions
    int ControlLoop();
    void PublishRobotState();

    bool connect();
    bool disconnect();
    bool calibrate();
    bool posControlOn();
    bool torqueControlOn();
    bool controlOff();

    bool IsModeChanged();

    // Accessor
    int get_mode();

    // Mutators

    void SafeSleepSeconds(double seconds);
    void BlockSleepSeconds(double useconds);
    void SendCmdRobot();
    void SendCmdSim();
    void Read();
    void Write();
    bool Read(const ros::Time time, const ros::Duration period);
    void Write(const ros::Time time, const ros::Duration period);

  private:
    // DAQ channels vairables
    // Current sensors channels
    int kAiIndexCurrMotor0 = 0;
    int kAiIndexCurrMotor1 = 1;
    int kAiIndexCurrMotor2 = 2;
    // Engagement photosensors channels
    int kAiIndexSensor0 = 8;
    int kAiIndexSensor1 = 9;
    int kAiIndexSensor2 = 10;
    // const unsigned int kDiIndexSwitch = 0;    // Not implemented
    // const unsigned int kDoIndexLedYellow = 0; // Not implemented
    // const unsigned int kDoIndexLedGreen = 1;  // Not implemented
    // Motor Driver Control channels
    int kAoIndexMotor0 = 0;
    int kAoIndexMotor1 = 1;
    int kAoIndexMotor2 = 2;
    //  Motor Encoder channels
    int kEncIndexMotor0 = 0;
    int kEncIndexMotor1 = 1;
    int kEncIndexMotor2 = 2;
    // Photosensor voltage treshold for interface engaged
    double kSensorTresh0 = 0.12;
    double kSensorTresh1 = 0.12;
    double kSensorTresh2 = 0.12;
    // Motor encoder especifications
    int kRatioMotorPitch = 3640;  // [counts/rev] [64*64*36/32]
    int kRatioMotorFinger = 4096; // [counts/rev] [64*64*40/40]
    // Motor current for calibration step
    double kCalibMotorCurr = 0.1; // [A]
    // Conversion factor from Motor current to Control voltage
    double kMotorCurrToControlSignal = 50; // 10V/0.2A

    // PID gains fro calibration
    double kMotorKp0 = 0.003;  // 0.0175
    double kMotorKp1 = 0.0045; // 0.0175
    double kMotorKp2 = 0.006;  // 0.0175

    // ROS
    ros::NodeHandle nh_;

    // Robot variables
    int openrst_id_;
    int n_dof_;
    // std::string ip_address_;

    // Services Server
    ros::ServiceServer srv_server_openrst_request_;
    // Service Client
    ros::ServiceClient srv_client_daq_cmd_;
    mc_daq_ros::daq_cmd srv_client_daq_cmd_req_;

    // Suscribers
    ros::Subscriber sub_sim_joint_state_;
    ros::Subscriber sub_joint_cmd_;
    ros::Subscriber sub_ai_state_;
    ros::Subscriber sub_di_state_;
    ros::Subscriber sub_ao_state_;
    ros::Subscriber sub_enc_state_;

    // Publishers
    ros::Publisher pub_state_;
    std_msgs::Int32MultiArray pub_state_msg_;

    ros::Publisher pub_sim_joint_cmd_;
    sensor_msgs::JointState pub_sim_joint_cmd_msg_;

    ros::Publisher pub_ao_cmd_;
    std_msgs::Float64MultiArray pub_ao_cmd_msg_;

    ros::Publisher pub_do_cmd_;
    std_msgs::Int32MultiArray pub_do_cmd_msg_;

    // Joint variables
    VectorXd des_joint_pos_;
    VectorXd des_joint_eff_;
    VectorXd act_joint_pos_;
    VectorXd sim_joint_pos_;
    VectorXd daq_joint_pos_;

    VectorXd enc_offset_;
    VectorXd enc_joint_range_;
    VectorXd enc_range_;
    VectorXd enc_coeff_;

    // DAQ variables

    std::vector<double> ai_state_;
    std::vector<int> di_state_;
    std::vector<double> ao_state_;
    std::vector<int> enc_state_;
    std::vector<double> ao_cmd_;
    std::vector<int> do_cmd_;

    // RT
    RTClock rt_clock_;
    int cycle_t_us_;

    // Current state of this process
    int state_;

    // b-Cap error storage
    int bcap_error_code_;

    // Bool to kill loops
    bool *kill_this_node_;

    // Asynchronous spinner
    ros::AsyncSpinner *spinner;

    // Mutex
    std::mutex m_mtxAct;

    // Joint limits vector
    std::vector<ArmJointLimits> joint_limits_;

    // Joint names
    std::vector<std::string> joint_names_;

    // Simulator
    bool use_sim_;
    bool mode_changed_;
    int mode_;

    // Time variables
    ros::Time now_timestamp_;
    ros::Time prev_timestamp_;
    ros::Duration period_;

    // DAQ parameters
    int ai_channels_;
    int ao_channels_;
    int di_channels_;
    int do_channels_;
    int enc_channels_;
  };
} // namespace openrst_nu

#endif
