/******************************************************************************
# openrst_control.cpp:   OpenRST ROS controller                               #
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

#include <openrst_control/openrst_control.h>

// C
#include <signal.h>

// Additional libraries
#include <ros/ros.h>

namespace openrst_nu
{

  // Constructor
  OpenRSTControl::OpenRSTControl(ros::NodeHandle &node_handle,
                                 bool *kill_this_node)
  {
    nh_ = node_handle;

    ai_channels_ = 8;
    ao_channels_ = 8;
    di_channels_ = 4;
    do_channels_ = 4;
    enc_channels_ = 4;

    //* ROS PARAMETER SERVER
    // Robot Identifier
    if (nh_.getParam("openrst_id", openrst_id_))
    {
      ROS_INFO_STREAM(
          "Robot ID obtained from ROS Parameter server as: " << openrst_id_);
    }
    else
    {
      ROS_INFO_STREAM(
          "No ID information in parameter server, using default: [0]");
      openrst_id_ = 0;
    }

    // Get Current Sensors configuration from ROS Param
    if (!nh_.getParam("m0_current_ai_ch", kAiIndexCurrMotor0))
    {
      ROS_WARN("No AI channels set for Motor 0 [Current sensor] in launch "
               "file. Default is 0");
    }

    if (!nh_.getParam("m1_current_ai_ch", kAiIndexCurrMotor1))
    {
      ROS_WARN("No AI channels set for Motor 1 [Current sensor] in launch "
               "file. Default is 1");
    }

    if (!nh_.getParam("m2_current_ai_ch", kAiIndexCurrMotor2))
    {
      ROS_WARN("No AI channels set for Motor 0 [Current sensor] in launch "
               "file. Default is 2");
    }

    // Get Engagement Photosensors configuration from ROS Param
    if (!nh_.getParam("m0_photosensor_ai_ch", kAiIndexSensor0))
    {
      ROS_WARN("No AI channels set for Motor 0 [Engagement sensor] in launch "
               "file. Default is 8");
    }

    if (!nh_.getParam("m1_photosensor_ai_ch", kAiIndexSensor1))
    {
      ROS_WARN("No AI channels set for Motor 1 [Engagement sensor] in launch "
               "file. Default is 9");
    }

    if (!nh_.getParam("m2_photosensor_ai_ch", kAiIndexSensor2))
    {
      ROS_WARN("No AI channels set for Motor 2 [Engagement sensor] in launch "
               "file. Default is 10");
    }

    // Get Motor Driver Control configuration from ROS Param
    if (!nh_.getParam("m0_control_ch", kAoIndexMotor0))
    {
      ROS_WARN("No AO channels set for Motor 0 [Motor control] in launch "
               "file. Default is 0");
    }

    if (!nh_.getParam("m1_control_ch", kAoIndexMotor1))
    {
      ROS_WARN("No AO channels set for Motor 1 [Motor control] in launch "
               "file. Default is 1");
    }

    if (!nh_.getParam("m2_control_ch", kAoIndexMotor2))
    {
      ROS_WARN("No AO channels set for Motor 2 [Motor control] in launch "
               "file. Default is 2");
    }

    // Get Motor Encoder configuration from ROS Param
    if (!nh_.getParam("m0_encoder_ch", kEncIndexMotor0))
    {
      ROS_WARN("No ENC channel set for Motor 0 [Encoder] in launch "
               "file. Default is 0");
    }

    if (!nh_.getParam("m1_encoder_ch", kEncIndexMotor1))
    {
      ROS_WARN("No ENC channel set for Motor 1 [Encoder] in launch "
               "file. Default is 1");
    }

    if (!nh_.getParam("m2_encoder_ch", kEncIndexMotor2))
    {
      ROS_WARN("No ENC channel set for Motor 2 [Encoder] in launch "
               "file. Default is 2");
    }

    if (!nh_.getParam("m0_engage_sensor_threshold", kSensorTresh0))
    {
      ROS_WARN("No Engaged sensor threshold. Default is 0.1 V");
    }
    if (!nh_.getParam("m1_engage_sensor_threshold", kSensorTresh1))
    {
      ROS_WARN("No Engaged sensor threshold. Default is 0.1 V");
    }
    if (!nh_.getParam("m2_engage_sensor_threshold", kSensorTresh2))
    {
      ROS_WARN("No Engaged sensor threshold. Default is 0.1 V");
    }

    // Thread Sampling Time
    if (!nh_.getParam("cyclic_time_usec", cycle_t_us_))
      cycle_t_us_ = 2000;

    ROS_INFO("OpenRST Controller cyclic time [us] : %d", cycle_t_us_);

    // Robot simulaton
    if (!nh_.getParam("use_sim", use_sim_))
      use_sim_ = true;

    ROS_INFO_STREAM("OpenRST simulator:" << std::boolalpha << use_sim_);

    // Joint names
    if (!nh_.getParam("effort_controller/joints", joint_names_))
      joint_names_.clear();

    n_dof_ = joint_names_.size();
    ROS_INFO("Number of OpenRST Joints: %d", n_dof_);

    // Services
    srv_server_openrst_request_ = nh_.advertiseService(
        "openrst_request", &OpenRSTControl::SrvOpenRSTCommandCb, this);

    srv_client_daq_cmd_ = nh_.serviceClient<mc_daq_ros::daq_cmd>("daq_command");

    // Subscribers
    sub_sim_joint_state_ = nh_.subscribe(
        "sim/joint/state", 1, &OpenRSTControl::SubUpdateSimJointStateCb, this);
    sub_joint_cmd_ =
        nh_.subscribe("effort_controller/command", 1,
                      &OpenRSTControl::SubUpdateJointCommandCb, this);
    if (!use_sim_)
    {
      sub_ai_state_ = nh_.subscribe("/usb1608/ai/state", 1,
                                    &OpenRSTControl::SubUpdateAiStateCb, this);
      sub_di_state_ = nh_.subscribe("/usb1608/di/state", 1,
                                    &OpenRSTControl::SubUpdateDiStateCb, this);
      sub_ao_state_ = nh_.subscribe("/usb3104/ao/state", 1,
                                    &OpenRSTControl::SubUpdateAoStateCb, this);
      sub_enc_state_ =
          nh_.subscribe("/usbquad08/enc/state", 1,
                        &OpenRSTControl::SubUpdateEncStateCb, this);
    }

    // Publishers
    pub_state_ = nh_.advertise<std_msgs::Int32MultiArray>("openrst_state", 1);
    pub_sim_joint_cmd_ =
        nh_.advertise<sensor_msgs::JointState>("sim/joint/cmd", 1);

    if (!use_sim_)
    {
      pub_ao_cmd_ =
          nh_.advertise<std_msgs::Float64MultiArray>("/usb3104/ao/cmd", 1);
      pub_do_cmd_ =
          nh_.advertise<std_msgs::Int32MultiArray>("/usb1608/do/cmd", 1);
    }

    // Resize publishing messages
    pub_sim_joint_cmd_msg_.name.resize(n_dof_);
    pub_sim_joint_cmd_msg_.position.resize(n_dof_);
    pub_sim_joint_cmd_msg_.velocity.resize(n_dof_);
    pub_sim_joint_cmd_msg_.effort.resize(n_dof_);

    // Real time clock intialization
    rt_clock_ = RTClock(cycle_t_us_);

    kill_this_node_ = kill_this_node;

    // Flags

    // Command frame
    des_joint_pos_ = VectorXd::Zero(n_dof_);
    act_joint_pos_ = VectorXd::Zero(n_dof_);
    sim_joint_pos_ = VectorXd::Zero(n_dof_);
    daq_joint_pos_ = VectorXd::Zero(n_dof_);
    des_joint_eff_ = VectorXd::Zero(n_dof_);

    enc_offset_ = VectorXd::Zero(n_dof_);
    enc_joint_range_ = VectorXd::Zero(n_dof_);
    enc_range_ = VectorXd::Zero(n_dof_);
    enc_coeff_ = VectorXd::Zero(n_dof_);

    // For 3 joints
    enc_joint_range_[0] = 180 * kDeg2Rad;
    enc_joint_range_[1] = 180 * kDeg2Rad;
    enc_joint_range_[2] = 180 * kDeg2Rad;

    ao_cmd_.resize(ao_channels_);
    std::fill(ao_cmd_.begin(), ao_cmd_.end(),
              -100.0);             // All outputs are not enabled
    ao_cmd_[kAoIndexMotor0] = 0.0; // Output for Motor 0 enable
    ao_cmd_[kAoIndexMotor1] = 0.0; // Output for Motor 1 enable
    ao_cmd_[kAoIndexMotor2] = 0.0; // Output for Motor 2 enable

    // Initialize ros messages
    pub_state_msg_.data.resize(1);

    // Initial state
    state_ = F_UNINITIALIZED;

    jointInterfaceResize(n_dof_);
    jointInterfaceSetZero();

    // Initialize controllers
    for (int i = 0; i < n_dof_; i++)
    {
      // Create Joint state interface
      JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i],
                                        &joint_velocity_[i], &joint_effort_[i]);
      joint_state_interface_.registerHandle(jointStateHandle);

      // Create Position Joint interface
      JointHandle jointPositionHandle(jointStateHandle,
                                      &joint_position_cmd_[i]);
      position_interface_.registerHandle(jointPositionHandle);

      // Create Velocity Joint interface
      JointHandle jointVelocityHandle(jointStateHandle,
                                      &joint_velocity_cmd_[i]);
      velocity_interface_.registerHandle(jointVelocityHandle);

      // Create Effort joint interface
      JointHandle jointEffortHandle(jointStateHandle, &joint_effort_cmd_[i]);
      effort_interface_.registerHandle(jointEffortHandle);

      JointLimits limits;
      // SoftJointLimits softLimits;

      getJointLimits(joint_names_[i], nh_, limits);
      // getSoftJointLimits(joint_names_[i], nh_, softLimits);

      // joint_limits_interface::PositionJointSoftLimitsHandle
      // jointLimitsHandle(
      // //     jointEffortHandle, limits, softLimits);
      // joint_limits_interface::EffortJointSoftLimitsHandle jointLimitsHandle(
      //     jointEffortHandle, limits, softLimits);

      joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle(
          jointEffortHandle, limits);

      // position_limits_interface_.registerHandle(jointLimitsHandle);
      effort_limits_interface_.registerHandle(jointLimitsHandle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_interface_);
    registerInterface(&velocity_interface_);
    registerInterface(&effort_interface_);
    registerInterface(&effort_limits_interface_);

    now_timestamp_ = ros::Time::now();
    prev_timestamp_ = ros::Time::now();

    mode_changed_ = false;
    mode_ = MODE_MANUAL;

    ROS_INFO_STREAM("OpenRST controller succesfully loaded");

    // daq_in_.reset(new USB1608(nh_, "01F92A95"));
    // daq_out_.reset(new USB3104(nh_, "01F7E9D2"));
    // daq_enc_.reset(new USBQUAD08(nh_, "1001633"));

    // daq_in_->InitAI();
    // daq_in_->InitDIO(4, 4);
    // daq_out_->InitAO();
    // daq_enc_->InitENC();

    ai_state_.resize(ai_channels_);
    di_state_.resize(di_channels_);
    ao_state_.resize(ao_channels_);
    enc_state_.resize(enc_channels_);
    pub_ao_cmd_msg_.data.resize(ao_channels_);
    pub_do_cmd_msg_.data.resize(do_channels_);
  }

  OpenRSTControl::~OpenRSTControl() {}

  bool OpenRSTControl::IsModeChanged()
  {
    int new_mode = MODE_MANUAL;
    if (new_mode != mode_)
    {
      mode_ = new_mode;
      return true;
    }

    return false;
  }

  //* Callbacks
  // Subscribers
  void OpenRSTControl::SubUpdateSimJointStateCb(
      const sensor_msgs::JointState::ConstPtr &msg)
  {
    std::unique_lock<std::mutex> lockAct(m_mtxAct);
    sim_joint_pos_ = VectorXd::Map(&msg->position[0], n_dof_);
    return;
  }

  void OpenRSTControl::SubUpdateJointCommandCb(
      const std_msgs::Float64MultiArray::ConstPtr &msg)
  {
    des_joint_pos_ = VectorXd::Map(&msg->data[0], msg->data.size());
    return;
  }

  void OpenRSTControl::SubUpdateAiStateCb(
      const std_msgs::Float64MultiArray::ConstPtr &msg)
  {
    ai_state_ = msg->data;
  }

  void OpenRSTControl::SubUpdateDiStateCb(
      const std_msgs::Int32MultiArray::ConstPtr &msg)
  {
    di_state_ = msg->data;
  }

  void OpenRSTControl::SubUpdateAoStateCb(
      const std_msgs::Float64MultiArray::ConstPtr &msg)
  {
    ao_state_ = msg->data;
  }

  void OpenRSTControl::SubUpdateEncStateCb(
      const std_msgs::Int32MultiArray::ConstPtr &msg)
  {
    enc_state_ = msg->data;

    daq_joint_pos_[0] =
        enc_coeff_[0] * (enc_state_[kEncIndexMotor0] - enc_offset_[0]);
    daq_joint_pos_[1] =
        -1.0 * enc_coeff_[1] * (enc_state_[kEncIndexMotor1] - enc_offset_[1]);
    daq_joint_pos_[2] =
        -1.0 * enc_coeff_[2] * (enc_state_[kEncIndexMotor2] - enc_offset_[2]);
  }

  // Service Server
  bool OpenRSTControl::SrvOpenRSTCommandCb(
      openrst_control::openrst_request::Request &request,
      openrst_control::openrst_request::Response &response)
  {
    bool res;
    ROS_INFO_STREAM("OpenRST Command Called with request: " << request.message);
    if (request.message == "connect")
      res = connect();
    else if (request.message == "disconnect")
      res = disconnect();
    else if (request.message == "calibrate")
      res = calibrate();
    else if (request.message == "pos_control_on")
      res = posControlOn();
    else if (request.message == "torque_control_on")
      res = torqueControlOn();
    else if (request.message == "control_off")
      res = controlOff();
    else
      return false;

    response.succeeded = res;
    return true;
  }

  int OpenRSTControl::ControlLoop()
  {
    controller_manager::ControllerManager cm(this);

    ROS_INFO_STREAM("Starting control loop");
    rt_clock_.Init();

    while (not(*kill_this_node_))
    {
      now_timestamp_ = ros::Time::now();
      period_ = now_timestamp_ - prev_timestamp_;
      prev_timestamp_ = now_timestamp_;

      if (IsModeChanged())
      {
        disconnect();
      }
      if (mode_ == MODE_AUTO && state_ == F_UNINITIALIZED)
      {
        connect();
      }
      else if (mode_ == MODE_AUTO && state_ == F_CONNECTED)
      {
        calibrate();
      }
      else if (state_ >= F_READY)
      {
        Read();
        cm.update(now_timestamp_, period_);
        if (state_ > F_READY)
          Write(now_timestamp_, period_);
      }

      rt_clock_.SleepToCompleteCycle();
      PublishRobotState();
      // End while not kill this node
    }

    disconnect();

    return 0;
  }

  // Ros Control functions
  void OpenRSTControl::Read()
  {
    if (!use_sim_)
    {
      act_joint_pos_ = daq_joint_pos_;
    }
    else
    {
      act_joint_pos_ = sim_joint_pos_;
    }
    joint_position_ = act_joint_pos_;
  }

  void OpenRSTControl::Write(const ros::Time time, const ros::Duration period)
  {
    effort_limits_interface_.enforceLimits(period);
    des_joint_eff_ = joint_effort_cmd_;
    if (!use_sim_)
    {
      SendCmdRobot();
      SendCmdSim();
    }
    else
    {
      SendCmdSim();
    }
  }

  //* Robot Command Functions
  bool OpenRSTControl::connect()
  {
    if (!use_sim_)
    {
      ROS_INFO_STREAM("OpenRSTControl::ConnectCallback called " << state_);
      act_joint_pos_ = daq_joint_pos_;
      joint_effort_cmd_ = VectorXd::Zero(n_dof_);
    }
    else
    {
      act_joint_pos_ = sim_joint_pos_;
    }

    des_joint_pos_ = act_joint_pos_;

    ROS_INFO_STREAM("OpenRST State change to: CONNECTED");
    ROS_INFO_STREAM(
        "Current openrst joint positions are: " << act_joint_pos_.transpose());

    state_ = F_CONNECTED;
    return true;
  }

  bool OpenRSTControl::calibrate()
  {
    ROS_INFO("Calibration started");
    if (!use_sim_)
    {
      if (state_ == F_CONNECTED)
      {
        bool flag_m0_engaged = false;
        bool flag_m1_engaged = false;
        bool flag_m2_engaged = false;

        bool flag_m0_high_calibrated = false;
        bool flag_m0_low_calibrated = false;
        bool flag_m1_high_calibrated = false;
        bool flag_m1_low_calibrated = false;
        bool flag_m2_high_calibrated = false;
        bool flag_m2_low_calibrated = false;

        bool flag_m0_calibrated = false;
        bool flag_m1_calibrated = false;
        bool flag_m2_calibrated = false;

        int m0_enc_high = 0;
        int m0_enc_low = 0;
        int m1_enc_high = 0;
        int m1_enc_low = 0;
        int m2_enc_high = 0;
        int m2_enc_low = 0;

        bool flag_initialization = false;

        bool flag_all_engaged = false;

        double engaged_sensor0 = 0;
        double engaged_sensor1 = 0;
        double engaged_sensor2 = 0;

        int enc_motor0 = 0;
        int enc_motor1 = 0;
        int enc_motor2 = 0;

        int enc_motor0_km1 = 0;
        int enc_motor1_km1 = 0;
        int enc_motor2_km1 = 0;

        double m0_max_calib_current = 0.10;
        double m1_max_calib_current = 0.16;
        double m2_max_calib_current = 0.16;

        double m0_start_calib_current = 0.05;
        double m1_start_calib_current = 0.05;
        double m2_start_calib_current = 0.05;

        double m0_calib_current = m0_start_calib_current;
        double m1_calib_current = m1_start_calib_current;
        double m2_calib_current = m2_start_calib_current;

        int wait_counter = 0;

        int step_jaw_calibration = CALIB_READY;

        ros::Rate calibration_rate(100);

        ROS_INFO("Initializing OpenRST  ....");

        while (!flag_initialization)
        {
          engaged_sensor0 = ai_state_[kAiIndexSensor0];
          engaged_sensor1 = ai_state_[kAiIndexSensor1];
          engaged_sensor2 = ai_state_[kAiIndexSensor2];

          enc_motor0 = enc_state_[kEncIndexMotor0];
          enc_motor1 = enc_state_[kEncIndexMotor1];
          enc_motor2 = enc_state_[kEncIndexMotor2];

          // ROS_INFO("Sensor0: %f, Sensor1: %f, Sensor2: %f", engaged_sensor0,
          //          engaged_sensor1, engaged_sensor2);

          // ROS_INFO("Encoder0: %d, Encoder1: %d, Encoder2: %d", enc_motor0,
          //          enc_motor1, enc_motor2);

          if (!flag_all_engaged)
          {
            //* MOTOR 0 - PITCH
            if (!flag_m0_engaged && engaged_sensor0 < kSensorTresh0)
            {
              des_joint_eff_[0] = -kCalibMotorCurr * kMotorCurrToControlSignal;
              ROS_INFO_STREAM_ONCE("CMD M0: " << des_joint_eff_[0]);
            }
            else if (!flag_m0_engaged && engaged_sensor0 >= kSensorTresh0)
            {
              des_joint_eff_[0] = 0.0;
              ROS_INFO_STREAM_ONCE("Motor0 engaged");
              flag_m0_engaged = true;
            }

            //* MOTOR 1 - RIGHT JAW
            if (!flag_m1_engaged && engaged_sensor1 < kSensorTresh1)
            {
              des_joint_eff_[1] = -kCalibMotorCurr * kMotorCurrToControlSignal;
              ROS_INFO_STREAM_ONCE("CMD M1: " << des_joint_eff_[1]);
            }
            else if (!flag_m1_engaged && engaged_sensor1 >= kSensorTresh1)
            {
              des_joint_eff_[1] = 0.0;
              ROS_INFO_STREAM_ONCE("Motor1 engaged");
              flag_m1_engaged = true;
            }

            //* MOTOR 2 - LEFT JAW
            if (!flag_m2_engaged && engaged_sensor2 < kSensorTresh2)
            {
              des_joint_eff_[2] = -kCalibMotorCurr * kMotorCurrToControlSignal;
              ROS_INFO_STREAM_ONCE("CMD M2: " << des_joint_eff_[2]);
            }
            else if (!flag_m2_engaged && engaged_sensor2 >= kSensorTresh2)
            {
              des_joint_eff_[2] = 0.0;
              ROS_INFO_STREAM_ONCE("Motor2 engaged");
              flag_m2_engaged = true;
            }

            if (flag_m0_engaged && flag_m1_engaged && flag_m2_engaged)
            {
              ROS_INFO_STREAM("All motors engaged");
              des_joint_eff_[0] = 0.0;
              des_joint_eff_[1] = 0.0;
              des_joint_eff_[2] = 0.0;

              flag_all_engaged = true;
              des_joint_eff_ = VectorXd::Zero(n_dof_);
            }
          }
          else if (flag_all_engaged)
          {
            //* MOTOR 0 - PITCH
            if (step_jaw_calibration <= SET_M0_ZERO)
            {

              if (enc_motor0_km1 == enc_motor0)
              {
                if (m0_calib_current > m0_max_calib_current)
                {
                  wait_counter += 1;
                  if (wait_counter > 100)
                  {
                    wait_counter = 0;
                    m0_calib_current = m0_start_calib_current;
                    if (step_jaw_calibration == CALIB_M0_HIGH)
                    {
                      m0_enc_high = enc_motor0;
                      flag_m0_high_calibrated = true;
                      step_jaw_calibration = CALIB_M0_LOW;
                    }
                    else if (step_jaw_calibration == CALIB_M0_LOW)
                    {
                      m0_enc_low = enc_motor0;
                      flag_m0_low_calibrated = true;
                      step_jaw_calibration = SET_M0_ZERO;
                    }
                  }
                }
                else
                {
                  m0_calib_current += 0.001;
                }
              }
              else
              {
                enc_motor0_km1 = enc_motor0;
              }
            }
            else if (step_jaw_calibration == CALIB_M1_HIGH ||
                     step_jaw_calibration == CALIB_M1_LOW)
            {
              if (enc_motor1_km1 == enc_motor1)
              {
                if (m1_calib_current > m1_max_calib_current)
                {
                  wait_counter += 1;
                  if (wait_counter > 100)
                  {
                    wait_counter = 0;
                    m1_calib_current = m1_start_calib_current;
                    if (step_jaw_calibration == CALIB_M1_HIGH)
                    {
                      m1_enc_high = enc_motor1;
                      flag_m1_high_calibrated = true;
                      step_jaw_calibration = CALIB_M2_HIGH;
                    }
                    else if (step_jaw_calibration == CALIB_M1_LOW)
                    {
                      m1_enc_low = enc_motor1;
                      flag_m1_low_calibrated = true;
                      step_jaw_calibration = SET_M1_HIGH;
                    }
                  }
                }
                else
                {
                  m1_calib_current += 0.001;
                }
              }
              else
              {
                enc_motor1_km1 = enc_motor1;
              }
            }
            else if (step_jaw_calibration == CALIB_M2_HIGH ||
                     step_jaw_calibration == CALIB_M2_LOW)
            {
              if (enc_motor2_km1 == enc_motor2)
              {
                if (m2_calib_current > m2_max_calib_current)
                {
                  wait_counter += 1;
                  if (wait_counter > 100)
                  {
                    wait_counter = 0;
                    m2_calib_current = m2_start_calib_current;
                    if (step_jaw_calibration == CALIB_M2_HIGH)
                    {
                      m2_enc_high = enc_motor2;
                      flag_m2_high_calibrated = true;
                      step_jaw_calibration = CALIB_M1_LOW;
                    }
                    else if (step_jaw_calibration == CALIB_M2_LOW)
                    {
                      m2_enc_low = enc_motor2;
                      flag_m2_low_calibrated = true;
                      step_jaw_calibration = SET_M2_ZERO;
                    }
                  }
                }
                else
                {
                  m2_calib_current += 0.001;
                }
              }
              else
              {
                enc_motor2_km1 = enc_motor2;
              }
            }

            if (step_jaw_calibration == CALIB_READY)
            {
              step_jaw_calibration = CALIB_M0_HIGH;
            }

            if (step_jaw_calibration == CALIB_M0_HIGH)
            {
              ROS_INFO("Calibrating Motor0 Upper limit with current: %3.3f [A]",
                       m0_calib_current);
              des_joint_eff_[0] = -m0_calib_current * kMotorCurrToControlSignal;
              des_joint_eff_[1] = 0.0;
              des_joint_eff_[2] = 0.0;
            }
            else if (step_jaw_calibration == CALIB_M0_LOW)
            {
              ROS_INFO("Calibrating Motor0 Lower limit with current: %3.3f [A]",
                       m0_calib_current);

              des_joint_eff_[0] = m0_calib_current * kMotorCurrToControlSignal;
              des_joint_eff_[1] = 0.0;
              des_joint_eff_[2] = 0.0;
            }
            else if (step_jaw_calibration == SET_M0_ZERO)
            {
              double m0_target = (m0_enc_high + m0_enc_low) / 2.0;
              des_joint_eff_[1] = 0.0;
              des_joint_eff_[2] = 0.0;

              ROS_INFO_ONCE("Motor 0 calibrated. Low: %d High: %d", m0_enc_high,
                            m0_enc_low);
              ROS_INFO_ONCE("Motor 0 Target: %f [count for 0deg]", m0_target);

              if (abs(m0_target - (double)enc_motor0) > 10.0)
              {
                des_joint_eff_[0] =
                    ((double)enc_motor0 - m0_target) * kMotorKp0;
                if (des_joint_eff_[0] > 7.0)
                  des_joint_eff_[0] = 7.0;
                if (des_joint_eff_[0] < -7.0)
                  des_joint_eff_[0] = -7.0;
              }
              else
              {
                flag_m0_calibrated = true;
                step_jaw_calibration = CALIB_M1_HIGH;
              }
            }
            else if (step_jaw_calibration == CALIB_M1_HIGH)
            {
              ROS_INFO("Calibrating Motor1 Upper limit with current: %f [A]",
                       m1_calib_current);
              des_joint_eff_[0] = 0.0;
              des_joint_eff_[1] = -m1_calib_current * kMotorCurrToControlSignal;
              des_joint_eff_[2] = 0.0;
            }
            else if (step_jaw_calibration == CALIB_M2_HIGH)
            {
              ROS_INFO("Calibrating Motor2 Upper limit with current: %f [A]",
                       m2_calib_current);
              des_joint_eff_[0] = 0.0;
              des_joint_eff_[1] = 0.0;
              des_joint_eff_[2] = -m2_calib_current * kMotorCurrToControlSignal;
            }
            else if (step_jaw_calibration == CALIB_M1_LOW)
            {
              ROS_INFO("Calibrating Motor1 Lower limit with current: %f [A]",
                       m1_calib_current);
              des_joint_eff_[0] = 0.0;
              des_joint_eff_[1] = m1_calib_current * kMotorCurrToControlSignal;
              des_joint_eff_[2] = 0.0;
            }
            else if (step_jaw_calibration == SET_M1_HIGH)
            {
              double m1_target = m1_enc_high;
              des_joint_eff_[0] = 0.0;
              des_joint_eff_[2] = 0.0;

              ROS_INFO_ONCE("Motor1 Target: %f [count for -90deg]", m1_target);

              if (abs(m1_target - (double)enc_motor1) > 200.0)
              {
                des_joint_eff_[1] =
                    ((double)enc_motor1 - m1_target) * kMotorKp1;
                if (des_joint_eff_[1] > 6.0)
                  des_joint_eff_[1] = 6.0;
                if (des_joint_eff_[1] < -6.0)
                  des_joint_eff_[1] = -6.0;
              }
              else
              {
                des_joint_eff_[1] = 0.0;
                step_jaw_calibration = CALIB_M2_LOW;
              }
            }
            else if (step_jaw_calibration == CALIB_M2_LOW)
            {
              ROS_INFO("Calibrating Motor2 Lower limit with current: %f [A]",
                       m2_calib_current);
              des_joint_eff_[0] = 0.0;
              des_joint_eff_[1] = 0.0;
              des_joint_eff_[2] = m2_calib_current * kMotorCurrToControlSignal;
            }
            else if (step_jaw_calibration == SET_M2_ZERO)
            {
              double m2_target = (m2_enc_high + m2_enc_low) / 2.0;
              des_joint_eff_[0] = 0.0;
              des_joint_eff_[1] = 0.0;

              ROS_INFO_ONCE("Motor 2 calibrated. Low: %d High: %d", m2_enc_high,
                            m2_enc_low);
              ROS_INFO_ONCE("Motor2 Target: %f [count for 0deg]", m2_target);

              if (abs(m2_target - (double)enc_motor2) > 100.0)
              {
                des_joint_eff_[2] =
                    ((double)enc_motor2 - m2_target) * kMotorKp2;
                if (des_joint_eff_[2] > 6.0)
                  des_joint_eff_[2] = 6.0;
                if (des_joint_eff_[2] < -6.0)
                  des_joint_eff_[2] = -6.0;
              }
              else
              {
                flag_m2_calibrated = true;
                des_joint_eff_[2] = 0.0;
                step_jaw_calibration = SET_M1_ZERO;
              }
            }
            else if (step_jaw_calibration == SET_M1_ZERO)
            {
              double m1_target = (m1_enc_high + m1_enc_low) / 2.0;
              des_joint_eff_[0] = 0.0;
              des_joint_eff_[2] = 0.0;
              ROS_INFO_ONCE("Motor 1 calibrated. Low: %d High: %d", m1_enc_high,
                            m1_enc_low);
              ROS_INFO_ONCE("Motor1 Target: %f [count for 0deg]", m1_target);

              if (abs(m1_target - (double)enc_motor1) > 100.0)
              {
                des_joint_eff_[1] =
                    ((double)enc_motor1 - m1_target) * kMotorKp1;
                if (des_joint_eff_[1] > 6.0)
                  des_joint_eff_[1] = 6.0;
                if (des_joint_eff_[1] < -6.0)
                  des_joint_eff_[1] = -6.0;
              }
              else
              {
                flag_m1_calibrated = true;
                des_joint_eff_[1] = 0.0;
                step_jaw_calibration = CALIB_COMPLETED;
              }
            }

            if (flag_m0_calibrated && flag_m1_calibrated && flag_m2_calibrated)
            {
              des_joint_eff_[0] = 0.0;
              des_joint_eff_[1] = 0.0;
              des_joint_eff_[2] = 0.0;
              des_joint_eff_ = VectorXd::Zero(n_dof_);
              flag_initialization = true;
              ROS_INFO("\n Calibration Finished: Summary");
              ROS_INFO("--------------------------------------");
              ROS_INFO("\tMotor0 Low: %d High: %d", m0_enc_low, m0_enc_high);
              ROS_INFO("\tMotor1 Low: %d High: %d", m1_enc_low, m1_enc_high);
              ROS_INFO("\tMotor2 Low: %d High: %d", m2_enc_low, m2_enc_high);

              enc_range_[0] = m0_enc_high - m0_enc_low;
              enc_range_[1] = m1_enc_high - m1_enc_low;
              enc_range_[2] = m2_enc_high - m2_enc_low;

              ROS_INFO("enc_range: %f %f %f", enc_range_[0], enc_range_[1],
                       enc_range_[2]);

              enc_coeff_[0] = enc_joint_range_[0] / enc_range_[0];
              enc_coeff_[1] = enc_joint_range_[1] / enc_range_[1];
              enc_coeff_[2] = enc_joint_range_[2] / enc_range_[2];
              ROS_INFO("enc_coeff: %f %f %f", enc_coeff_[0], enc_coeff_[1],
                       enc_coeff_[2]);

              enc_offset_[0] = (m0_enc_low + m0_enc_high) / 2.0;
              enc_offset_[1] = (m1_enc_low + m1_enc_high) / 2.0;
              enc_offset_[2] = (m2_enc_low + m2_enc_high) / 2.0;
              ROS_INFO("enc_offset: %f %f %f", enc_offset_[0], enc_offset_[1],
                       enc_offset_[2]);

              daq_joint_pos_[0] = enc_coeff_[0] * (enc_state_[kEncIndexMotor0] -
                                                   enc_offset_[0]);
              daq_joint_pos_[1] =
                  -enc_coeff_[1] *
                  (enc_state_[kEncIndexMotor1] - enc_offset_[1]);
              daq_joint_pos_[2] =
                  -enc_coeff_[2] *
                  (enc_state_[kEncIndexMotor2] - enc_offset_[2]);
            }
          }

          ao_cmd_[kAoIndexMotor0] = des_joint_eff_[0];
          ao_cmd_[kAoIndexMotor1] = des_joint_eff_[1];
          ao_cmd_[kAoIndexMotor2] = des_joint_eff_[2];
          SendCmdRobot();

          calibration_rate.sleep();
        }
      }
    }
    state_ = F_READY;
    ROS_INFO_STREAM("OpenRST State change to: READY");
    return true;
  } // namespace openrst_nu

  bool OpenRSTControl::posControlOn()
  {
    if (!use_sim_)
    {
      if (state_ == F_READY)
      {
      }
      else
      {
        ROS_ERROR_STREAM("OpenRST are not READY");
      }
    }
    state_ = F_POSITION_CONTROL;
    ROS_INFO_STREAM("OpenRST State change to: POSITION SLAVE");
    return true;
  }

  bool OpenRSTControl::torqueControlOn()
  {
    if (!use_sim_)
    {
      if (state_ == F_READY)
      {
        // TODO
      }
      else
      {
        ROS_ERROR_STREAM("OpenRST are not READY");
      }
    }
    state_ = F_TORQUE_CONTROL;
    ROS_INFO_STREAM("OpenRST State change to: F_TORQUE_CONTROL");
    return true;
  }

  bool OpenRSTControl::controlOff()
  {
    if (!use_sim_)
    {
      if (state_ > F_READY)
      {
        ROS_INFO_STREAM("Stopping OpenRST Motion Control");
        des_joint_eff_ = VectorXd::Zero(n_dof_);
        SendCmdRobot();
      }
    }
    state_ = F_CONNECTED;
    ROS_INFO_STREAM("Robot State change to: F_READY");
    return true;
  }

  bool OpenRSTControl::disconnect()
  {
    if (!use_sim_)
    {
      ROS_INFO_STREAM("OpenRST Control");
      if (state_ > F_READY)
      {
        controlOff();
      }
    }
    ROS_INFO_STREAM("Robot State change to: UNINITIALIZED");
    state_ = F_UNINITIALIZED;

    return true;
  }

  void OpenRSTControl::SafeSleepSeconds(double seconds)
  {
    int n_cycles = int(1000000.0 * seconds / cycle_t_us_);
    for (int i = 0; i < n_cycles && not(*kill_this_node_); i++)
    {
      rt_clock_.SleepToCompleteCycle();
    }
  }

  void OpenRSTControl::BlockSleepSeconds(double seconds)
  {
    int n_cycles = int(1000000.0 * seconds / cycle_t_us_);
    for (int i = 0; i < n_cycles; i++)
    {
      rt_clock_.SleepToCompleteCycle();
    }
  }

  //* Auxiliar Functions

  void OpenRSTControl::SendCmdRobot()
  {
    for (int joint = 0; joint < n_dof_; joint++)
    {
      if (des_joint_eff_[joint] > 10)
      {
        des_joint_eff_[joint] = 10.0;
      }
      else if (des_joint_eff_[joint] < -10)
      {
        des_joint_eff_[joint] = -10.0;
      }
    }
    ao_cmd_[kAoIndexMotor0] = des_joint_eff_[0];
    ao_cmd_[kAoIndexMotor1] = des_joint_eff_[1];
    ao_cmd_[kAoIndexMotor2] = des_joint_eff_[2];
    // ROS_INFO("Sending CMD: %0.3f %0.3f %0.3f", ao_cmd_[kAoIndexMotor0],
    //          ao_cmd_[kAoIndexMotor1], ao_cmd_[kAoIndexMotor2]);
    pub_ao_cmd_msg_.data = ao_cmd_;
    pub_ao_cmd_.publish(pub_ao_cmd_msg_);
  }

  void OpenRSTControl::PublishRobotState()
  {
    pub_state_msg_.data[0] = state_;
    pub_state_msg_.data[1] = state_;
    pub_state_.publish(pub_state_msg_);
  }

  void OpenRSTControl::SendCmdSim()
  {
    pub_sim_joint_cmd_msg_.header.stamp = ros::Time::now();
    VectorXd::Map(&pub_sim_joint_cmd_msg_.position[0],
                  pub_sim_joint_cmd_msg_.position.size()) = des_joint_pos_;
    pub_sim_joint_cmd_.publish(pub_sim_joint_cmd_msg_);
  }

} // namespace openrst_nu
