// C
#include <stdio.h>
#include <stdlib.h>

// C++

// External
#include "uldaq.h"
#include <ul_lib/utility.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

// Internal

class USB3104
{
  const int kNumberOfAoChannels = 8;

  public:
  USB3104();
  USB3104(ros::NodeHandle nh, std::string daq_id);
  ~USB3104();

  int InitAO();
  int UpdateStateAO();
  int UpdateStateChannelAO(int channel);

  int  PublishStateAO();
  void Quit();

  void PrintError(UlError err_);

  std::vector<double> GetAoState();

  bool IsEnabledAO();
  // Accessors
  int get_ao_channels();
  // Mutators
  void set_ao_cmd(std::vector<double> ao_cmd);
  // Callbacks
  void UpdateAOValueCb(const std_msgs::Float64MultiArray::ConstPtr &msg);

  private:
  // ROS
  ros::NodeHandle nh_;

  // ROS Topics
  ros::Publisher              pub_ao_state_;
  ros::Subscriber             sub_ao_cmd_;
  std_msgs::Float64MultiArray pub_ao_state_msg_;

  // Device
  int                 descriptorIndex_;
  DaqDeviceDescriptor devDescriptor_;
  DaqDeviceInterface  interfaceType_;
  DaqDeviceHandle     daqDeviceHandle_;
  unsigned int        numDevs_;
  UlError             err_;

  std::string daq_id_;
  bool        daq_ready_;
  bool        daq_connected_;

  // Analog Output vars
  Range               range_;
  AOutFlag            flags_;
  int                 numberOfChannels_;
  std::vector<double> ao_state_;
  std::vector<double> ao_cmd_;
  bool                ao_enabled_;
};