
// C
#include <stdio.h>
#include <stdlib.h>

// C++

// External
#include "uldaq.h"
#include <ul_lib/utility.h>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

// Internal
#include <mc_daq_ros/daq_cmd.h>

class USBQUAD08
{

  const int kNumberOfEncChannels = 8;

public:
  USBQUAD08();
  USBQUAD08(ros::NodeHandle nh, std::string daq_id);
  ~USBQUAD08();

  bool SrvDaqCommandCb(mc_daq_ros::daq_cmd::Request &req,
                       mc_daq_ros::daq_cmd::Response &res);

  int InitENC();
  int SetZero(int enc_id);
  int SetAllZero();

  int UpdateStateENC();
  int PublishStateENC();
  void Quit();

  int StartScanENC();
  int StopScanENC();
  int UpdateScanStateENC();

  std::vector<int> GetEncState();

  void PrintError(UlError err_);

  bool IsEnabledENC();
  int get_enc_channels();

  void set_offset(int value);

private:
  // ROS
  ros::NodeHandle nh_;

  // ROS Topics
  ros::Publisher pub_enc_state_;
  std_msgs::Int32MultiArray pub_enc_state_msg_;

  ros::ServiceServer srv_server_daq_cmd;

  // Device
  int descriptorIndex_;
  DaqDeviceDescriptor devDescriptor_;
  DaqDeviceInterface interfaceType_;
  DaqDeviceHandle daqDeviceHandle_;
  unsigned int numDevs_;
  UlError err_;

  // encoder settings
  CounterMeasurementType type_;
  CounterMeasurementMode mode_;
  CounterEdgeDetection edgeDetection_;
  CounterTickSize tickSize_;
  CounterDebounceMode debounceMode_;
  CounterDebounceTime debounceTime_;
  CConfigScanFlag configFlags_;
  CInScanFlag flags_;

  std::string daq_id_;
  bool daq_ready_;
  bool daq_connected_;

  // Encoder vars
  int numberOfEncoders_;
  std::vector<int> enc_state_;
  bool enc_enabled_;
  unsigned long long *enc_buffer_ = NULL;
  int samplesPerCounter_;
  double rate_;
  ScanOption scanOptions_;

  int offset_;
};