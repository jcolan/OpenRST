// C
#include <stdio.h>
#include <stdlib.h>

// C++

// External
#include "uldaq.h"
#include <ul_lib/utility.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

// Internal
#include <math_utils/filters.h>

class USB1608
{
  const int kNumberOfAiChannels = 16;
  const int kNumberOfDioPorts = 1;
  const int kNumberOfDioIO = 8;
  const int kNumberOfCntChannels = 2;
  const int kNumberOfTimers = 1;
  const int kMaxDevCount = 10;
  const int kMaxStrlength = 64;

  public:
  USB1608();
  USB1608(ros::NodeHandle nh, std::string daq_id);
  ~USB1608();

  int InitDIO(int n_din, int n_dout);
  int InitDIO();
  int InitAI();

  int UpdateStateDI(int bit_number);
  int UpdateStateDI();
  int PublishStateDI();

  int  UpdateStateDO(int bit_number, unsigned int bit_value);
  int  UpdateStateDO();
  int  UpdateStateAI();
  int  PublishStateAI();
  void Quit();

  int StartScanAI();
  int UpdateScanStateAI();
  int StopScanAI();

  std::vector<double> GetScanStateChannelsAI(int start_ch, int n_channels);

  void PrintError(UlError err_);

  bool IsEnabledDIO() { return dio_enabled_; }
  bool IsEnabledAI() { return ai_enabled_; }

  std::vector<double> get_ai_state() { return ai_state_; }
  std::vector<int>    get_di_state() { return di_state_; }

  int get_ai_channels();
  int get_di_channels();
  int get_do_channels();

  // Callbacks
  void UpdateDOValueCb(const std_msgs::Int32MultiArray::ConstPtr &msg);

  private:
  // ROS
  ros::NodeHandle nh_;

  // ROS Topics
  ros::Publisher              pub_ai_state_;
  ros::Publisher              pub_ai_state_filtered;
  ros::Publisher              pub_di_state_;
  ros::Subscriber             sub_do_cmd_;
  std_msgs::Float64MultiArray pub_ai_state_msg_;
  std_msgs::Float64MultiArray pub_ai_state_filtered_msg_;
  std_msgs::Int32MultiArray   pub_di_state_msg_;

  filters::IIR *filter_iir_ai;
  VectorXd      ai_state_filtered_;

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

  // Analog Input vars
  AiInputMode         inputMode_;
  Range               ai_range_;
  int                 numberOfChannels_;
  int                 samplesPerChannel_;
  std::vector<double> ai_state_;
  bool                ai_enabled_;
  double *            ai_buffer_ = NULL;
  double              ai_rate_;
  ScanOption          ai_scanOptions_;
  AInScanFlag         flags_ai_scan_;
  AInFlag             flags_;

  // Digital Port vars
  DigitalPortType   portType_;
  DigitalPortIoType portIoType_;

  int              bitsPerPort_;
  int              n_dinput_;
  int              n_doutput_;
  std::vector<int> di_state_;
  std::vector<int> do_cmd_;
  bool             dio_enabled_;
};