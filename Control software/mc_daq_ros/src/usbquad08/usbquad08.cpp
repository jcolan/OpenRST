#include <stdio.h>
#include <stdlib.h>
#include "uldaq.h"
#include <ul_lib/utility.h>

#define MAX_DEV_COUNT 100
#define MAX_SCAN_OPTIONS_LENGTH 256
#define MAX_ENCODER_COUNTERS 16

#include <usbquad08/usbquad08.h>

USBQUAD08::USBQUAD08(ros::NodeHandle nh, std::string daq_id)
    : nh_(nh), daq_id_(daq_id)
{
  daq_ready_ = false;
  daq_connected_ = false;

  descriptorIndex_ = 0;
  interfaceType_ = USB_IFC;
  daqDeviceHandle_ = 0;
  numDevs_ = MAX_DEV_COUNT;

  type_ = CMT_ENCODER;
  mode_ = (CounterMeasurementMode)(CMM_ENCODER_X4 | CMM_ENCODER_CLEAR_ON_Z);
  edgeDetection_ = CED_RISING_EDGE;
  tickSize_ = CTS_TICK_20ns;
  debounceMode_ = CDM_NONE;
  debounceTime_ = CDT_DEBOUNCE_0ns;
  configFlags_ = CF_DEFAULT;
  flags_ = CINSCAN_FF_DEFAULT;

  samplesPerCounter_ = 10;
  rate_ = 10000;
  scanOptions_ = (ScanOption)(SO_DEFAULTIO | SO_CONTINUOUS);

  enc_state_.resize(kNumberOfEncChannels);

  offset_ = 0;

  numberOfEncoders_ = kNumberOfEncChannels;

  int i = 0;
  err_ = ERR_NO_ERROR;
  DaqDeviceDescriptor devDescriptors[MAX_DEV_COUNT];

  err_ = ulGetDaqDeviceInventory(interfaceType_, devDescriptors, &numDevs_);
  if (err_ != ERR_NO_ERROR)
  {
    ROS_ERROR_STREAM("Error getting DAQ boards information:" << err_ << "\n");
    return;
  }

  // verify at least one DAQ device is detected
  if (numDevs_ == 0)
  {
    ROS_ERROR_STREAM("No DAQ device is detected\n");
    return;
  }

  for (i = 0; i < (int)numDevs_; i++)
  {
    if (devDescriptors[i].uniqueId == daq_id_)
    {
      devDescriptor_ = devDescriptors[i];
      break;
    }

    else if (i == (int)numDevs_)
    {
      ROS_ERROR_STREAM("DAQ Board with ID " << daq_id_ << "not found");
      return;
    }
  }

  // get a handle to the DAQ device associated with the first descriptor
  ROS_INFO_STREAM("Creating DAQ Handle for ID " << daq_id_);
  daqDeviceHandle_ = ulCreateDaqDevice(devDescriptor_);
  if (daqDeviceHandle_ == 0)
  {
    ROS_ERROR_STREAM("Unable to create a handle for " << daq_id_
                                                      << " DAQ device");
    return;
  }

  daq_ready_ = true;

  // Publishers
  pub_enc_state_ =
      nh_.advertise<std_msgs::Int32MultiArray>("/usbquad08/enc/state", 1);

  pub_enc_state_msg_.data.resize(kNumberOfEncChannels);

  // Services
  srv_server_daq_cmd =
      nh_.advertiseService("daq_command", &USBQUAD08 ::SrvDaqCommandCb, this);
}

USBQUAD08::~USBQUAD08() {}

bool USBQUAD08::SrvDaqCommandCb(mc_daq_ros::daq_cmd::Request &req,
                                mc_daq_ros::daq_cmd::Response &res)
{
  bool result = false;
  ROS_INFO_STREAM("DAQ Command Called with request: " << req.message);
  if (req.message == "reset_enc")
  {
    SetZero(req.port);
    return true;
  }
  else if (req.message == "reset_all_enc")
  {
    SetAllZero();
    return true;
  }

  return result;
}

int USBQUAD08::InitENC()
{
  int hasCI = 0;
  int hasPacer = 0;
  int encoderCounters[MAX_ENCODER_COUNTERS];
  char scanOptionsStr[MAX_SCAN_OPTIONS_LENGTH];

  if (!daq_ready_)
  {
    ROS_ERROR_STREAM("DAQ board is not ready");
    return -1;
  }

  // verify the specified DAQ device supports counter
  err_ = getDevInfoHasCtr(daqDeviceHandle_, &hasCI);
  if (!hasCI)
  {
    ROS_ERROR_STREAM("The specified DAQ device does not support counter input");
    return -1;
  }

  // verify the specified DAQ device supports hardware pacing for counters
  err_ = getCtrInfoHasPacer(daqDeviceHandle_, &hasPacer);
  if (!hasPacer)
  {
    ROS_ERROR_STREAM("The specified DAQ device does not support hardware paced "
                     "counter input");
    return -1;
  }

  ROS_WARN("Connecting to device %s - please wait ...",
           devDescriptor_.devString);

  // establish a connection to the DAQ device
  int attempts = 10;
  for (int trial = 0; trial < attempts; trial++)
  {
    err_ = ulConnectDaqDevice(daqDeviceHandle_);
    if (err_ == ERR_NO_ERROR)
    {
      ROS_INFO("Connected to device %s", devDescriptor_.devString);
      break;
    }
    else
    {
      ROS_WARN("Connection to device %s failed, attempt %d of %d",
               devDescriptor_.devString, trial + 1, attempts);
      if (trial == attempts - 1)
      {
        ROS_ERROR("Connection to device %s failed, exiting",
                  devDescriptor_.devString);
        return -1;
      }
      sleep(1);
    }
  }

  daq_connected_ = true;

  // get the counter numbers for the supported encoders
  err_ = getCtrInfoSupportedEncoderCounters(daqDeviceHandle_, encoderCounters,
                                            &numberOfEncoders_);
  if (numberOfEncoders_ == 0)
  {
    ROS_ERROR("\nThe specified DAQ device does not support encoder channels\n");
  }

  // configure the encoders
  for (int i = 0; i < numberOfEncoders_; i++)
  {
    err_ = ulCConfigScan(daqDeviceHandle_, i, type_, mode_, edgeDetection_,
                         tickSize_, debounceMode_, debounceTime_, configFlags_);
  }

  // ConvertScanOptionsToString(scanOptions, scanOptionsStr);

  ROS_INFO("\tENCODER INFO");
  ROS_INFO("\t# Encoders: %d", numberOfEncoders_);

  SetAllZero();
  ROS_INFO("Encoders initialized as Zero");
  enc_enabled_ = true;

  return 0;
}

/*
 * Start scanning encoder channels
 */
int USBQUAD08::StartScanENC()
{
  double data = 0;

  if (!enc_enabled_)
  {
    ROS_ERROR_STREAM("AI Module is not enabled");
    return -1;
  }

  enc_buffer_ = (unsigned long long *)malloc(
      numberOfEncoders_ * samplesPerCounter_ * sizeof(unsigned long long));

  if (enc_buffer_ == NULL)
  {
    printf("\nOut of memory, unable to create scan buffer\n");
    return -1;
  }

  err_ = ulCInScan(daqDeviceHandle_, 0, numberOfEncoders_, samplesPerCounter_,
                   &rate_, scanOptions_, flags_, enc_buffer_);

  if (err_ == ERR_NO_ERROR)
    return 0;

  return -1;
}

/*
 * Stop scanning encoder channels
 */
int USBQUAD08::StopScanENC()
{
  ScanStatus status;
  TransferStatus transferStatus;

  err_ = ulCInScanStatus(daqDeviceHandle_, &status, &transferStatus);

  double data = 0;
  if (status == SS_RUNNING && err_ == ERR_NO_ERROR)
  {
    err_ = ulCInScanStop(daqDeviceHandle_);
  }

  return 0;
}

int USBQUAD08::SetZero(int enc_id)
{
  ulCClear(daqDeviceHandle_, enc_id);
  return 0;
}
int USBQUAD08::SetAllZero()
{
  for (int chan = 0; chan < numberOfEncoders_; chan++)
  {
    ulCClear(daqDeviceHandle_, chan);
  }
  return 0;
}

void USBQUAD08::set_offset(int value)
{
  offset_ = value;
}

int USBQUAD08::UpdateStateENC()
{
  unsigned long long data = 0;

  if (!enc_enabled_)
  {
    ROS_ERROR_STREAM("ENC Module is not enabled");
    return -1;
  }

  // ROS_INFO("Active DAQ device: %s (%s)", devDescriptor_.productName,
  // devDescriptor_.uniqueId);

  // display data for all the analog input channels
  for (int chan = 0; chan < numberOfEncoders_; chan++)
  {
    err_ = ulCIn(daqDeviceHandle_, chan, &data);

    if (err_ == ERR_NO_ERROR)
      // {
      //   ROS_INFO("Channel(%d) Data: %d", chan, data);
      enc_state_.at(chan) = data;
  }

  return 0;
}

/*
 * Update encoder channels state with current scan values
 */
int USBQUAD08::UpdateScanStateENC()
{
  ScanStatus status;
  TransferStatus transferStatus;

  err_ = ulCInScanStatus(daqDeviceHandle_, &status, &transferStatus);
  int index = transferStatus.currentIndex;
  // printf("actual scan rate = %f\n\n", ai_rate_);

  for (int ch = 0; ch < numberOfEncoders_; ch++)
  {
    if (err_ == ERR_NO_ERROR)
    {
      // ROS_INFO("Channel(%d) Data: %+-10.6f", ch, ai_buffer_[index + ch]);
      enc_state_.at(ch) = enc_buffer_[index + ch];
      if (enc_state_[ch] > 32768)
        enc_state_[ch] -= 65536;
      // enc_state_.at(ch) += offset_;
    }
  }
  return 0;
}

bool USBQUAD08::IsEnabledENC() { return enc_enabled_; }

int USBQUAD08::PublishStateENC()
{
  pub_enc_state_msg_.data = enc_state_;
  // ROS_INFO("Publishing Encoder State: %d %d %d", enc_state_[0], enc_state_[1], enc_state_[2]);

  pub_enc_state_.publish(pub_enc_state_msg_);
  return 0;
}

void USBQUAD08::Quit()
{
  // disconnect from the DAQ device
  ulDisconnectDaqDevice(daqDeviceHandle_);

  // release the handle to the DAQ device
  if (daqDeviceHandle_)
    ulReleaseDaqDevice(daqDeviceHandle_);
}

void USBQUAD08::PrintError(UlError err_)
{
  if (err_ != ERR_NO_ERROR)
  {
    char errMsg[ERR_MSG_LEN];
    ulGetErrMsg(err_, errMsg);
    ROS_ERROR("Error Code: %d \n", err_);
    ROS_ERROR("Error Message: %s \n", errMsg);
  }
}

int USBQUAD08::get_enc_channels() { return 0; }

std::vector<int> USBQUAD08::GetEncState() { return enc_state_; }
