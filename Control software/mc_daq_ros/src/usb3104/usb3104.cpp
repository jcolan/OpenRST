#include <usb3104/usb3104.h>

#define MAX_DEV_COUNT 10
#define MAX_STR_LENGTH 64

USB3104::USB3104(ros::NodeHandle nh, std::string daq_id)
    : nh_(nh), daq_id_(daq_id)
{
  daq_ready_ = false;
  daq_connected_ = false;

  descriptorIndex_ = 0;
  interfaceType_ = USB_IFC;
  daqDeviceHandle_ = 0;
  numDevs_ = MAX_DEV_COUNT;

  numberOfChannels_ = kNumberOfAoChannels;

  ao_state_.resize(numberOfChannels_);
  flags_ = AOUT_FF_DEFAULT;
  // flags_array_ = AOUTARRAY_FF_DEFAULT;

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
      ROS_ERROR_STREAM("DAQ Board with ID " << daq_id_ << " not found");
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
  pub_ao_state_ =
      nh_.advertise<std_msgs::Float64MultiArray>("/usb3104/ao/state", 1);

  // Subscribers
  sub_ao_cmd_ =
      nh_.subscribe("/usb3104/ao/cmd", 1000, &USB3104::UpdateAOValueCb, this);

  pub_ao_state_msg_.data.resize(numberOfChannels_);

  ao_cmd_.resize(numberOfChannels_);

  for (int ch : ao_cmd_)
    ch = 0;
}

USB3104::~USB3104() {}

int USB3104::InitAO()
{
  int hasAO = 0;
  // char inputModeStr[64];
  char rangeStr[64];

  double min = 0.0;
  double max = 0.0;

  if (!daq_ready_)
  {
    ROS_ERROR_STREAM("DAQ board is not ready");
    return -1;
  }

  // verify the specified DAQ device supports analog input
  err_ = getDevInfoHasAo(daqDeviceHandle_, &hasAO);
  if (!hasAO)
  {
    ROS_ERROR_STREAM("The specified DAQ device does not support analog output");
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

  err_ = getAoInfoFirstSupportedRange(daqDeviceHandle_, &range_, rangeStr);
  ConvertRangeToMinMax(range_, &min, &max);

  ROS_INFO("\tANALOG OUTPUT INFO");
  ROS_INFO("\tRange: %s", rangeStr);

  ao_enabled_ = true;

  return 0;
}

void USB3104::UpdateAOValueCb(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  std::vector<double> ao_cmd_tmp;
  ao_cmd_tmp = msg->data;

  for (int ch = 0; ch < ao_cmd_tmp.size(); ch++)
  {
    if (ao_cmd_tmp[ch] >= -10 and ao_cmd_tmp[ch] <= 10)
    {
      ao_cmd_[ch] = ao_cmd_tmp[ch];
      UpdateStateChannelAO(ch);
    }
  }
}

int USB3104::UpdateStateAO()
{

  // double data = 0;

  if (!ao_enabled_)
  {
    ROS_ERROR_STREAM("AO Module is not enabled");
    return -1;
  }

  if (ao_cmd_.size() != numberOfChannels_)
  {
    ROS_ERROR_STREAM("# AO Channels doesn't match command size");
    return -1;
  }
  // ROS_INFO("Active DAQ device: %s (%s)", devDescriptor_.productName,
  // devDescriptor_.uniqueId);

  // for (int chan = 0; chan < numberOfChannels_; chan++)
  for (int chan = 0; chan < 6; chan++)
  {
    err_ = ulAOut(daqDeviceHandle_, chan, range_, flags_, ao_cmd_.at(chan));
    if (err_ == ERR_NO_ERROR)
    {
      // ROS_INFO("Channel(%d) Data: %+-10.6f", chan, ao_cmd_.at(chan));
      ao_state_.at(chan) = ao_cmd_.at(chan);
    }
    else
    {
      ROS_ERROR("Error setting AO command");
      return -1;
    }
  }
  ao_state_ = ao_cmd_;

  return 0;
}

int USB3104::UpdateStateChannelAO(int channel)
{

  // double data = 0;

  if (!ao_enabled_)
  {
    ROS_ERROR_STREAM("AO Module is not enabled");
    return -1;
  }

  if (ao_cmd_.size() != numberOfChannels_)
  {
    ROS_ERROR_STREAM("# AO Channels doesn't match command size");
    return -1;
  }

  // It takes about 1ms to update each channel
  err_ = ulAOut(daqDeviceHandle_, channel, range_, flags_, ao_cmd_.at(channel));

  if (err_ == ERR_NO_ERROR)
  {
    // ROS_INFO("Channel(%d) Data: %+-10.6f", chan, ao_cmd_.at(chan));
    ao_state_.at(channel) = ao_cmd_.at(channel);
  }
  else
  {
    ROS_ERROR("Error setting AO command");
    return -1;
  }

  ao_state_ = ao_cmd_;

  return 0;
}

int USB3104::PublishStateAO()
{
  pub_ao_state_msg_.data = ao_state_;
  pub_ao_state_.publish(pub_ao_state_msg_);
  return 0;
}

bool USB3104::IsEnabledAO() { return ao_enabled_; }

void USB3104::Quit()
{
  // disconnect from the DAQ device
  ulDisconnectDaqDevice(daqDeviceHandle_);

  // release the handle to the DAQ device
  if (daqDeviceHandle_)
    ulReleaseDaqDevice(daqDeviceHandle_);
}

void USB3104::PrintError(UlError err_)
{
  if (err_ != ERR_NO_ERROR)
  {
    char errMsg[ERR_MSG_LEN];
    ulGetErrMsg(err_, errMsg);
    ROS_ERROR("Error Code: %d \n", err_);
    ROS_ERROR("Error Message: %s \n", errMsg);
  }
}
void USB3104::set_ao_cmd(std::vector<double> ao_cmd) { ao_cmd_ = ao_cmd; }

int USB3104::get_ao_channels() { return 0; }
