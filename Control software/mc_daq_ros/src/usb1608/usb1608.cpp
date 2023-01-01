
#include <usb1608/usb1608.h>

/*
 * Constructor for USB1608
 * @param nh Node handle
 * @param daq_id Board ID
 */

USB1608::USB1608(ros::NodeHandle nh, std::string daq_id)
    : nh_(nh), daq_id_(daq_id)
{
  daq_ready_ = false;
  daq_connected_ = false;
  ai_enabled_ = false;
  dio_enabled_ = false;
  n_dinput_ = 0;
  n_doutput_ = 0;

  descriptorIndex_ = 0;
  interfaceType_ = USB_IFC;
  daqDeviceHandle_ = 0;
  numDevs_ = kMaxDevCount;

  ai_state_.resize(kNumberOfAiChannels);
  di_state_.resize(kNumberOfDioIO);
  flags_ = AIN_FF_DEFAULT;

  // Scan options AI
  numberOfChannels_ = 0;
  ai_rate_ = 10000;
  ai_scanOptions_ = (ScanOption)(SO_DEFAULTIO | SO_CONTINUOUS);
  flags_ai_scan_ = AINSCAN_FF_DEFAULT;
  samplesPerChannel_ = 10;

  int i = 0;
  err_ = ERR_NO_ERROR;
  DaqDeviceDescriptor devDescriptors[kMaxDevCount];

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
      ROS_ERROR_STREAM("DAQ Board with ID " << daq_id_ << " not found between "
                                            << numDevs_ << " devices found");
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

  // Filters coefficients

  VectorXd a_coef(3);
  a_coef << -2.7488358092146754, 2.5282312191425587, -0.7776385602380801;

  VectorXd b_coef(4);
  b_coef << 0.00021960621122536214, 0.0006588186336760865,
      0.0006588186336760865, 0.00021960621122536214;

  filter_iir_ai = new filters::IIR(16, 3, b_coef, a_coef);

  // Publishers
  pub_ai_state_ =
      nh_.advertise<std_msgs::Float64MultiArray>("/usb1608/ai/state", 1);
  pub_ai_state_filtered = nh_.advertise<std_msgs::Float64MultiArray>(
      "/usb1608/ai/state_filtered", 1);
  pub_di_state_ =
      nh_.advertise<std_msgs::Int32MultiArray>("/usb1608/di/state", 1);

  // Subscribers
  sub_do_cmd_ =
      nh_.subscribe("/usb1608/do/cmd", 1000, &USB1608::UpdateDOValueCb, this);

  pub_ai_state_msg_.data.resize(kNumberOfAiChannels);
  pub_ai_state_filtered_msg_.data.resize(kNumberOfAiChannels);
  pub_di_state_msg_.data.resize(kNumberOfDioIO);

  ai_state_filtered_.resize(kNumberOfAiChannels);
}

USB1608::~USB1608() {}

/*
 * Initialize the Analog Input channels
 */
int USB1608::InitAI()
{
  int hasAI = 0;
  char inputModeStr[64];
  char rangeStr[64];

  if (!daq_ready_)
  {
    ROS_ERROR_STREAM("DAQ board is not ready");
    return -1;
  }

  // verify the specified DAQ device supports analog input
  err_ = getDevInfoHasAi(daqDeviceHandle_, &hasAI);
  if (!hasAI)
  {
    ROS_ERROR_STREAM("The specified DAQ device does not support analog input");
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

  // get the first supported analog input mode
  err_ = getAiInfoFirstSupportedInputMode(daqDeviceHandle_, &numberOfChannels_,
                                          &inputMode_, inputModeStr);

  // get the first supported analog input range
  err_ = getAiInfoFirstSupportedRange(daqDeviceHandle_, inputMode_, &ai_range_,
                                      rangeStr);

  // ROS_INFO("%s ready", devDescriptor_.devString);
  ROS_INFO("\tANALOG INPUT INFO");
  ROS_INFO("\t# Channels: %d", numberOfChannels_);
  ROS_INFO("\tInput mode: %s", inputModeStr);
  ROS_INFO("\tRange: %s", rangeStr);

  ai_enabled_ = true;
  return 0;
}

/*
 *  Initialize digital input/output ports
 */
int USB1608::InitDIO()
{
  InitDIO(kNumberOfDioIO, 0); // All DIO as inputs
  return 0;
}

/*
 * Initialize the digital input/output ports
 *
 * @param n_din Number of digital input
 * @param n_dout Number of digital output
 */
int USB1608::InitDIO(int n_din, int n_dout)
{
  int hasDIO = 0;
  char portTypeStr[kMaxStrlength];
  char portIoTypeStr[kMaxStrlength];

  if (!daq_ready_)
  {
    ROS_ERROR_STREAM("DAQ board is not ready");
    return -1;
  }

  if (n_din + n_dout != kNumberOfDioIO)
  {
    ROS_ERROR_STREAM(
        "Requested DIN + DOUT doesn't match device number of DIOs");
    return -1;
  }

  n_dinput_ = n_din;
  n_doutput_ = n_dout;

  // verify the device supports digital input
  err_ = getDevInfoHasDio(daqDeviceHandle_, &hasDIO);
  if (!hasDIO)
  {
    ROS_ERROR_STREAM("The specified DAQ device does not support Digital I/O");
    return -1;
  }

  // establish a connection to the DAQ device
  if (!daq_connected_)
    err_ = ulConnectDaqDevice(daqDeviceHandle_);

  if (err_ != ERR_NO_ERROR)
  {
    ROS_ERROR_STREAM("ERROR connecting to DAQ device");
    return -1;
  }

  daq_connected_ = true;

  // get the first port type (AUXPORT0, FIRSTPORTA, ...)
  err_ = getDioInfoFirstSupportedPortType(daqDeviceHandle_, &portType_,
                                          portTypeStr);

  // get the I/O type for the fisrt port
  err_ = getDioInfoFirstSupportedPortIoType(daqDeviceHandle_, &portIoType_,
                                            portIoTypeStr);

  // get the number of bits for the first port (port index = 0)
  err_ = getDioInfoNumberOfBitsForFirstPort(daqDeviceHandle_, &bitsPerPort_);

  if (n_din == kNumberOfDioIO)
  {
    if (portIoType_ == DPIOT_IO || portIoType_ == DPIOT_BITIO)
    {
      // configure the first port for input
      err_ = ulDConfigPort(daqDeviceHandle_, portType_, DD_INPUT);
      ROS_INFO("DIO Port configure as INPUT");
    }
  }
  else if (n_din == 0)
  {
    if (portIoType_ == DPIOT_IO || portIoType_ == DPIOT_BITIO)
    {
      // configure the first port for output
      err_ = ulDConfigPort(daqDeviceHandle_, portType_, DD_OUTPUT);
      ROS_INFO("DIO Port configure as OUTPUT");
    }
  }
  else
  {
    if (portIoType_ == DPIOT_BITIO)
    {
      ROS_WARN("\tAssigning first %i bits as INPUT and next %i bits as OUTPUT",
               n_din, bitsPerPort_ - n_din);

      // configure all of the bits for input for the port
      for (int bitNumber = 0; bitNumber < bitsPerPort_; bitNumber++)
      {
        if (bitNumber < n_din)
          err_ = ulDConfigBit(daqDeviceHandle_, portType_, bitNumber, DD_INPUT);
        else
          err_ =
              ulDConfigBit(daqDeviceHandle_, portType_, bitNumber, DD_OUTPUT);

        if (err_ != ERR_NO_ERROR)
          break;
      }
    }
    else
    {
      ROS_WARN("Device doesn't support bit IO assignment. Setting entire port "
               "as INPUT");
      err_ = ulDConfigPort(daqDeviceHandle_, portType_, DD_INPUT);
    }
  }

  // ROS_INFO("%s ready", devDescriptor_.devString);
  ROS_INFO("\tDIO PORT INFO");
  ROS_INFO("\tPort: %s", portTypeStr);
  ROS_INFO("\tPort I/O type: %s", portIoTypeStr);

  dio_enabled_ = true;
  ROS_INFO("\tDIO ENABLED: %i", dio_enabled_);
  return 0;
}

/*
 * Start scanning analog input channels
 */
int USB1608::StartScanAI()
{
  double data = 0;

  if (!ai_enabled_)
  {
    ROS_ERROR_STREAM("AI Module is not enabled");
    return -1;
  }

  ai_buffer_ =
      (double *)malloc(numberOfChannels_ * samplesPerChannel_ * sizeof(double));
  err_ = ulAInScan(daqDeviceHandle_, 0, numberOfChannels_ - 1, inputMode_,
                   ai_range_, samplesPerChannel_, &ai_rate_, ai_scanOptions_,
                   flags_ai_scan_, ai_buffer_);

  if (err_ == ERR_NO_ERROR)
    return 0;

  return -1;
}

/*
 * Stop scanning analog input channels
 */
int USB1608::StopScanAI()
{
  ScanStatus status;
  TransferStatus transferStatus;

  err_ = ulAInScanStatus(daqDeviceHandle_, &status, &transferStatus);

  double data = 0;
  if (status == SS_RUNNING && err_ == ERR_NO_ERROR)
  {
    err_ = ulAInScanStop(daqDeviceHandle_);
  }

  return 0;
}

/*
 * Update analog input channels state with current values
 */
int USB1608::UpdateStateAI()
{
  double data = 0;

  if (!ai_enabled_)
  {
    ROS_ERROR_STREAM("AI Module is not enabled");
    return -1;
  }

  // display data for all the analog input channels
  for (int chan = 0; chan < numberOfChannels_; chan++)
  {
    err_ = ulAIn(daqDeviceHandle_, chan, inputMode_, ai_range_, flags_, &data);

    if (err_ == ERR_NO_ERROR)
    {
      // ROS_INFO("Channel(%d) Data: %+-10.6f", chan, data);
      ai_state_.at(chan) = data;
    }
  }
  return 0;
}

/*
 * Update analog input channels state with current scan values
 */
int USB1608::UpdateScanStateAI()
{
  ScanStatus status;
  TransferStatus transferStatus;

  err_ = ulAInScanStatus(daqDeviceHandle_, &status, &transferStatus);
  int index = transferStatus.currentIndex;
  // printf("actual scan rate = %f\n\n", ai_rate_);

  for (int ch = 0; ch < numberOfChannels_; ch++)
  {
    if (err_ == ERR_NO_ERROR)
    {
      // ROS_INFO("Channel(%d) Data: %+-10.6f", ch, ai_buffer_[index + ch]);
      ai_state_.at(ch) = ai_buffer_[index + ch];
    }
  }

  ai_state_filtered_ = filter_iir_ai->update(VectorXd::Map(&ai_state_[0], 16));

  return 0;
}

/*
 * Get current analog input state for a range of channels
 * @param start_ch Start Channel
 * @param n_channels Number of channels
 */
std::vector<double> USB1608::GetScanStateChannelsAI(int start_ch,
                                                    int n_channels)
{
  ScanStatus status;
  TransferStatus transferStatus;
  std::vector<double> ai_out;

  ai_out.resize(n_channels);

  err_ = ulAInScanStatus(daqDeviceHandle_, &status, &transferStatus);
  int index = transferStatus.currentIndex;
  // printf("actual scan rate = %f\n\n", ai_rate_);

  for (int ch = start_ch; ch < start_ch + n_channels; ch++)
  {
    if (err_ == ERR_NO_ERROR)
    {
      // ROS_INFO("Channel(%d) Data: %+-10.6f", ch, ai_buffer_[index + ch]);
      ai_out.at(ch) = ai_buffer_[index + ch];
    }
  }

  return ai_out;
}

/*
 * Update digital input port state with current value
 */
int USB1608::UpdateStateDI()
{
  unsigned int data = 0;

  if (!dio_enabled_)
  {
    ROS_ERROR_STREAM("DI Module is not enabled");
    return -1;
  }

  // read the port
  // err_ = ulDIn(daqDeviceHandle_, portType_, &data);

  for (int bitNumber = 0; bitNumber <= bitsPerPort_; bitNumber++)
  {
    err_ = ulDBitIn(daqDeviceHandle_, portType_, bitNumber, &data);

    if (err_ == ERR_NO_ERROR)
    {
      // ROS_INFO("Bit Number(%d) Data: %+-10.6f", bitNumber, data);
      di_state_.at(bitNumber) = data;
    }
  }

  // ROS_INFO("Data: %lld (0x%llx)\n", data, data);
  return 0;
}
/*
 * Update digital input channels state with current values
 * @param bit_number Bit number
 */
int USB1608::UpdateStateDI(int bit_number)
{
  unsigned int data = 0;

  if (!dio_enabled_)
  {
    ROS_ERROR_STREAM("DI Module is not enabled");
    return -1;
  }

  // read the port
  err_ = ulDBitIn(daqDeviceHandle_, portType_, bit_number, &data);

  // clearEOL();
  ROS_INFO("Bit Number: %d:  Data: %d\n", bit_number, data);
  return 0;
}

/*
 * Update digital output channels state
 * @param bit_number Bit number
 * @param bit_value Bit value
 */

int USB1608::UpdateStateDO(int bit_number, unsigned int bit_value)
{
  if (!dio_enabled_)
  {
    ROS_ERROR_STREAM("DIO Module is not enabled");
    return -1;
  }

  err_ = ulDBitOut(daqDeviceHandle_, portType_, bit_number, bit_value);
  // ROS_INFO("Bit %d = %d\n", bit_number, bit_value);
  return 0;
}

/*
 * Update digital output port state
 */
int USB1608::UpdateStateDO()
{
  if (!dio_enabled_)
  {
    ROS_ERROR_STREAM("DIO Module is not enabled");
    return -1;
  }
  if (n_doutput_ > 0 && do_cmd_.size() == n_doutput_)
  {
    for (int bitNumber = 0; bitNumber < n_doutput_; bitNumber++)
    {
      err_ = ulDBitOut(daqDeviceHandle_, portType_, n_dinput_ + bitNumber,
                       do_cmd_.at(bitNumber));
      // ROS_INFO("Bit DO %d = %d\n", bitNumber, do_cmd_.at(bitNumber));
    }
  }
  else
  {
    ROS_ERROR(
        "No DO enabled OR Command size doesn't match number of DOs enabled");
  }
  return 0;
}

/*
 * Publish Analog Input State
 */
int USB1608::PublishStateAI()
{
  pub_ai_state_msg_.data = ai_state_;
  pub_ai_state_.publish(pub_ai_state_msg_);
  VectorXd::Map(&pub_ai_state_filtered_msg_.data[0],
                pub_ai_state_filtered_msg_.data.size()) = ai_state_filtered_;
  pub_ai_state_filtered.publish(pub_ai_state_filtered_msg_);
  return 0;
}

/*
 * Publish Digital Input State
 */
int USB1608::PublishStateDI()
{
  pub_di_state_msg_.data = di_state_;
  pub_di_state_.publish(pub_di_state_msg_);
  return 0;
}

void USB1608::Quit()
{
  // disconnect from the DAQ device
  ulDisconnectDaqDevice(daqDeviceHandle_);

  // release the handle to the DAQ device
  if (daqDeviceHandle_)
    ulReleaseDaqDevice(daqDeviceHandle_);
}

void USB1608::PrintError(UlError err_)
{
  if (err_ != ERR_NO_ERROR)
  {
    char errMsg[ERR_MSG_LEN];
    ulGetErrMsg(err_, errMsg);
    ROS_ERROR("Error Code: %d \n", err_);
    ROS_ERROR("Error Message: %s \n", errMsg);
  }
}

void USB1608::UpdateDOValueCb(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
  ROS_INFO("Updating DO");
  do_cmd_ = msg->data;
  UpdateStateDO();
}
