#include <openrst_control/openrst_control.h>

// C
#include <signal.h>

// C++
#include <thread>

using namespace openrst_nu;

bool kill_this_process = false;

void SigIntHandler(int signal)
{
  kill_this_process = true;
  ROS_WARN_STREAM("SHUTDOWN SIGNAL RECEIVED");
  ros::shutdown();
}

int main(int argc, char **argv)
{
  // Ros related
  ros::init(argc, argv, "openrst_control");
  ros::NodeHandle node_handle;
  signal(SIGINT, SigIntHandler);

  OpenRSTControl orst(node_handle, &kill_this_process);

  ros::AsyncSpinner spinner(3);
  spinner.start();

  std::thread t(std::bind(&OpenRSTControl::ControlLoop, &orst));

  t.join();
  spinner.stop();

  return 0;
}
