#ifndef _OPENRST_STATUS_CODE_H_
#define _OPENRST_STATUS_CODE_H_

// Function return status

#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1
#define EXIT_PREEMPTED 2
#define EXIT_ABORTED 3

// Object number
#define OPENRST0 0
#define OPENRST1 1

enum STATUS
{
  STOPPED = 0,
  MOVING
};

// OpenRST Status
enum
{
  F_UNINITIALIZED = 0,
  F_CONNECTED,
  F_READY,
  F_POSITION_CONTROL,
  F_TORQUE_CONTROL,
};

// OpenRST Modes
enum
{
  MODE_MANUAL = 0,
  MODE_AUTO,
};

#endif
