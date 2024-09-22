#define USE_NEW_ROS_COMM TRUE  // Enable the module TRUE

#define NEW_ROS_NUM \
  1  // The number of the ros communication: 2 for sentry, 1 for other robots

#define NEW_ROS_MAX_PACKAGE_SIZE \
  64  // User defined maximum payload length of a single package

#define NEW_ROS_RX_BUF_SIZE \
  1024  // The rx buffer size; better to be the same with the tx buffer size of
        // the ROS Host

#define NEW_ROS_TX_BUF_SIZE \
  512  // Tx buffer size; better to be the same with the rx buffer size of the
       // ROS host

#define NEW_ROS_RX_TIMEOUT \
  50  // THe timeout of receiving data from the ROS host

#define NEW_ROS_MAX_FRAME_CALLBACK_NUM \
  4  // The number of the callback function allowed

#define NEW_ROS_TX_TASK_STACK_SIZE 256  // tx task stack size

#define NEW_ROS_RX_TASK_STACK_SIZE 512  // rx task stack size