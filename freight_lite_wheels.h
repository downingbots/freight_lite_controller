// #include "/home/ubuntu/catkin_ws/src/freight_lite/src/libusb.h"
#include <controller_interface/controller_base.h>
#include <std_msgs/Int16.h>

namespace freight_lite { 
  class freight_lite_wheels  
  {
  public:
    void multicastSetWheelSpeeds(int16_t fl, int16_t fr, int16_t bl, int16_t br);
    double getVelocity(int);
    void initWheels(void);
    // callback to dgram message; subscribed by initWheels 
    void wheelCallback(const packets_485net::packet_485net_dgram& ws);
    // only called by initWheels
    void initWheelPID();
    // only called by initWheelPID
    void initOneWheelPID(unsigned char id, int32_t p, int32_t i, int32_t d, unsigned char dir);

  private:
    ros::Publisher cmd_pub;
    int pid_p = 0;
    int pid_i = 0;
    int pid_d = 0;
    unsigned char idfl;
    unsigned char idfr;
    unsigned char idbl;
    unsigned char idbr;
    unsigned char mcaw;
    bool rosinfodbg = true;

    // wheel_state (sent in wheelCallback)
    bool wheel_stopped = true;
    //fl, fr, bl, br
    bool wheel_started[4] = {false, false, false, false};

    //front left, front right, back left, back right
    unsigned char wheel_debug_bits[4] = {0};

    bool complete_wheel_command(void);
  }; 
} 

