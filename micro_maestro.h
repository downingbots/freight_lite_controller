// #include "/home/ubuntu/catkin_ws/src/freight_lite/src/libusb.h"
#include <freight_lite/libusb.h>
#include <controller_interface/controller_base.h>
#include <std_msgs/Int16.h>

namespace freight_lite { 
  class micro_maestro  
  {
  public:
    void initServos();
    int convertRadiansToPosition(int servo, double rad);
    double convertPositionToRadians(int servo, int position);
    bool servoDeviceMatchesVendorProduct(libusb_device *device, unsigned short idVendor, unsigned short idProduct);
    int servoGetPosition(int servo);
    void servoSetTarget(int servo, double rads);
    void servoSetAllTargets(double rad0, double rad1, double rad2, double rad3);
    void initDriveMode(int driveMode, bool do_sleep);
    void adjustSteeringCallback(const std_msgs::Int16& wheel_mode);

  }; 
} 
