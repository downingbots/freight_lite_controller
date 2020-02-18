// #include <libusb.h>
// #include "protocol"
#include <std_msgs/Int16.h>
#include <iostream>
#include <ros/ros.h>
#include <ros/ros.h>
#include <time.h>
#include <freight_lite/libusb.h>
#include <freight_lite/micro_maestro.h>
#include <freight_lite/protocol.h>
#include <freight_lite_teleop/freight_lite_defines.h>

// using namespace servo;
namespace freight_lite {

int      _startPosition[4];
int      _currentPosition[4];
int      _targetPosition[4];
long int _startTime[4];
double    _startRad[4];
double    _currentRad[4];
double    _targetRad[4];
int      _servoDirection[4];
int      _servoStraight[4];
int      _servoTwist[4];
int      _servoHoriz[4];
int      _servoMin[4];
int      _servoMax[4];
int      _driveMode;
int      _prevDriveMode;
int      _servoLock;
ros::Subscriber _sub_adjust_wheel;

struct timespec gettime_now;

#define RAD_TO_DEG       57.2958
#define DEG_TO_RAD       0.0174533
#define SEC_PER_60DEG    1.96
#define NS_PER_SEC       1000000000
#define RADIANS_PER_NS   ((DEG_TO_RAD) / (60 * SEC_PER_60DEG * NS_PER_SEC))
#define PI               3.14159

// 7 to 1 ratio in CM-785HB
// counter clockwise as pos increases
// 1413° max rot
// 128 or 256 max
// max 404"
// 2Pi radians in circle
// elapsedTime += 1000000000;	                 //(Rolls over every 1 second)

void micro_maestro::initServos()
{
  ros::NodeHandle n;

  _servoLock = 0;
  n.param("/freight_lite_controller/wheel_fr_dir", _servoDirection[WHEEL_FR], 1);
  n.param("/freight_lite_controller/wheel_fl_dir", _servoDirection[WHEEL_FL], 1);
  n.param("/freight_lite_controller/wheel_bl_dir", _servoDirection[WHEEL_BL], -1);
  n.param("/freight_lite_controller/wheel_br_dir", _servoDirection[WHEEL_BR], -1);
  n.param("/freight_lite_controller/wheel_fr_straight", _servoStraight[WHEEL_FR], 1);
  n.param("/freight_lite_controller/wheel_fl_straight", _servoStraight[WHEEL_FL], 1);
  n.param("/freight_lite_controller/wheel_bl_straight", _servoStraight[WHEEL_BL], -1);
  n.param("/freight_lite_controller/wheel_br_straight", _servoStraight[WHEEL_BR], -1);
  n.param("/freight_lite_controller/wheel_fr_horiz", _servoHoriz[WHEEL_FR], 1);
  n.param("/freight_lite_controller/wheel_fl_horiz", _servoHoriz[WHEEL_FL], 1);
  n.param("/freight_lite_controller/wheel_bl_horiz", _servoHoriz[WHEEL_BL], -1);
  n.param("/freight_lite_controller/wheel_br_horiz", _servoHoriz[WHEEL_BR], -1);
  n.param("/freight_lite_controller/wheel_fr_twist", _servoTwist[WHEEL_FR], 1);
  n.param("/freight_lite_controller/wheel_fl_twist", _servoTwist[WHEEL_FL], 1);
  n.param("/freight_lite_controller/wheel_bl_twist", _servoTwist[WHEEL_BL], -1);
  n.param("/freight_lite_controller/wheel_br_twist", _servoTwist[WHEEL_BR], -1);
  n.param("/freight_lite_controller/wheel_fr_min", _servoMin[WHEEL_FR], 1);
  n.param("/freight_lite_controller/wheel_fl_min", _servoMin[WHEEL_FL], 1);
  n.param("/freight_lite_controller/wheel_bl_min", _servoMin[WHEEL_BL], -1);
  n.param("/freight_lite_controller/wheel_br_min", _servoMin[WHEEL_BR], -1);
  n.param("/freight_lite_controller/wheel_fr_max", _servoMax[WHEEL_FR], 1);
  n.param("/freight_lite_controller/wheel_fl_max", _servoMax[WHEEL_FL], 1);
  n.param("/freight_lite_controller/wheel_bl_max", _servoMax[WHEEL_BL], -1);
  n.param("/freight_lite_controller/wheel_br_max", _servoMax[WHEEL_BR], -1);

  // Straight is 0 Rads
  servoSetTarget(WHEEL_BL, 0.0);
  servoSetTarget(WHEEL_FL, 0.0);
  servoSetTarget(WHEEL_BR, 0.0);
  servoSetTarget(WHEEL_FR, 0.0);
  sleep(SEC_PER_60DEG*3.5);  // max timeout to initialize start position
  _startRad[WHEEL_FR] = 0;
  _startRad[WHEEL_FL] = 0;
  _startRad[WHEEL_BL] = 0;
  _startRad[WHEEL_BR] = 0;
  _currentRad[WHEEL_FL] = 0;
  _currentRad[WHEEL_FR] = 0;
  _currentRad[WHEEL_BL] = 0;
  _currentRad[WHEEL_BR] = 0;
  // _sub_adjust_wheel = n.subscribe("adjust_steering", 1, &micro_maestro::adjustSteeringCallback, this);
}

int micro_maestro::convertRadiansToPosition(int servo, double rad)
{
  // fl br = -1 direction ;  fr bl = 1 direction
  // | all   :    0 RADS             = STRAIGHT (1700)(0 deg)
  // \ fl, br:    PI/4 ( 0.785) RADS = (STRAIGHT + HORIZ) / 2 
  // \ fl, br:   5PI/4 ( 0.785) RADS = (STRAIGHT + HORIZ) / 2  opp dir
  // \ fl, br:   9PI/4 ( 0.785) RADS = (STRAIGHT + HORIZ) / 2  opp dir
  // - fl, br:   2PI/4 ( 0.785) RADS 
  // - fl, br:   6PI/4 ( 0.785) RADS 
  // / fl, br:   7PI/4 ( 0.785) RADS = TWIST
  // / fl, br:   -PI/4 (-0.785) RADS = TWIST (45 deg)
  // / fl, br:   3PI/4 ( 0.785) RADS = TWIST opp dir
  // 
  // fl br = -1 direction ;  fr bl = 1 direction
  // | all   :    0 RADS             = STRAIGHT (1700)(0 deg)
  // \ fr, bl:    PI/4 ( 0.785) RADS = TWIST
  // \ fr, bl:   5PI/4 ( 0.785) RADS = TWIST
  // \ fr, bl:   9PI/4 ( 0.785) RADS = TWIST
  // - fl, br:   2PI/4 ( 0.785) RADS 
  // - fl, br:   6PI/4 ( 0.785) RADS 
  // / fr, bl:   7PI/4 ( 0.785) RADS = (STRAIGHT + HORIZ) / 2
  // / fr, bl:   -PI/4 (-0.785) RADS = (STRAIGHT + HORIZ) / 2
  // / fr, bl:   3PI/4 ( 0.785) RADS = (STRAIGHT + HORIZ) / 2
  // 
  int pos;
  double deg, mydeg, deg45;

  deg = rad * 180 / PI;
  mydeg = std::abs(deg);
  if (mydeg > 90)
      mydeg = (180 - mydeg);
  if ((servo == WHEEL_FL || servo == WHEEL_BR) 
     && (deg <= 0 && deg >= -180))
  {
    if (servo == WHEEL_FL) {
      deg45 = (_servoHoriz[servo] + _servoStraight[servo]) / 2;
      pos = int(((std::min(1.0, mydeg/ 45.0)) * (+deg45 - _servoStraight[servo])) + _servoStraight[servo] + .5);
    } else
      pos = int((std::min(1.0,(mydeg / 45.0)) * (+_servoTwist[servo] - _servoStraight[servo])) + _servoStraight[servo] + .5);
  } else if ((servo == WHEEL_FL || servo == WHEEL_BR)
      &&(deg >= 0 && deg <= 180))
  {
    if (servo == WHEEL_BR) {
      deg45 = (_servoHoriz[servo] + _servoStraight[servo]) / 2;
      pos = int(((std::min(1.0,(mydeg / 45.0))) * (+deg45 - _servoStraight[servo])) + _servoStraight[servo] + .5);
    } else
      pos = int((std::min(1.0,(mydeg / (45.0))) * (+_servoTwist[servo] - _servoStraight[servo])) + _servoStraight[servo] + .5);
  } else if ((servo == WHEEL_FR || servo == WHEEL_BL)
      &&(deg <= 0 && deg >= -180))
  {
    if (servo == WHEEL_BL)
      pos = int((std::min(1.0,(mydeg / (45.0))) * (-_servoTwist[servo] + _servoStraight[servo])) + _servoStraight[servo] + .5);
    else {
      deg45 = (_servoHoriz[servo] + _servoStraight[servo]) / 2;
      pos = int((std::min(1.0,(mydeg / (45.0))) * (-deg45 + _servoStraight[servo])) + _servoStraight[servo] + .5);
    }
  } else if ((servo == WHEEL_FR || servo == WHEEL_BL)
      &&(deg >= 0 && deg <= 180))
  {
    if (servo == WHEEL_FR)
      pos = int((std::min(1.0,(mydeg / (45.0))) * (-_servoTwist[servo] + _servoStraight[servo])) + _servoStraight[servo] + .5);
    else {
      deg45 = (_servoHoriz[servo] + _servoStraight[servo]) / 2;
      pos = int((std::min(1.0,(mydeg / (45.0))) * (-deg45 + _servoStraight[servo])) + _servoStraight[servo] + .5);
    }
  }
  else
  {
    std::ostringstream os1, os2, os3, os4, os5;
    os1 << servo;
    os2 << deg;
    os3 << mydeg;
    os4 << rad;
    os5 << pos;
    ROS_ERROR_STREAM("ERROR: servo " << os1.str() << " deg " << os2.str() << " mydeg " << os3.str() << " rad " << os4.str() << " pos " << os5.str());
  }
  if (_driveMode == MODE_STRAIGHT) {
    return(pos);
  } else if (_driveMode == MODE_HORIZ) {
    return(pos - _servoStraight[servo] + _servoHoriz[servo]);
  } else if (_driveMode == MODE_TWIST) {
    return(_servoTwist[servo]);
  }
}

double micro_maestro::convertPositionToRadians(int servo, int position)
{
  // return PI * ((position - _servoMin[servo])/(_servoMax[servo] - _servoMin[servo]));
  return PI/4 * std::max(-1.0, std::min(1.0, 1.0*(position - _servoStraight[servo]) / (_servoTwist[servo] - _servoStraight[servo])))*_servoDirection[servo];
}


bool micro_maestro::servoDeviceMatchesVendorProduct(libusb_device *device, unsigned short idVendor, unsigned short idProduct)
{
  libusb_device_descriptor desc;
  libusb_get_device_descriptor(device, &desc);
  return idVendor == desc.idVendor && idProduct == desc.idProduct;
}

void micro_maestro::initDriveMode(int driveMode, bool do_sleep)
{
  if (driveMode == MODE_STRAIGHT) 
  {
    servoSetTarget(WHEEL_FL, 0);
    servoSetTarget(WHEEL_FR, 0);
    servoSetTarget(WHEEL_BL, 0);
    servoSetTarget(WHEEL_BR, 0);
  } 
  else if (driveMode == MODE_HORIZ) 
  {
    servoSetTarget(WHEEL_FL, PI/2);
    servoSetTarget(WHEEL_FR, PI/2);
    servoSetTarget(WHEEL_BL, PI/2);
    servoSetTarget(WHEEL_BR, PI/2);
  } 
  else if (driveMode == MODE_TWIST) 
  {
    servoSetTarget(WHEEL_FL, PI/4);
    servoSetTarget(WHEEL_FR, PI/4);
    servoSetTarget(WHEEL_BL, PI/4);
    servoSetTarget(WHEEL_BR, PI/4);
  } 
  if (do_sleep)
    sleep(SEC_PER_60DEG*3.5);  // max timeout to initialize start position
}

void micro_maestro::servoSetTarget(int servo, double rads)
{
  int position;
  const unsigned short vendorId = 0x1ffb;
  unsigned short productIDArray[]={0x0089, 0x008a, 0x008b, 0x008c};

  while (_servoLock) sleep(.01);
  _servoLock = 1;
  // if (rads == 0)
    // return;
  // ARD: prevent new servoSetTarget if previous target not yet reached?
  libusb_context *ctx=0;
  libusb_device **device_list=0;
  libusb_init(&ctx);
  int count=libusb_get_device_list(ctx, &device_list);
  for(int i=0;i<count;i++)
  {
    libusb_device *device=device_list[i];
    {
      for(int Id=0;Id<4;Id++)
      {
        if(servoDeviceMatchesVendorProduct(device, vendorId, productIDArray[Id]))
        {
          libusb_device_handle *device_handle;
          libusb_open(device, &device_handle);
          // for (int sv = 0 ; sv < 4; sv++) {
          position = convertRadiansToPosition(servo, rads);
          // std::cout << "servo " << servo << " pos " << position << " rads " << rads << "/n";
          libusb_control_transfer(device_handle, 0x40, REQUEST_SET_TARGET, position*4, servo, 0, 0, (ushort)5000);
          // _startRad[sv] = _currentRad[sv];
          // }
          libusb_close(device_handle);
          break;
        }
      }
    }
  }
  libusb_free_device_list(device_list, 0);
  libusb_exit(ctx);
  _startRad[servo] = _currentRad[servo];
  _targetRad[servo] = rads;
  _targetPosition[servo] = rads * RAD_TO_DEG;      // in degrees ; NOT PWM POS!
  clock_gettime(CLOCK_REALTIME, &gettime_now);
  _startTime[servo] = gettime_now.tv_nsec;

  std::ostringstream os0, os1, os2, os3;
  os1 << std::fixed << std::setprecision(4) << _targetRad[servo];
  os0 << position;
  os2 << std::fixed << std::setprecision(4) << _startRad[servo];
  os3 << _startTime[servo];
  ROS_INFO_STREAM("Set target " << os1.str() << " position " << os0.str() << " start " << os2.str() << " at " << os3.str());
  _servoLock = 0;
}

void micro_maestro::servoSetAllTargets(double rad0, double rad1, double rad2, double rad3)
{
  int pos0, pos1, pos2, pos3;
  const unsigned short vendorId = 0x1ffb;
  unsigned short productIDArray[]={0x0089, 0x008a, 0x008b, 0x008c};

  while (_servoLock) sleep(.01);
  _servoLock = 1;
  libusb_context *ctx=0;
  libusb_device **device_list=0;
  libusb_init(&ctx);
  int count=libusb_get_device_list(ctx, &device_list);
  for(int i=0;i<count;i++)
  {
    libusb_device *device=device_list[i];
    {
      for(int Id=0;Id<4;Id++)
      {
        if(servoDeviceMatchesVendorProduct(device, vendorId, productIDArray[Id]))
        {
          libusb_device_handle *device_handle;
          libusb_open(device, &device_handle);
          pos0 = convertRadiansToPosition(0, rad0);
          pos1 = convertRadiansToPosition(1, rad1);
          pos2 = convertRadiansToPosition(2, rad2);
          pos3 = convertRadiansToPosition(3, rad3);
          libusb_control_transfer(device_handle, 0x40, REQUEST_SET_TARGET, pos0*4, 0, 0, 0, (ushort)5000);
          libusb_control_transfer(device_handle, 0x40, REQUEST_SET_TARGET, pos1*4, 1, 0, 0, (ushort)5000);
          libusb_control_transfer(device_handle, 0x40, REQUEST_SET_TARGET, pos2*4, 2, 0, 0, (ushort)5000);
          libusb_control_transfer(device_handle, 0x40, REQUEST_SET_TARGET, pos3*4, 3, 0, 0, (ushort)5000);
          libusb_close(device_handle);
          break;
        }
      }
    }
  }
  libusb_free_device_list(device_list, 0);
  libusb_exit(ctx);
  clock_gettime(CLOCK_REALTIME, &gettime_now);
  _targetRad[0] = rad0;
  _targetRad[1] = rad1;
  _targetRad[2] = rad2;
  _targetRad[3] = rad3;
  _targetPosition[0] = rad0 * RAD_TO_DEG;
  _targetPosition[1] = rad1 * RAD_TO_DEG;
  _targetPosition[2] = rad2 * RAD_TO_DEG;
  _targetPosition[3] = rad3 * RAD_TO_DEG;
  for (int servo = 0 ; servo < 4; servo++) 
  {
    _startRad[servo] = _currentRad[servo];
    _startTime[servo] = gettime_now.tv_nsec;
  }

  std::ostringstream os0, os1, os2, os3;
  os0 << pos0;
  os1 << pos1;
  os2 << pos2;
  os3 << pos3;
  ROS_INFO_STREAM("Set positions " << os0.str() << ", " << os1.str() << ", " << os2.str() << ", " << os3.str());
  _servoLock = 0;
}

int micro_maestro::servoGetPosition(int servo)
{
  long int elapsedTime;
  clock_gettime(CLOCK_REALTIME, &gettime_now);
  elapsedTime = gettime_now.tv_nsec - _startTime[servo]; //Get nS value
  if (elapsedTime < 0)
    elapsedTime += 1000000000;	                 //(Rolls over every 1 second)
  if (_startRad[servo] > _targetRad[servo])
  {
     // _currentRad[servo] = _startRad[servo] - (elapsedTime * RADIANS_PER_NS) * _servoDirection[servo];
     _currentRad[servo] = _startRad[servo] - (elapsedTime * RADIANS_PER_NS) ;
     if (_currentRad[servo] < _targetRad[servo])
       _currentRad[servo] = _targetRad[servo];
  }
  else 
  {
     // _currentRad[servo] = _startRad[servo] + (elapsedTime * RADIANS_PER_NS) * _servoDirection[servo];
     _currentRad[servo] = _startRad[servo] + (elapsedTime * RADIANS_PER_NS) ;
     if (_currentRad[servo] > _targetRad[servo])
       _currentRad[servo] = _targetRad[servo];
  }
  std::ostringstream oss, os0, os1, os2, os3;
  oss << servo;
  os0 << std::fixed << std::setprecision(4) << _currentRad[servo];
  os1 << std::fixed << std::setprecision(4) << _targetRad[servo];
  os2 << std::fixed << std::setprecision(4) << _startRad[servo];
  os3 << _startTime[servo];
  ROS_DEBUG_STREAM("CurrentRad " << os0.str() << " target " << os1.str() << " start " << os2.str()  << " time " << os3.str());

  return _currentRad[servo];
}

void micro_maestro::adjustSteeringCallback(const std_msgs::Int16& wheel_mode)
{
    if (_driveMode != MODE_NONE)
      _prevDriveMode = _driveMode;
    // to map wheel mode to the wheel, take absolute value and subtract 1 
    int servo = abs(wheel_mode.data)-1;
    if (servo == WHEEL_FL || servo == WHEEL_FR
     || servo == WHEEL_BL || servo == WHEEL_BR) 
    {
      // _driveMode = MODE_ADJUST;
      if (wheel_mode.data > 0) {
        // if positive, increase wheel settings by 1 
        _servoTwist[servo] += 5;
        _servoStraight[servo] += 5;
        _servoHoriz[servo] += 5;
        _servoMin[servo] += 5;
        _servoMax[servo] += 5;
      } else {
        // if negative, decrease wheel settings by 1 
        _servoTwist[servo] -= 5;
        _servoStraight[servo] -= 5;
        _servoHoriz[servo] -= 5;
        _servoMin[servo] -= 5;
        _servoMax[servo] -= 5;
      }
      ROS_INFO_STREAM( "servo " << servo << " Twist " << _servoTwist[servo] << " Straight " << _servoStraight[servo] << " Horiz " << _servoHoriz[servo] << " Min " << _servoMin[servo] << " Max " << _servoMax[servo] << "/n");
      // std::cout << "servo " << servo << " Twist " << _servoTwist[servo] << " Straight " << _servoStraight[servo] << " Horiz " << _servoHoriz[servo] << " Min " << _servoMin[servo] << " Max " << _servoMax[servo] << "/n";
      initDriveMode(_driveMode, false);
    }
    else if (wheel_mode.data == ADJUST_WHEEL_ALL_TWIST)
    {
      _driveMode = MODE_TWIST;
    }
    else if (wheel_mode.data == ADJUST_WHEEL_ALL_STRAIGHT)
    {
      _driveMode = MODE_STRAIGHT;
    }
    else if (wheel_mode.data == ADJUST_WHEEL_ALL_HORIZ)
    {
      _driveMode = MODE_HORIZ;
    }
    else if (wheel_mode.data == ADJUST_WHEEL_ALL_HORIZ)
    {
      _driveMode = MODE_NONE;
    }
    if (_prevDriveMode != _driveMode) 
    {
      initDriveMode(_driveMode, true);
    }
}
}

