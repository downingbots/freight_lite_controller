
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "packets_485net/packet_485net_dgram.h"
#include "net_485net_id_handler/SearchID.h"
#include "packets_485net/ReturnJointStates.h"
#include <unistd.h>
#include <cstdlib>
#include <freight_lite/freight_lite_wheels.h>

namespace freight_lite {

ros::ServiceClient idlookup;
ros::Subscriber wheel_sub;

// ros::Publisher cmd_pub;
// int pid_p = 0;
// int pid_i = 0;
// int pid_d = 0;
int32_t pid_p = 0x01;
int32_t pid_i = 0x08;
int32_t pid_d = 0x01;
int16_t vl[2];
int16_t vr[2];
bool rosinfodbg = true;

// wheel_state (sent in wheelCallback)
bool wheel_stopped = true;
//fl, fr, bl, br
bool wheel_started[4] = {false, false, false, false};

//front left, front right, back left, back right
unsigned char wheel_debug_bits[4] = {0};

uint8_t lookup_id(const char *type, const char *desc)
{
  net_485net_id_handler::SearchID search;
  
  search.request.type = type;
  search.request.desc = desc;
  // ROS_INFO("type %s desc %s", type, desc);
  if(idlookup.call(search))
  {
    return search.response.res;
  }
  else
  {
    // const char * const names[] = {"front left", "front right", "back left", "back right"};
    // const int dest[] = {8, 10, 9, 11};
    // 08|wheel-cnt|front left
    // 09|wheel-cnt|back left
    // 0A|wheel-cnt|front right
    // 0B|wheel-cnt|back right
    if (type == "wheel-cnt" and desc == "front left")
      return 0x08;
    else if (type == "wheel-cnt" and desc == "front right")
      return 0x0A;
    else if (type == "wheel-cnt" and desc == "back left")
      return 0x09;
    else if (type == "wheel-cnt" and desc == "back right")
      return 0x0B;
    else if (type == "multicast" and desc == "all wheels")
      return 0xC5;   // 197
    //ERROR
    ROS_INFO("ERROR: lookup_id %s; %s\n", type, desc);
    return 0xFF;
  }
}



// static
void freight_lite_wheels::initOneWheelPID(unsigned char id, int32_t p, int32_t i, int32_t d, unsigned char dir)
{
  if(id > 3)
  {
    ROS_INFO("ERROR: tried to send pid for invalid wheel %d\n", id);
    return;
  }
  
  const char * const names[] = {"front left", "front right", "back left", "back right"};
  // const uint8_t dest[] = {8, 10, 9, 11};
  const uint8_t dest[] = {0x08, 0x0A, 0x09, 0x0B};


  packets_485net::packet_485net_dgram pid_gains;

  ROS_INFO("WC 2 : %d %d %d %d", lookup_id("wheel-cnt", "front left"), lookup_id("wheel-cnt", "front right") , lookup_id("wheel-cnt", "back right"), lookup_id("wheel-cnt", "back left"));
  
  pid_gains.source = 0xF0;
  pid_gains.sport = 7;
  pid_gains.dport = 1;
    
  pid_gains.data.push_back((p >>  0) & 0xFF);
  pid_gains.data.push_back((p >>  8) & 0xFF);
  pid_gains.data.push_back((p >> 16) & 0xFF);
  pid_gains.data.push_back((p >> 24) & 0xFF);
  pid_gains.data.push_back((i >>  0) & 0xFF);
  pid_gains.data.push_back((i >>  8) & 0xFF);
  pid_gains.data.push_back((i >> 16) & 0xFF);
  pid_gains.data.push_back((i >> 24) & 0xFF);
  pid_gains.data.push_back((d >>  0) & 0xFF);
  pid_gains.data.push_back((d >>  8) & 0xFF);
  pid_gains.data.push_back((d >> 16) & 0xFF);
  pid_gains.data.push_back((d >> 24) & 0xFF);
  
  pid_gains.data.push_back(dir);
  
  pid_gains.destination = lookup_id("wheel-cnt", names[id]);
  if (pid_gains.destination == 255) {
    pid_gains.destination = dest[id];
  }

  /*
   8: ('wheel-cnt', 'front left'), 
   9: ('wheel-cnt', 'back left'), 
  10: ('wheel-cnt', 'front right'), 
  11: ('wheel-cnt', 'back right'), 
  'wheel-cnt': ('firmware/wheel-cnt.bin', 'firmware/485lib-normal.bin')
  */

  
  int w = 10;
  do
  {
    ROS_INFO("Trying to send pid to %d %s %d\n", pid_gains.destination, names[id],wheel_debug_bits[id]);
    cmd_pub.publish(pid_gains);
    ros::Duration(0.01).sleep();
    ros::spinOnce();
    // w--;
  }
  while((wheel_debug_bits[id] & 4) == 0 && w > 0);
  // ARD debugging hack
  // while(false);

  ROS_INFO("WC 1 : %d %d %d %d", lookup_id("wheel-cnt", "front left"), lookup_id("wheel-cnt", "front right") , lookup_id("wheel-cnt", "back right"), lookup_id("wheel-cnt", "back left"));
  ROS_INFO("pid_p %d pid_i %d pid_p %d\n", pid_p, pid_i, pid_d);
}

// static 
void freight_lite_wheels::initWheelPID()
{
  /* TUNING THE PID
     First set K_i and K_d values to zero. 
     Increase the K_p until the output of the loop oscillates, 
     Then the K_p should be set to approximately half of that value 
         for a "quarter amplitude decay" type response. 
     Then increase K_i until any offset is corrected 
          Too much K_i will cause instability. 
     Finally, increase K_d, if required, until the loop is acceptably quick 
          to reach its reference after a load disturbance. i
          Too much K_d will cause excessive response and overshoot. 
     A fast PID loop tuning usually overshoots slightly to reach the setpoint 
     If overshoot undesirable set K_p significantly less than half that of 
          the K_p setting that was causing oscillation.

Effects of increasing a parameter independently
Parameter Rise time 	Overshoot 	Settling time 	Steady-state?   Stabile
K_p 	Decrease 	Increase 	Small change 	Decrease 	Degrade
K_i 	Decrease 	Increase 	Increase 	Eliminate 	Degrade
K_d 	Minor change 	Decrease 	Decrease 	No effect in theory 	Improve if K_d small
  */ 
  int32_t p = pid_p;
  int32_t i = pid_i;
  int32_t d = pid_d;

  //front right
  initOneWheelPID(1, p << 16, i << 16, d << 16, 255);
  //back left
  initOneWheelPID(2, p << 16, i << 16, d << 16, 1);
  //front left
  initOneWheelPID(0, p << 16, i << 16, d << 16, 1);
  //back right
  initOneWheelPID(3, p << 16, i << 16, d << 16, 255);
}

void freight_lite_wheels::multicastSetWheelSpeeds(int16_t fl, int16_t fr, int16_t bl, int16_t br)
{
  unsigned char idfl = lookup_id("wheel-cnt", "front left");
  unsigned char idfr = lookup_id("wheel-cnt", "front right");
  unsigned char idbl = lookup_id("wheel-cnt", "back left");
  unsigned char idbr = lookup_id("wheel-cnt", "back right");

  packets_485net::packet_485net_dgram wheel_cmd;
  
  wheel_cmd.source = 0xF0;
  wheel_cmd.sport = 7;
  wheel_cmd.dport = 6;
  wheel_cmd.destination = lookup_id("multicast", "all wheels");
  
  wheel_cmd.data.push_back(5);
  wheel_cmd.data.push_back(idfl);
  wheel_cmd.data.push_back(1);
  wheel_cmd.data.push_back((fl >> 0) & 0xFF);
  wheel_cmd.data.push_back((fl >> 8) & 0xFF);
  
  wheel_cmd.data.push_back(5);
  wheel_cmd.data.push_back(idfr);
  wheel_cmd.data.push_back(1);
  wheel_cmd.data.push_back((fr >> 0) & 0xFF);
  wheel_cmd.data.push_back((fr >> 8) & 0xFF);
  
  wheel_cmd.data.push_back(5);
  wheel_cmd.data.push_back(idbl);
  wheel_cmd.data.push_back(1);
  wheel_cmd.data.push_back((bl >> 0) & 0xFF);
  wheel_cmd.data.push_back((bl >> 8) & 0xFF);
  
  wheel_cmd.data.push_back(5);
  wheel_cmd.data.push_back(idbr);
  wheel_cmd.data.push_back(1);
  wheel_cmd.data.push_back((br >> 0) & 0xFF);
  wheel_cmd.data.push_back((br >> 8) & 0xFF);
  
  do
  {
    ROS_INFO("Sending command to %hd wheels (%hd, %hd, %hd, %hd)\n", wheel_cmd.destination, fl, fr, bl, br);
    cmd_pub.publish(wheel_cmd);
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }
  while((wheel_debug_bits[0] & 0x10) != 0x10 || (wheel_debug_bits[1] & 0x10) != 0x10 || (wheel_debug_bits[2] & 0x10) != 0x10 || (wheel_debug_bits[3] & 0x10) != 0x10);
  
  //all got commands now
  wheel_cmd.data.clear();
  wheel_cmd.data.push_back(0xa5);
  wheel_cmd.data.push_back(0x5a);
  wheel_cmd.data.push_back(0xcc);
  wheel_cmd.data.push_back(0x33);
  wheel_cmd.dport = 3;
  
  bool needsfl, needsfr, needsbl, needsbr;
  needsfl = needsfr = needsbl = needsbr = true;
  
  do
  {
    if(needsfl)
    {
      ROS_INFO("Sending sync deassert to fl\n");
      wheel_cmd.destination = idfl;
      cmd_pub.publish(wheel_cmd);
    }
    if(needsfr)
    {
      ROS_INFO("Sending sync deassert to fr\n");
      wheel_cmd.destination = idfr;
      cmd_pub.publish(wheel_cmd);
    }
    if(needsbl)
    {
      ROS_INFO("Sending sync deassert to bl\n");
      wheel_cmd.destination = idbl;
      cmd_pub.publish(wheel_cmd);
    }
    if(needsbr)
    {
      ROS_INFO("Sending sync deassert to br\n");
      wheel_cmd.destination = idbr;
      cmd_pub.publish(wheel_cmd);
    }
    
    ros::Duration(0.01).sleep();
    
    ros::spinOnce();
    
    needsfl = (wheel_debug_bits[0] & 0x10) != 0;
    needsfr = (wheel_debug_bits[1] & 0x10) != 0;
    needsbl = (wheel_debug_bits[2] & 0x10) != 0;
    needsbr = (wheel_debug_bits[3] & 0x10) != 0;
    ROS_INFO("wheels need (%hd, %hd, %hd, %hd)\n", 
	(wheel_debug_bits[0] & 0x10), (wheel_debug_bits[1] & 0x10),
    	(wheel_debug_bits[2] & 0x10), (wheel_debug_bits[3] & 0x10));
  }
  while(needsfl || needsfr || needsbl || needsbr);
}

// only place to set wheel_stopped
// static 
void freight_lite_wheels::wheelCallback(const packets_485net::packet_485net_dgram& ws)
{
  // move to .h file

  uint16_t itmp;
// ROS_INFO("WC 1 : %d  %d %d %d %d", ws.source, lookup_id("wheel-cnt", "front left"), lookup_id("wheel-cnt", "front right") , lookup_id("wheel-cnt", "front right"), lookup_id("wheel-cnt", "back left"));
  if(ws.source != lookup_id("wheel-cnt", "front left") && ws.source != lookup_id("wheel-cnt", "front right") && ws.source != lookup_id("wheel-cnt", "back left") && ws.source != lookup_id("wheel-cnt", "back right"))
    return;
// ROS_INFO("WC 2");
  if(!(ws.destination == 0xF0 || ws.destination == 0x00))
    return;
// ROS_INFO("WC 3");
  if(ws.dport != 7)
    return;
// ROS_INFO("WC 4");
  if(ws.data.size() != 23)
    return;
// ROS_INFO("WC 5");
    
  itmp = ws.data[4] | ((ws.data[5]) << 8);
  ROS_INFO("ws data[4,5] = %d", itmp); 
  if(ws.source == lookup_id("wheel-cnt", "front left"))
  {
    vl[0] = itmp;
    wheel_debug_bits[0] = ws.data[22];
  }
  else if(ws.source == lookup_id("wheel-cnt", "front right"))
  {
    vr[0] = itmp;
    wheel_debug_bits[1] = ws.data[22];
  }
  else if(ws.source == lookup_id("wheel-cnt", "back left"))
  {
    vl[1] = itmp;
    wheel_debug_bits[2] = ws.data[22];
  }
  else if(ws.source == lookup_id("wheel-cnt", "back right"))
  {
    vr[1] = itmp;
    wheel_debug_bits[3] = ws.data[22];
  }
  itmp = ws.data[20] | ((ws.data[21]) << 8);
  ROS_INFO("ws data[20,21] (want 3k) = %d", itmp); 


  if(rosinfodbg)
    ROS_INFO("debug bits %02X %02X %02X %02X", wheel_debug_bits[0], wheel_debug_bits[1], wheel_debug_bits[2], wheel_debug_bits[3]);
  //int addr = ws.srcaddr / 2 - 1;
  //vl[addr] = ws.ticks0_interval;
  //vr[addr] = ws.ticks1_interval;  
  wheel_stopped = (0 == vl[0] && 0 == vl[1] && 0 == vr[0] && 0 == vr[1]);
  if (rosinfodbg) ROS_INFO("wheel_stopped %d %d %d %d", vl[0], vl[1], vr[0], vr[1]);
  wheel_started[0] = vl[0] != 0;    //fl
  wheel_started[1] = vr[0] != 0;    //fr
  wheel_started[2] = vl[1] != 0;    //bl
  wheel_started[3] = vr[1] != 0;    //br
  // state_change();
}

double freight_lite_wheels::getVelocity(int i)
{
  // ARD: need to convert to a double????
  if (i == 0) return vl[0];  // front left
  if (i == 1) return vr[0];  // front right
  if (i == 2) return vl[1];  // back left
  if (i == 3) return vr[1];  // back right
  return -1;
}

void freight_lite_wheels::initWheels(void)
{

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  // Hopefully already called in main()
  // ros::init();
  ros::NodeHandle nh;

  idlookup = nh.serviceClient<net_485net_id_handler::SearchID>("/search_id", true);
  idlookup.waitForExistence();


  wheel_sub = nh.subscribe("net_485net_incoming_dgram", 1000, &freight_lite_wheels::wheelCallback, this);

  cmd_pub = nh.advertise<packets_485net::packet_485net_dgram>("net_485net_outgoing_dgram", 1000);
  
  // nh.param("/base_controller/pid_p", pid_p, 11);
  // nh.param("/base_controller/pid_i", pid_i, 6);
  // nh.param("/base_controller/pid_d", pid_d, 0);
  nh.param("/base_controller/pid_p", pid_p, 1);
  nh.param("/base_controller/pid_i", pid_i, 8);
  nh.param("/base_controller/pid_d", pid_d, 1);
  initWheelPID();
}

}
