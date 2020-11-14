// NOTE: The contents of this file have been taken largely from the ros_control wiki tutorials

// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

// servo controller
#include <freight_lite/micro_maestro.h>

#include <stdint.h>
#include <packets_485net/packet_485net_dgram.h>
#include <freight_lite/freight_lite_wheels.h>

// NaN
#include <limits>

// ostringstream
#include <sstream>

class FreightLite : public hardware_interface::RobotHW
{
public:

  FreightLite()
  : running_(true)
  , start_srv_(nh_.advertiseService("start", &FreightLite::start_callback, this))
  , stop_srv_(nh_.advertiseService("stop", &FreightLite::stop_callback, this))
  {
    std::vector<std::string> velocity_joints_name = {"front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"};
    servos_.initServos();
    wheel_velocity_.initWheels();
    // Connect and register the joint state and velocity interface

    for (unsigned int i = 0; i < velocity_joints_name.size(); ++i)
    {

      hardware_interface::JointStateHandle state_handle(velocity_joints_name[i], &wheel_joints_[i].position, &wheel_joints_[i].velocity, &wheel_joints_[i].effort);
      jnt_state_interface_.registerHandle(state_handle);

      hardware_interface::JointHandle vel_handle(state_handle, &wheel_joints_[i].velocity_command);
      jnt_vel_interface_.registerHandle(vel_handle);
    }
//  FreightLiteController::init(hardware_interface::RobotHW *robot_hw,
//              ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh)


    std::vector<std::string> position_joints_name = {"front_left_steering_joint", "front_right_steering_joint", "rear_left_steering_joint", "rear_right_steering_joint"};
    // Connect and register the joint state and position interface
    for (unsigned int i = 0; i < position_joints_name.size(); ++i)
    {
      hardware_interface::JointStateHandle state_handle(position_joints_name[i], &steering_joints_[i].position, &steering_joints_[i].velocity, &steering_joints_[i].effort);
      jnt_state_interface_.registerHandle(state_handle);

      hardware_interface::JointHandle pos_handle(state_handle, &steering_joints_[i].position_command);
      jnt_pos_interface_.registerHandle(pos_handle);
    }

    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_vel_interface_);
    registerInterface(&jnt_pos_interface_);
  }

  ros::Time getTime() const {return ros::Time::now();}
  // ARD: shouldn't this be based upon the publish_rate param?
  // ros::Duration getPeriod() const {return ros::Duration(0.01);}
  ros::Duration getPeriod() const {return ros::Duration(0.1);}

  void read()
  {
    std::ostringstream os;
    int16_t fl, fr, bl, br;

    for (unsigned int i = 0; i < 3; ++i)
    {
      os << wheel_joints_[i].velocity_command << ", ";
    }
    os << wheel_joints_[3].velocity_command;

    ROS_DEBUG_STREAM("Commands for joints: " << os.str());
    ROS_INFO_STREAM("Commands for joints: " << os.str());

    os.str("");
    for (unsigned int i = 0; i < 4; ++i)
    {
      // if (wheel_joints_[i].velocity_command != 0) 
      {
        os << steering_joints_[i].position_command << ", ";
        // servos_.servoSetTarget(i, steering_joints_[i].position_command);
      }
    }
    servos_.servoSetAllTargets(
                      steering_joints_[0].position_command,
                      steering_joints_[1].position_command,
                      steering_joints_[2].position_command,
                      steering_joints_[3].position_command);
    ROS_DEBUG_STREAM("Commands for steering joints: " << os.str());
    ROS_INFO_STREAM("Commands for steering joints: " << os.str());

    // ARD: velocity_command is a double; fl,fr,bl,br are int16; how to convert?
    os.str("");
    for (unsigned int i = 0; i < 4; ++i)
    {
        os << wheel_joints_[i].velocity_command << ", ";
    }
    ROS_INFO_STREAM("Commands for wheel velocity: " << os.str());
    fl = wheel_joints_[0].velocity_command;
    fr = wheel_joints_[1].velocity_command;
    bl = wheel_joints_[2].velocity_command;
    br = wheel_joints_[3].velocity_command;
    wheel_velocity_.multicastSetWheelSpeeds(fl, fr, bl, br);
    ROS_INFO_STREAM("Sent multicast");

  }

  void write()
  {
    std::ostringstream os;

    if (!running_) {
      running_ = true;  // ARD hack
      ROS_INFO_STREAM("write: set running_ to True");
    }
    if (running_)
    {
      for (unsigned int i = 0; i < 4; ++i)
      {
        // Note that joints_[i].position will be NaN for one more cycle after we start(),
        // but that is consistent with the knowledge we have about the state
        // of the robot.

        // ARD: velocity_command is a double; fl,fr,bl,br are int16; how to convert?
        wheel_joints_[i].velocity = wheel_velocity_.getVelocity(i);
        wheel_joints_[i].position += wheel_joints_[i].velocity*getPeriod().toSec(); // update position

        // ARD: the following was a temporary hack before integrating pr2lite_base???
        // wheel_joints_[i].velocity = wheel_joints_[i].velocity_command; // might add smoothing here later
        os << wheel_joints_[i].position << ", ";
        
      }
      ROS_INFO_STREAM("wheel position: " << os.str());
      os.str("");
      for (unsigned int i = 0; i < 4; ++i)
      {
        // steering_joints_[i].position = steering_joints_[i].position_command; 
        // might add smoothing here later
        // steering_joints_[i].position = servo.servoGetPosition(steering_joints_[i])
        steering_joints_[i].position = servos_.servoGetPosition(i);
        os << steering_joints_[i].position << ", ";
      }
      ROS_INFO_STREAM("Position steering joints: " << os.str());
    }
    else
    {
      for (unsigned int i = 0; i < 4; ++i)
      {
        wheel_joints_[i].position = std::numeric_limits<double>::quiet_NaN();
        wheel_joints_[i].velocity = std::numeric_limits<double>::quiet_NaN();
      }
      for (unsigned int i = 0; i < 4; ++i)
      {
        steering_joints_[i].position = std::numeric_limits<double>::quiet_NaN();
        steering_joints_[i].velocity = std::numeric_limits<double>::quiet_NaN();
      }
    }
  }

  bool start_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = true;
    return true;
  }

  bool stop_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = false;
    return true;
  }

  freight_lite::micro_maestro servos_;  // ARD: search for servos_
  freight_lite::freight_lite_wheels wheel_velocity_;  // ARD: search for wheel_velocity_


private:
  hardware_interface::JointStateInterface    jnt_state_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;

  struct WheelJoint
  {
    double position;
    double velocity;
    double effort;
    double velocity_command;

    WheelJoint() : position(0), velocity(0), effort(0), velocity_command(0) { }
  } wheel_joints_[4];

  struct SteeringJoint
  {
    double position;
    double velocity;
    double effort;
    double position_command;

    SteeringJoint(): position(0), velocity(0), effort(0), position_command(0) {}
  } steering_joints_[4];

  bool running_;

  ros::NodeHandle nh_;
  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;
};
