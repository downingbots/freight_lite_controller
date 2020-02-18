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
    steering_servo_.initServos();
    // Connect and register the joint state and velocity interface
    for (unsigned int i = 0; i < velocity_joints_name.size(); ++i)
    {

      hardware_interface::JointStateHandle state_handle(velocity_joints_name[i], &wheel_joints_[i].position, &wheel_joints_[i].velocity, &wheel_joints_[i].effort);
      jnt_state_interface_.registerHandle(state_handle);

      hardware_interface::JointHandle vel_handle(state_handle, &wheel_joints_[i].velocity_command);
      jnt_vel_interface_.registerHandle(vel_handle);
    }

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
    for (unsigned int i = 0; i < 3; ++i)
    {
      os << wheel_joints_[i].velocity_command << ", ";
    }
    os << wheel_joints_[3].velocity_command;

    ROS_DEBUG_STREAM("Commands for joints: " << os.str());
    // ROS_INFO_STREAM("Commands for joints: " << os.str());

    os.str("");
    for (unsigned int i = 0; i < 4; ++i)
    {
      // if (wheel_joints_[i].velocity_command != 0) 
      {
        os << steering_joints_[i].position_command << ", ";
        // steering_servo_.servoSetTarget(i, steering_joints_[i].position_command);
      }
    }
    steering_servo_.servoSetAllTargets(
                      steering_joints_[0].position_command,
                      steering_joints_[1].position_command,
                      steering_joints_[2].position_command,
                      steering_joints_[3].position_command);
    ROS_DEBUG_STREAM("Commands for steering joints: " << os.str());
    // ROS_INFO_STREAM("Commands for steering joints: " << os.str());
  }

  void write()
  {
    std::ostringstream os;

    if (running_)
    {
      for (unsigned int i = 0; i < 4; ++i)
      {
        // Note that joints_[i].position will be NaN for one more cycle after we start(),
        // but that is consistent with the knowledge we have about the state
        // of the robot.
        wheel_joints_[i].position += wheel_joints_[i].velocity*getPeriod().toSec(); // update position
        wheel_joints_[i].velocity = wheel_joints_[i].velocity_command; // might add smoothing here later
        os << wheel_joints_[i].position << ", ";
        
      }
      // ROS_INFO_STREAM("wheel position: " << os.str());
      for (unsigned int i = 0; i < 4; ++i)
      {
        // steering_joints_[i].position = steering_joints_[i].position_command; 
        // might add smoothing here later
        // steering_joints_[i].position = servo.servoGetPosition(steering_joints_[i])
        steering_joints_[i].position = steering_servo_.servoGetPosition(i);
        os << steering_joints_[i].position << ", ";
      }
      // ROS_INFO_STREAM("Position steering joints: " << os.str());
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

  freight_lite::micro_maestro steering_servo_;  // ARD: search for steering_servo_
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
