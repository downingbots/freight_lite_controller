// NOTE: The contents of this file have been taken largely from the ros_control wiki tutorials

// #include <freight_lite/freight_lite.h>
#include <freight_lite/freight_lite.h>
#include <freight_lite/freight_lite_controller.h>
#include <chrono>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "freight_lite");
  ros::NodeHandle nh;
  // ros::NodeHandle controller_nh;

  // This should be set in launch files as well
  // nh.setParam("/use_sim_time", true);

  FreightLite robot;
  ROS_WARN_STREAM("period: " << robot.getPeriod().toSec());

  // ARD: The following didn't work with the new robot_control interface.
  // Error: "Controller Spawner couldn't find the expected controller_manager 
  //        ROS interface."
  // controller_manager::ControllerManager cm(&robot, nh);

  freight_lite::FreightLiteController freight_lite_controller_;
  freight_lite_controller_.init(&robot, nh, nh);
  // freight_lite_controller_.init(&robot, nh, controller_nh);
/*
  void FreightLiteController::update(const ros::Time& time, const ros::Duration& period)
  void FreightLiteController::starting(const ros::Time& time)
  void FreightLiteController::stopping(const ros::Time& )
  void FreightLiteController::updateOdometry(const ros::Time& time)
  void FreightLiteController::updateCommand(const ros::Time& time, const ros::Duration& period)
  void FreightLiteController::brake()
  void FreightLiteController::cmdVelCallback(const geometry_msgs::Twist& command)
  void FreightLiteController::cmdFreightLiteCallback(const four_wheel_steering_msgs::FourWheelSteering& command)
  void FreightLiteController::adjustSteeringCallback(const std_msgs::Int16& wheel_mode)
  void FreightLiteController::setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
*/


  ros::Publisher clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::chrono::system_clock::time_point begin = std::chrono::system_clock::now();
  std::chrono::system_clock::time_point end   = std::chrono::system_clock::now();

  ros::Time internal_time(0);
  const ros::Duration dt = robot.getPeriod();
  double elapsed_secs = 0;

  while(ros::ok())
  {
    begin = std::chrono::system_clock::now();

    robot.read();
    // ARD: bypassing cm
    // cm.update(internal_time, dt);
    freight_lite_controller_.update(internal_time, dt);
    robot.write();

    end = std::chrono::system_clock::now();

    elapsed_secs = std::chrono::duration_cast<std::chrono::duration<double> >((end - begin)).count();

    if (dt.toSec() - elapsed_secs < 0.0)
    {
      ROS_WARN_STREAM_THROTTLE(
            0.1, "Control cycle is taking to much time, elapsed: " << elapsed_secs);
    }
    else
    {
      ROS_DEBUG_STREAM_THROTTLE(1.0, "Control cycle is, elapsed: " << elapsed_secs);
      usleep((dt.toSec() - elapsed_secs) * 1e6);
    }

    rosgraph_msgs::Clock clock;
    clock.clock = ros::Time(internal_time);
    clock_publisher.publish(clock);
    internal_time += dt;
  }
  spinner.stop();

  return 0;
}
