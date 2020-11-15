# freight_lite_controller

Back in 2011, Frank Ou and I mentored 4 high school students in the building of PR2-Lite.  PR2-Lite was inspired by Willow Garage's PR2.  Details of PR2-Lite can be found at: https://mattdowning.wordpress.com/pr2-lite/ 

Unfortunately, the weight of the upper body of PR2-Lite proved to be too heavy for its pseudo-omnidirectional base.  PR2-Lite is slowly being brought back to life, and its base has been replaced by a ubiquity robot ( https://www.ubiquityrobotics.com/ ). The old base of PR2-Lite was now available for repurposing, and given that it somewhat resembles a Freight robot ( https://fetchrobotics.com/robotics-platforms/freight-base/ ), the base has been renamed Freight-Lite.

Freight-Lite has been enhanced to have 4 servos with servocity gearboxes to rotate the wheels for pseudo-omnidirectional driving. It also has an optional extending arm based upon https://www.servocity.com/cascading-x-rail-slide-kit/ that is attached to a vertical low-end linear actuator built from servocity parts.

The software is based upon the PR2-Lite ROS software. Initially, the goal is simple joystick-based RC control.
