#include <iostream>

#include "ros/ros.h"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <urdf/model.h>

#include <kinova_teleop/kinova_controller.hpp>
#include <thread>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinova_controller_node", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  KinovaController kc(nh);
  std::thread main_thread(&KinovaController::main, &kc);

  ros::spin();
  return 0;
}

