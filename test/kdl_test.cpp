#include "ros/ros.h"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <urdf/model.h>

#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kdl_test", ros::init_options::AnonymousName);
  KDL::Chain chain;
  KDL::Tree tree;
  urdf::Model model;
  model.initParam("/my_gen3/robot_description");

  if (!kdl_parser::treeFromUrdfModel(model, tree))
  {
    const std::string error_string("Failed to parse robot_description parameter to build the kinematic tree!"); 
    ROS_ERROR("%s", error_string.c_str());
    throw(std::runtime_error(error_string));
  }
  if (!tree.getChain("base_link", "tool_frame", chain))
  {
    const std::string error_string("Failed to get kdl chain from the kinematic tree!"); 
    ROS_ERROR("%s", error_string.c_str());
    throw(std::runtime_error(error_string));
  }

  std::unique_ptr<KDL::ChainIkSolverVel_pinv> iksolver_vel;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fksolver;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacsolver;
  iksolver_vel.reset(new KDL::ChainIkSolverVel_pinv(chain));
  fksolver.reset(new KDL::ChainFkSolverPos_recursive(chain));
  jacsolver.reset(new KDL::ChainJntToJacSolver(chain));

  std::cout << chain.getNrOfJoints() << std::endl;

  return 0;
}