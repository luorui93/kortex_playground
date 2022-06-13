#ifndef KINOVA_CONTROLLER_H
#define KINOVA_CONTROLLER_H

// ROS
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>

// KDL
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <urdf/model.h>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>

// Eigen
#include <Eigen/Dense>

// Standard
#include <mutex>

enum ErrorCode
{
  NOERROR = 0
};

class KinovaController
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber robot_state_sub;

  void cb_joint_states(const sensor_msgs::JointState::ConstPtr& state);

  std::mutex m_state_mutex;

  // Data member
  sensor_msgs::JointState m_current_state;
  KDL::Chain m_chain;
  KDL::Tree m_tree;
  urdf::Model m_model;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> iksolver_vel;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fksolver;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacsolver;
  std::unique_ptr<KDL::ChainDynParam> dynsolver;
  bool m_first_state_received;
  int  m_dof;

public:
  KinovaController(ros::NodeHandle&);
  ~KinovaController();
  int main();
  int eigen_svd(const Eigen::MatrixXf& m);
  int kdl_svd(const KDL::Jacobian& jac, KDL::JntArray& S, std::vector<KDL::JntArray>& U, std::vector<KDL::JntArray>& V);
  int compute_inertia_matrix_ee(const KDL::JntSpaceInertiaMatrix &M, const KDL::Jacobian& jac, Eigen::MatrixXf& M_ee);
  int jnt_array_to_eigen(const std::vector<KDL::JntArray>& jnt_m, Eigen::MatrixXf& m);
};

#endif