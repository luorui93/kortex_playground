#include <kinova_teleop/kinova_controller.hpp>
#include <chrono>
#include <kdl/utilities/svd_HH.hpp>
#include <thread>

KinovaController::KinovaController(ros::NodeHandle& nh): nh_(nh),
                                                         m_first_state_received(false),
                                                         m_dof(7)
{
  robot_state_sub = nh_.subscribe<sensor_msgs::JointState>
    ("/my_gen3/joint_states", 1, &KinovaController::cb_joint_states, this);  
  m_model.initParam("/my_gen3/robot_description");
  if (!kdl_parser::treeFromUrdfModel(m_model, m_tree))
  {
    const std::string error_string("Failed to parse robot_description parameter to build the kinematic tree!"); 
    ROS_ERROR("%s", error_string.c_str());
    throw(std::runtime_error(error_string));
  }
  if (!m_tree.getChain("base_link", "tool_frame", m_chain))
  {
    const std::string error_string("Failed to get kdl chain from the kinematic tree!"); 
    ROS_ERROR("%s", error_string.c_str());
    throw(std::runtime_error(error_string));
  }
  iksolver_vel.reset(new KDL::ChainIkSolverVel_pinv(m_chain));
  fksolver.reset(new KDL::ChainFkSolverPos_recursive(m_chain));
  jacsolver.reset(new KDL::ChainJntToJacSolver(m_chain));
  dynsolver.reset(new KDL::ChainDynParam(m_chain, KDL::Vector(0, 0, -9.81)));

  std::cout << "Read from URDF model" << std::endl;
  std::cout << "Robot has " << m_chain.getNrOfJoints() << " joints" << std::endl;

}

KinovaController::~KinovaController()
{
}

void KinovaController::cb_joint_states(const sensor_msgs::JointState::ConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(m_state_mutex);
  m_first_state_received = true;
  m_current_state = *msg;
}

int KinovaController::main()
{
  while(!m_first_state_received)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  sensor_msgs::JointState current_joint_state;
  {
    std::lock_guard<std::mutex> lock(m_state_mutex);
    current_joint_state = m_current_state;

  } 
  KDL::JntArray current_q;
  current_q.data = Eigen::VectorXd(m_dof);
  for (int i = 0; i < m_dof; i++)
  {
    current_q.data[i] = current_joint_state.position[i];
  }
  KDL::Jacobian jac(m_dof);
  // std::cout << current_q.data << std::endl;
  auto err = jacsolver->JntToJac(current_q, jac);
  if (err < jacsolver->E_NOERROR) 
  {
    ROS_ERROR_STREAM("Jacobian solver error: " << jacsolver->strError(err) << std::endl);
    return err;
  }
  Eigen::MatrixXf eigen_jac = jac.data.cast<float>();

  using namespace std::chrono;

  // SVD
  auto start = high_resolution_clock::now();
  eigen_svd(eigen_jac);
  auto end = high_resolution_clock::now();
  auto time_span = duration_cast<microseconds> (end - start);
  std::cout << "Eigen SVD took: " << time_span.count() << "us" << std::endl;

  start = high_resolution_clock::now();
  KDL::JntArray S(m_dof);
  std::vector<KDL::JntArray> U(6, KDL::JntArray(m_dof));
  std::vector<KDL::JntArray> V(m_dof, KDL::JntArray(m_dof));
  kdl_svd(jac, S, U, V);
  end = high_resolution_clock::now();
  time_span = duration_cast<microseconds> (end - start);
  std::cout << "KDL SVD took: " << time_span.count() << "us" << std::endl;

  Eigen::MatrixXf UM;
  start = high_resolution_clock::now();
  err = jnt_array_to_eigen(U, UM);
  if (err < ErrorCode::NOERROR)
  {
    ROS_ERROR_STREAM("Eigen matrix conversion error" << std::endl);
    return -1;
  }
  end = high_resolution_clock::now();
  time_span = duration_cast<microseconds> (end - start);
  // for (auto array : U)
  // {
  //   std::cout << array.data << std::endl;
  // }
  // std::cout << "============================================" << std::endl;
  std::cout << "Conversion took: " << time_span.count() << "us" << std::endl;

  // Operational space inertia matrix 
  start = high_resolution_clock::now();
  KDL::JntSpaceInertiaMatrix M_q(m_dof);
  err = dynsolver->JntToMass(current_q, M_q);
  if (err < jacsolver->E_NOERROR)
  {
    ROS_ERROR_STREAM("ID solver error: " << jacsolver->strError(err) << std::endl);
    return err;
  }
  end = high_resolution_clock::now();
  time_span = duration_cast<microseconds> (end - start);
  std::cout << "KDL Inertia matrix took: " << time_span.count() << "us" << std::endl;
  // std::cout << M_q.data << std::endl;




  return 0;
}

int KinovaController::eigen_svd(const Eigen::MatrixXf& m)
{
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
  // std::cout << "Singular values are " << svd.singularValues() << std::endl;
  return 0;
}

int KinovaController::kdl_svd(const KDL::Jacobian& jac, KDL::JntArray& S, std::vector<KDL::JntArray>& U, std::vector<KDL::JntArray>& V)
{
  KDL::SVD_HH svd_hh(jac);
  svd_hh.calculate(jac, U, S, V, 150);
  // std::cout << "Singular values are " << S.data << std::endl;
  return 0;
}

int KinovaController::compute_inertia_matrix_ee(const KDL::JntSpaceInertiaMatrix& M, const KDL::Jacobian& jac, Eigen::MatrixXf& M_ee)
{
  return 0;
}

int KinovaController::jnt_array_to_eigen(const std::vector<KDL::JntArray>& jnt_m, Eigen::MatrixXf& m)
{
  if (jnt_m.size() == 0)
  {
    ROS_ERROR("Joint array matrix is empty");
    return -1;
  }
  // JntArray matrix is stored in Row-major format
  m.resize(jnt_m.size(), jnt_m[0].rows());
  int d = 0;
  for (int i = 0; i < jnt_m[0].rows(); i++)
  {
    for (int j = 0; j < jnt_m.size(); j++)
    {
      *(m.data() + d) = jnt_m[j](i);
      d++;
    }
  }

  // std::cout << m << std::endl;

  return ErrorCode::NOERROR;
}