#include <sstream>
#include <gtest/gtest.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

#include "robot_models.hpp"

void convert(const std::vector<std::vector<double>>& q_ins, std::vector<KDL::JntArray>* q_outs)
{
  std::vector<KDL::JntArray> converted;

  for (const auto& vec : q_ins)
  {
    KDL::JntArray q(vec.size());
    for (size_t i = 0; i < vec.size(); ++i)
    {
      q(i) = vec[i];
    }

    converted.push_back(q);
  }

  *q_outs = converted;
}

std::string toString(const KDL::JntArray& q)
{
  std::stringstream ss;
  for (unsigned int i = 0; i < q.rows(); ++i)
  {
    ss << q(i);
    if (i < q.rows() - 1) { ss << ", "; }
  }

  return ss.str();
}

std::string toString(const KDL::Frame& f)
{
  std::stringstream ss;

  ss << f.M.data[0] << "  " << f.M.data[1] << "  " << f.M.data[2] << "  " << f.p.x() << "\n";
  ss << f.M.data[3] << "  " << f.M.data[4] << "  " << f.M.data[5] << "  " << f.p.y() << "\n";
  ss << f.M.data[6] << "  " << f.M.data[7] << "  " << f.M.data[8] << "  " << f.p.z() << "\n";
  ss << 0           << "  " << 0           << "  " << 0           << "  " << 1       << "\n";

  return ss.str();
}

TEST(robot_model_iiwa, robot_model_iiwa_fk_test) {

  auto robotModel = sns_ik::robot_models::RobotModel::Iiwa;

  std::vector<std::string> jointNames;
  KDL::Chain chain = sns_ik::robot_models::getKdlChain(robotModel, &jointNames);
  KDL::JntArray qLow, qUpp, vMax, aMax;
  sns_ik::robot_models::getJointLimits(robotModel, &qLow, &qUpp, &vMax, &aMax);

  // Create a forward-kinematics solver:
  KDL::ChainFkSolverPos_recursive fwdKin(chain);

  std::vector< std::vector<double> > q_ins_d = {
    { 0, 0.785, 0, -1.571, 0, 0.785, 0 },
    { 0, 0, 0, 0, 0, 0, 0 },
    { 0, -0.785, -2.967, -1.571, 2.967, -0.785, 0 },
    { 0, 0.785, 0, -0.817, 0, 0.785, 0 },
  };

  std::vector<KDL::JntArray> q_ins;
  convert(q_ins_d, &q_ins);

  KDL::Frame p_soln;

  for (const auto& q_in : q_ins)
  {
    int result = fwdKin.JntToCart(q_in, p_soln);
    std::cout << "FK for joints [" << toString(q_in) << "]:\n";
    if (result < 0)
    {
      std::cout << "Error: " << fwdKin.strError(result) << std::endl;
    }
    else
    {
      std::cout << toString(p_soln) << std::endl;
    }
  }
}

/*************************************************************************************************/
// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
