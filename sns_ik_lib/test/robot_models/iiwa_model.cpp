/** @file iiwa_model.cpp
 *
 * @brief The file provides simple functions to return robot models for testing.
 *
 * @author Chris Smith
 * @author Matthew Kelly
 *
 * This file provides a set of functions that return simple kinematic chains that are used for the
 * unit tests. This allows unit tests to run quickly without depending on external URDF files.
 *
 *    Copyright 2018 Rethink Robotics
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#include "iiwa_model.hpp"

#include <ros/console.h>

namespace sns_ik {
namespace iiwa_model {

/*************************************************************************************************/

KDL::Chain getKdlChain(std::vector<std::string>* jointNames)
{
  if (jointNames){
    *jointNames = {"iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"};
  }

  using namespace KDL;
  Vector axis, origin;
  Joint jnt;
  Chain chain;

  const double PI = 3.14159265359;

  chain.addSegment(Segment("iiwa_link_0", Joint(Joint::None), Frame(Rotation::RPY(0, 0, 0), Vector(0, 0, 0.15))));
  chain.addSegment(Segment("iiwa_link_1", Joint(Joint::RotZ), Frame(Rotation::RPY(PI / 2, 0, PI), Vector(0, 0, 0.19))));
  chain.addSegment(Segment("iiwa_link_2", Joint(Joint::RotZ), Frame(Rotation::RPY(PI / 2, 0, PI), Vector(0, 0.21, 0))));
  chain.addSegment(Segment("iiwa_link_3", Joint(Joint::RotZ), Frame(Rotation::RPY(PI / 2, 0, 0), Vector(0.0, 0, 0.19))));
  chain.addSegment(Segment("iiwa_link_4", Joint(Joint::RotZ), Frame(Rotation::RPY(-PI / 2, PI, 0), Vector(0, 0.21, 0))));
  chain.addSegment(Segment("iiwa_link_5", Joint(Joint::RotZ), Frame(Rotation::RPY(PI / 2, 0, 0), Vector(0, 0.06070, 0.19))));
  chain.addSegment(Segment("iiwa_link_6", Joint(Joint::RotZ), Frame(Rotation::RPY(-PI / 2, PI, 0), Vector(0, 0.081, 0.06070))));
  chain.addSegment(Segment("iiwa_link_7", Joint(Joint::RotZ), Frame(Rotation::RPY(0.0,0.0,0.0), Vector(0.0, 0.0, 0.045))));
  chain.addSegment(Segment("iiwa_link_ee", Joint(Joint::None), Frame(Rotation::RPY(0.0,0.0,0.0), Vector(0.0, 0.0, 0.0))));

  return chain;
}

/*************************************************************************************************/

void getJointLimits(KDL::JntArray* qLow, KDL::JntArray* qUpp,
                    KDL::JntArray* vMax, KDL::JntArray* aMax)
{
  if (!qLow || !qUpp || !vMax || !aMax) { ROS_ERROR("Bad input!"); return; }
  int nJnt = 7;  // Iiwa has seven joints
  *qLow = KDL::JntArray(nJnt);
  *qUpp = KDL::JntArray(nJnt);
  *vMax = KDL::JntArray(nJnt);
  *aMax = KDL::JntArray(nJnt);

  // NOTE: it's unclear how accurate the velocity limits are. Acceleration limits are just double Sawyer's

  (*qLow)(0) = -2.96705972839;
  (*qUpp)(0) = 2.96705972839;
  (*vMax)(0) = 10;
  (*aMax)(0) = 16.0;
  (*qLow)(1) = -2.09439510239;
  (*qUpp)(1) = 2.09439510239;
  (*vMax)(1) = 10;
  (*aMax)(1) = 16.0;
  (*qLow)(2) = -2.96705972839;
  (*qUpp)(2) = 2.96705972839;
  (*vMax)(2) = 10;
  (*aMax)(2) = 16.0;
  (*qLow)(3) = -2.09439510239;
  (*qUpp)(3) = 2.09439510239;
  (*vMax)(3) = 10;
  (*aMax)(3) = 16.0;
  (*qLow)(4) = -2.96705972839;
  (*qUpp)(4) = 2.96705972839;
  (*vMax)(4) = 10;
  (*aMax)(4) = 20.0;
  (*qLow)(5) = -2.09439510239;
  (*qUpp)(5) = 2.09439510239;
  (*vMax)(5) = 10;
  (*aMax)(5) = 20.0;
  (*qLow)(6) = -3.05432619099;
  (*qUpp)(6) = 3.05432619099;
  (*vMax)(6) = 10;
  (*aMax)(6) = 20.0;
}

/*************************************************************************************************/

}  // namespace sns_ik
}  // namespace sawyer_model
