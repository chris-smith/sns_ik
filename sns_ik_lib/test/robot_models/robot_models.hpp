/** @file robot_models.hpp
 *
 * @brief The file provides simple functions to return robot models for testing.
 *
 * @author Chris Smith
 * @author Matt Kelly
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


 #ifndef SNS_IK_ROBOT_MODELS_H_
 #define SNS_IK_ROBOT_MODELS_H_

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

#include "iiwa_model.hpp"
#include "sawyer_model.hpp"

#include <ros/console.h>

namespace sns_ik {
namespace robot_models {

enum class RobotModel
{
  Sawyer = 0,
  Iiwa
};

KDL::Chain getKdlChain(RobotModel robot, std::vector<std::string>* jointNames)
{
  switch (robot)
  {
    case RobotModel::Sawyer:
      return sns_ik::sawyer_model::getKdlChain(jointNames);
    case RobotModel::Iiwa:
      return sns_ik::iiwa_model::getKdlChain(jointNames);
    default:
      // invalid
      return KDL::Chain();
  }  
}

/*
* This function returns the joint limits for the sawyer robot arm
* Full model description at: https://github.com/RethinkRobotics/sawyer_robot
* @param[out] qLow: lower bound on joint angles
* @param[out] qUpp: upper bound on joint angles
* @param[out] vMax: maximum joint speed (symmetric)
* @param[out] aMax: maximum joint acceleration (symmetric)
*/
void getJointLimits(RobotModel robot,
                    KDL::JntArray* qLow, KDL::JntArray* qUpp,
                    KDL::JntArray* vMax, KDL::JntArray* aMax)
{
  switch (robot)
  {
    case RobotModel::Sawyer:
      return sns_ik::sawyer_model::getJointLimits(qLow, qUpp, vMax, aMax);
    case RobotModel::Iiwa:
      return sns_ik::iiwa_model::getJointLimits(qLow, qUpp, vMax, aMax);
  }
}

}
}

#endif // SNS_IK_ROBOT_MODELS_H_
