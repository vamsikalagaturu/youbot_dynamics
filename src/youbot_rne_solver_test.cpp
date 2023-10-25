#include <array>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "kdl/jntarray.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainidsolver_recursive_newton_euler.hpp"
#include "kdl/kinfam_io.hpp"

// #include "youbot_driver/youbot/EthercatMasterInterface.hpp"
#include "youbot_driver/youbot/EthercatMasterWithThread.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"

#include "ros/ros.h"

int main(int argc, char** argv)
{

  std::string robot_urdf = "urdf/youbot_model.urdf";

  // set the base and tool links
  std::string base_link = "arm_link_0";
  std::string tool_link = "arm_link_5";

  KDL::Tree robot_tree;
  KDL::Chain robot_chain;

  // load the robot URDF into the KDL tree
  if (!kdl_parser::treeFromFile(robot_urdf, robot_tree))
  {
    return -1;
  }

  // create the KDL chain
  if (!robot_tree.getChain(base_link, tool_link, robot_chain))
  {
    return -1;
  }

  // number of joints
  int n_joints = robot_chain.getNrOfJoints();

  // number of segments
  int n_segments = robot_chain.getNrOfSegments();

  std::cout << "Number of joints: " << n_joints << std::endl;
  std::cout << "Number of segments: " << n_segments << std::endl;

  youbot::EthercatMaster::getInstance("youbot-ethercat.cfg", "/home/batsy/bitbots/src/youbot_driver/config/", true);

  // initialize youbot arm
  youbot::YouBotManipulator myArm("youbot-manipulator");
  myArm.doJointCommutation();
  std::cout << "Calibrating the arm!" << std::endl;
  myArm.calibrateManipulator();

  int oneSec = 1000000;
  int counter = 0;
  while (counter < 1) {
    usleep(oneSec);
    counter++;
  }

  // read joint angles
  std::vector<youbot::JointSensedAngle> joint_angles;
  std::vector<youbot::JointTorqueSetpoint> commandTorques;

  youbot::JointTorqueSetpoint desiredJointTorque;
  desiredJointTorque.torque = 0.0 * si::newton_meters;
  commandTorques.push_back(desiredJointTorque);
  commandTorques.push_back(desiredJointTorque);
  commandTorques.push_back(desiredJointTorque);
  commandTorques.push_back(desiredJointTorque);
  commandTorques.push_back(desiredJointTorque);

  myArm.getJointData(joint_angles);

  for (int i = 0; i < n_joints; i++)
  {
    std::cout << "Joint " << i << " angle: " << joint_angles[i].angle.value() << std::endl;
  }

  KDL::ChainIdSolver_RNE idsolver = KDL::ChainIdSolver_RNE(robot_chain, KDL::Vector(0, 0, -9.81));

  KDL::JntArray q(n_joints);
  KDL::JntArray qdot(n_joints);
  KDL::JntArray qdotdot(n_joints);
  std::vector<KDL::Wrench> f_ext(n_segments);
  KDL::JntArray torques(n_joints);

  KDL::JntArray q_offsets(n_joints);

  q_offsets(0) = KDL::deg2rad * 169.0;
  q_offsets(1) = KDL::deg2rad * 65.0;
  q_offsets(2) = KDL::deg2rad * -151.0;
  q_offsets(3) = KDL::deg2rad * 102.5;
  q_offsets(4) = KDL::deg2rad * 165.0;

  while (true)
  {

    myArm.getJointData(joint_angles);

    for (int i = 0; i < n_joints; i++)
    {
      q(i) = joint_angles[i].angle.value() - q_offsets(i);
      qdot(i) = 0.0;
      qdotdot(i) = 0.0;
    }

    idsolver.CartToJnt(q, qdot, qdotdot, f_ext, torques);

    std::vector<youbot::JointTorqueSetpoint> commandTorques;
    youbot::JointTorqueSetpoint desiredJointTorque;
    desiredJointTorque.torque = 0.0 * si::newton_meters;
    commandTorques.push_back(desiredJointTorque);
    desiredJointTorque.torque = torques(1) * si::newton_meters;
    commandTorques.push_back(desiredJointTorque);
    desiredJointTorque.torque = torques(2) * si::newton_meters;
    commandTorques.push_back(desiredJointTorque);
    desiredJointTorque.torque = torques(3) * si::newton_meters;
    commandTorques.push_back(desiredJointTorque);
    desiredJointTorque.torque = torques(4) * si::newton_meters;
    commandTorques.push_back(desiredJointTorque);

    // std::cout << "joint q: " << q << std::endl;
    // std::cout << "joint qdot: " << qdot << std::endl;
    // std::cout << "joint qdotdot: " << qdotdot << std::endl;
    std::cout << "joint torques: " << torques << std::endl;


    myArm.setJointData(commandTorques);
    
    usleep(10000);
  }
  
  return 0;
}