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

#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include "kdl/chainhdsolver_vereshchagin.hpp"
#include "kdl/chainidsolver_recursive_newton_euler.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/kinfam_io.hpp"
#include "kdl_parser/kdl_parser.hpp"

// #include "youbot_driver/youbot/EthercatMasterInterface.hpp"
#include "youbot_driver/youbot/EthercatMasterWithThread.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"

#include "ros/ros.h"

enum class CoordinateSystem
{
  BASE,
  EE
};

void
transform(KDL::Jacobian& source_jacobian,
          KDL::JntArray* q,
          CoordinateSystem source_cs,
          CoordinateSystem target_cs,
          KDL::ChainFkSolverPos_recursive* _fksolver,
          int segment_nr = -1)
{
  KDL::Frame target_frame;
  switch (source_cs) {
    case CoordinateSystem::BASE:
      switch (target_cs) {
        case CoordinateSystem::BASE:
          break;
        case CoordinateSystem::EE:
          _fksolver->JntToCart(*q, target_frame, segment_nr);
          for (int i = 0; i < source_jacobian.columns(); i++) {
            source_jacobian.setColumn(
              i, target_frame.M.Inverse() * source_jacobian.getColumn(i));
          }
          break;
      }
      break;
    case CoordinateSystem::EE:
      switch (target_cs) {
        case CoordinateSystem::BASE:
          _fksolver->JntToCart(*q, target_frame, segment_nr);
          for (int i = 0; i < source_jacobian.columns(); i++) {
            source_jacobian.setColumn(
              i, target_frame.M * source_jacobian.getColumn(i));
          }
          break;
        case CoordinateSystem::EE:
          break;
      }
      break;
  }
}

void
transform(KDL::JntArray& source_jnt_array,
          KDL::JntArray* q,
          CoordinateSystem source_cs,
          CoordinateSystem target_cs,
          KDL::ChainFkSolverPos_recursive* _fksolver,
          int segment_nr)
{
  KDL::Frame target_frame;
  KDL::Twist source_beta;
  switch (source_cs) {
    case CoordinateSystem::BASE:
      switch (target_cs)
      {
        case CoordinateSystem::BASE:
          break;
        case CoordinateSystem::EE:
          _fksolver->JntToCart(*q, target_frame, segment_nr);
          // multiply the source_jnt_array with the inverse of the target_frame
          // convert the jnt_array to a vector
          source_beta.vel = KDL::Vector(
            source_jnt_array(0), source_jnt_array(1), source_jnt_array(2));
          source_beta.rot = KDL::Vector(
            source_jnt_array(3), source_jnt_array(4), source_jnt_array(5));     
          // multiply the vector with the inverse of the target_frame
          source_beta = target_frame.M.Inverse() * source_beta;
          // convert the vector back to a jnt_array
          source_jnt_array(0) = source_beta.vel.x();
          source_jnt_array(1) = source_beta.vel.y();
          source_jnt_array(2) = source_beta.vel.z();
          source_jnt_array(3) = source_beta.rot.x();
          source_jnt_array(4) = source_beta.rot.y();
          source_jnt_array(5) = source_beta.rot.z();
          break;
      }
      break;
    case CoordinateSystem::EE:
      switch (target_cs) {
        case CoordinateSystem::BASE:
          _fksolver->JntToCart(*q, target_frame, segment_nr);
          // multiply the source_jnt_array with the target_frame
          // convert the jnt_array to a vector
          source_beta.vel = KDL::Vector(
            source_jnt_array(0), source_jnt_array(1), source_jnt_array(2));
          source_beta.rot = KDL::Vector(
            source_jnt_array(3), source_jnt_array(4), source_jnt_array(5));
          // multiply the vector with the target_frame
          source_beta = target_frame.M * source_beta;
          // convert the vector back to a jnt_array
          source_jnt_array(0) = source_beta.vel.x();
          source_jnt_array(1) = source_beta.vel.y();
          source_jnt_array(2) = source_beta.vel.z();
          source_jnt_array(3) = source_beta.rot.x();
          source_jnt_array(4) = source_beta.rot.y();
          source_jnt_array(5) = source_beta.rot.z();
          break;
        case CoordinateSystem::EE:
          break;
      }
      break;
  }
}

KDL::Twist pidController(const KDL::Twist& current_twist, const KDL::Twist& target_twist, double dt, KDL::Twist& error_sum, KDL::Twist& error_last)
{
  double kp = 5.0;
  double ki = 0.0;
  double kd = 0.05;

  // compute error
  KDL::Twist error_twist = target_twist - current_twist;

  // proportional term
  KDL::Twist p_term = kp * error_twist;

  // integral term
  error_sum += error_twist * dt;
  KDL::Twist i_term = ki * error_sum;

  // derivative term
  KDL::Twist d_term = kd * (error_twist - error_last) / dt;

  // compute control
  KDL::Twist control = p_term + i_term + d_term;

  // update error_last
  error_last = error_twist;

  return control;
}

int main(int argc, char** argv)
{

  std::string robot_urdf = "/home/batsy/bitbots/youbot_model.urdf";

  // set the base and tool links
  std::string base_link = "arm_link_0";
  std::string tool_link = "arm_link_5";

  KDL::Tree robot_tree;
  KDL::Chain robot_chain;

  // load the robot URDF into the KDL tree
  if (!kdl_parser::treeFromFile(robot_urdf, robot_tree)) {
    return -1;
  }

  // create the KDL chain
  if (!robot_tree.getChain(base_link, tool_link, robot_chain)) {
    return -1;
  }

  // set joint inertias
  robot_chain.getSegment(0).setJoint().setJointInertia(0.338489424);
  robot_chain.getSegment(1).setJoint().setJointInertia(0.338489424);
  robot_chain.getSegment(2).setJoint().setJointInertia(0.13571);
  robot_chain.getSegment(3).setJoint().setJointInertia(0.04698212);
  robot_chain.getSegment(4).setJoint().setJointInertia(0.01799637);

  // number of joints
  int n_joints = robot_chain.getNrOfJoints();

  // number of segments
  int n_segments = robot_chain.getNrOfSegments();

  std::cout << "Number of joints: " << n_joints << std::endl;
  std::cout << "Number of segments: " << n_segments << std::endl;

  youbot::EthercatMaster::getInstance(
    "youbot-ethercat.cfg",
    "/home/batsy/bitbots/src/youbot_driver/config/",
    true);

  // initialize youbot arm
  youbot::YouBotManipulator myArm("youbot-manipulator");
  myArm.doJointCommutation();
  std::cout << "Calibrating the arm!" << std::endl;
  myArm.calibrateManipulator();

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

  for (int i = 0; i < n_joints; i++) {
    std::cout << "Joint " << i << " angle: " << joint_angles[i].angle.value()
              << std::endl;
  }

  // initialize fksolver
  KDL::ChainFkSolverPos_recursive fksolver(robot_chain);
  // KDL::ChainFkSolverVel_recursive fksolver_vel(robot_chain);

  KDL::JntArray q(n_joints);
  KDL::JntArray qdot(n_joints);
  KDL::JntArray qdotdot(n_joints);
  std::vector<KDL::Wrench> f_ext(n_segments);
  KDL::JntArray torques(n_joints);

  // initialize PV solver params

  // root acceleration - set gravity vector
  // direction is specified opposite for the solver
  KDL::Twist root_acc(KDL::Vector(0.0, 0.0, 9.81), KDL::Vector(0.0, 0.0, 0.0));

  KDL::JntArray ff_tau(n_joints);

  // initialize the solver weights
  int solver_nc = 6;

  // use vereshchagin solver
  KDL::ChainHdSolver_Vereshchagin vereshchagin_solver(
    robot_chain, root_acc, solver_nc);

  KDL::JntArray q_offsets(n_joints);

  q_offsets(0) = KDL::deg2rad * 169.0;
  q_offsets(1) = KDL::deg2rad * 65.0;
  q_offsets(2) = KDL::deg2rad * -151.0;
  q_offsets(3) = KDL::deg2rad * 102.5;
  q_offsets(4) = KDL::deg2rad * 165.0;

  // pid controller terms
  // KDL::Twist error_sum(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));
  // KDL::Twist error_last(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));

  // system time step
  // double dt = 0.001;

  while (true) {

    myArm.getJointData(joint_angles);

    for (int i = 0; i < n_joints; i++) {
      q(i) = joint_angles[i].angle.value() - q_offsets(i);
      qdot(i) = 0.0;
      qdotdot(i) = 0.0;
    }

    // define unit constraint forces for EE
    KDL::Jacobian alpha_unit_forces_base(3);
    KDL::Jacobian alpha_unit_forces_ee(3);

    // beta - accel energy for EE
    KDL::JntArray beta_energy_base(3);
    KDL::JntArray beta_energy_ee(3);

    // set the unit constraint forces for base
    alpha_unit_forces_base.setColumn(
      0, KDL::Twist(KDL::Vector(1.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0)));
    alpha_unit_forces_base.setColumn(
      1, KDL::Twist(KDL::Vector(0.0, 1.0, 0.0), KDL::Vector(0.0, 0.0, 0.0)));
    alpha_unit_forces_base.setColumn(
      2, KDL::Twist(KDL::Vector(0.0, 0.0, 1.0), KDL::Vector(0.0, 0.0, 0.0)));
    
    // set the unit constraint forces for EE
    alpha_unit_forces_ee.setColumn(
      0, KDL::Twist(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0)));
    alpha_unit_forces_ee.setColumn(
      1, KDL::Twist(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 1.0, 0.0)));
    alpha_unit_forces_ee.setColumn(
      2, KDL::Twist(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 1.0)));

    // set the beta energy for base
    beta_energy_base(0) = 0.0;
    beta_energy_base(1) = 0.0;
    beta_energy_base(2) = 9.81;

    // set the beta energy for EE
    beta_energy_ee(3) = 0.0;
    beta_energy_ee(4) = 0.0;
    beta_energy_ee(5) = 0.0;

    // compute the control signal
    // KDL::Twist target_twist(KDL::Vector(0.0, 0.0, 0.0),
    //                         KDL::Vector(0.0, 0.0, 0.0));

    // // current joint velocities
    // KDL::JntArray qdot_current(n_joints);
    // std::vector<youbot::JointSensedVelocity> joint_velocities;
    // myArm.getJointData(joint_velocities);
    // for (int i = 0; i < n_joints; i++) {
    //   qdot_current(i) = joint_velocities[i].angularVelocity.value();
    // }

    // // compute end-effector twist
    // KDL::Twist ee_twist;
    // KDL::FrameVel ee_frame_vel;
    // KDL::JntArrayVel qdot_current_vel;
    // qdot_current_vel.q = q;
    // qdot_current_vel.qdot = qdot_current;

    // fksolver_vel.JntToCart(qdot_current_vel, ee_frame_vel);

    // ee_twist = ee_frame_vel.GetTwist();

    // // compute control signal
    // KDL::Twist control_signal = pidController(
    //   ee_twist, target_twist, dt, error_sum, error_last);

    // // add control signal to beta
    // beta_energy(0) += control_signal.vel.x();
    // beta_energy(1) += control_signal.vel.y();
    // beta_energy(2) += control_signal.vel.z();
    // beta_energy(3) += control_signal.rot.x();
    // beta_energy(4) += control_signal.rot.y();
    // beta_energy(5) += control_signal.rot.z();


    // transform alpha_unit_forces to BASE
    transform(alpha_unit_forces_ee,
              &q,
              CoordinateSystem::EE,
              CoordinateSystem::BASE,
              &fksolver,
              -1);

    // combine alpha_unit_forces
    KDL::Jacobian alpha_unit_forces(6);
    alpha_unit_forces.setColumn(0, alpha_unit_forces_base.getColumn(0));
    alpha_unit_forces.setColumn(1, alpha_unit_forces_base.getColumn(1));
    alpha_unit_forces.setColumn(2, alpha_unit_forces_base.getColumn(2));
    alpha_unit_forces.setColumn(3, alpha_unit_forces_ee.getColumn(0));
    alpha_unit_forces.setColumn(4, alpha_unit_forces_ee.getColumn(1));
    alpha_unit_forces.setColumn(5, alpha_unit_forces_ee.getColumn(2));

    // combine beta_energy
    KDL::JntArray beta_energy(6);
    beta_energy(0) = beta_energy_base(0);
    beta_energy(1) = beta_energy_base(1);
    beta_energy(2) = beta_energy_base(2);
    beta_energy(3) = beta_energy_ee(3);
    beta_energy(4) = beta_energy_ee(4);
    beta_energy(5) = beta_energy_ee(5);

    vereshchagin_solver.CartToJnt(
      q, qdot, qdotdot, alpha_unit_forces, beta_energy, f_ext, ff_tau, torques);

    // torque limit
    double torque_limit = 10.0;
    for (int i = 0; i < n_joints; i++) {
      if (torques(i) > torque_limit) {
        torques(i) = torque_limit;
      } else if (torques(i) < -torque_limit) {
        torques(i) = -torque_limit;
      }
    }

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
    desiredJointTorque.torque = 0.0 * si::newton_meters;
    commandTorques.push_back(desiredJointTorque);

    // std::vector<KDL::Twist> link_accs(n_segments);
    // vereshchagin_solver.getTransformedLinkAcceleration(link_accs);

    // std::cout << "joint q: " << q << std::endl;
    // std::cout << "joint qdot: " << qdot << std::endl;
    // std::cout << "joint qdotdot: " << qdotdot << std::endl;
    // auto ee_twist = link_accs[4];
    // std::cout << "betas: " << beta_energy << std::endl;
    std::cout << "joint torques: " << torques << std::endl;

    myArm.setJointData(commandTorques);
    // usleep(300);
  }

  return 0;
}