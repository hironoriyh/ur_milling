/*
 * GripperInterface.cpp
 *
 *  Created on: 26.07.2017
 *      Author: Hironori Yoshida
 */

#include "grasp_and_place/GripperInterface.hpp"
#include "grasp_and_place_msgs/HandCommand.h"
#include <inttypes.h>

namespace grasp_and_place {

GripperInterface::GripperInterface(ros::NodeHandle& nh)
: nodeHandle_(nh)
{

//  handCommandPublisher_ = nodeHandle_.advertise <std_msgs::String> ("/b3m_command", 1, true);
  gripperStateSubscriber_ = nodeHandle_.subscribe < grasp_and_place_msgs::HandStates> ("/b3m_state", 1, &GripperInterface::GripperCallback, this);

}

GripperInterface::~GripperInterface()
{
  ShutdownGripper();
}

void GripperInterface::ActivateGripper()
{
    ROS_INFO("GripperInterface: Activate Gripper");
  return;
}

void GripperInterface::ShutdownGripper()
{
  ROS_INFO("GripperInterface: Shutdown Gripper");
//  OpenGripper();
  return;
}

void GripperInterface::GripperCallback(const  grasp_and_place_msgs::HandStates::ConstPtr& msg)
{
  hand_state_ = *msg;
//  ROS_INFO_STREAM(hand_state_.collision);
//  uint8_t collision_check = 1;
//  printf("msg: %" PRIu8 "\n", hand_state_.collision);
//  printf("test: %" PRIu8 "\n", collision_check);

}

bool GripperInterface::ClientHandCommand(std::string command)
{
  ros::ServiceClient client = nodeHandle_.serviceClient<grasp_and_place_msgs::HandCommand>("/hand_command");
  grasp_and_place_msgs::HandCommand srv;
  srv.request.command = command;
  if (!client.waitForExistence(ros::Duration(2.0))) {
    return false;
  }
  if (client.call(srv)) {

  } else {
    ROS_WARN("GripperInterface:ClientHandCommand Could not call client.");
    return false;
  }

  return srv.response.success;
}

bool GripperInterface::ClientHandCommand(std::string command, std::vector<double> positions)
{
  ros::ServiceClient client = nodeHandle_.serviceClient<grasp_and_place_msgs::HandCommand>("/hand_command");
  grasp_and_place_msgs::HandCommand srv;
  srv.request.command = command;
//  std_msgs::
  srv.request.finger_positions = positions;
  ROS_INFO_STREAM("GripperInterface::ClientHandCommand: " <<  positions.size());
  if (!client.waitForExistence(ros::Duration(2.0))) {
    return false;
  }
  if (client.call(srv)) {

  } else {
    ROS_WARN("GripperInterface:ClientHandCommand Could not call client.");
    return false;
  }

  return srv.response.success;
}

grasp_and_place_msgs::HandStates GripperInterface::GetHandState()
{
  return hand_state_;
}

//void GripperInterface::SetBasicMode()
//{
//  ROS_INFO("GripperInterface: Basic Mode");
//  gripper_command_.rMOD = 0x0;
//  gripper_command_.rGTO = 0;
//  gripper_command_.rSPA = 255; // Max speed.
//  PublishCommand();
//  ros::Duration(0.5).sleep();
//
//  // Block until gripper mode is changed.
//  while (gripper_state_.gIMC != 3) {
//    ros::Duration(0.1).sleep();
//    ros::spinOnce();
//  }
//  return;
//}
//
//void GripperInterface::SetPinchMode()
//{
//  ROS_INFO("GripperInterface: Pinch Mode");
//  gripper_command_.rMOD = 0x1;
//  gripper_command_.rGTO = 0;
//  gripper_command_.rSPA = 255;
//  PublishCommand();
//  ros::Duration(0.5).sleep();
//
//  // Block until gripper mode is changed.
//  while (gripper_state_.gIMC != 3) {
//    ros::Duration(0.1).sleep();
//    ros::spinOnce();
//  }
//  return;
//}
//
//void GripperInterface::SetWideMode()
//{
//  ROS_INFO("GripperInterface: Wide Mode");
//  gripper_command_.rMOD = 0x2;
//  gripper_command_.rGTO = 0;
//  gripper_command_.rSPA = 255;
//  PublishCommand();
//  ros::Duration(0.5).sleep();
//
//  // Block until gripper mode is changed.
//  while (gripper_state_.gIMC != 3) {
//    ros::Duration(0.1).sleep();
//    ros::spinOnce();
//  }
//  return;
//}
//
//void GripperInterface::SetScissorMode()
//{
//  ROS_INFO("GripperInterface: Scissor Mode");
//  gripper_command_.rMOD = 0x3;
//  gripper_command_.rGTO = 0;
//  gripper_command_.rSPA = 255;
//  PublishCommand();
//  ros::Duration(0.5).sleep();
//
//  // Block until gripper mode is changed.
//  while (gripper_state_.gIMC != 3) {
//    ros::Duration(0.1).sleep();
//    ros::spinOnce();
//  }
//  return;
//}
//
//void GripperInterface::MoveFingerA(const double position, const double speed, const double force)
//{
//  ROS_INFO("GripperInterface: Move finger A.");
//  gripper_command_.rGTO = 1;
//  gripper_command_.rICF = 1;
//  gripper_command_.rFRA = saturate((unsigned int)(255 * force)); // Desired final grasping force.
//  gripper_command_.rSPA = saturate((unsigned int)(255 * speed)); // Gripper closing speed.
//  gripper_command_.rPRA = saturate((unsigned int)(255 * position)); // Gripper position.
//  gripper_command_.rPRB = gripper_state_.gPOB;
//  gripper_command_.rPRC = gripper_state_.gPOC;
//  PublishCommand();
//  ros::Duration(0.5).sleep();
//  while (IsInMotion()) {
//    ros::Duration(0.05).sleep();
//    ros::spinOnce();
//  }
//  return;
//}
//
//void GripperInterface::MoveFingerB(const double position, const double speed, const double force)
//{
//  ROS_INFO("GripperInterface: Move finger B.");
//  gripper_command_.rGTO = 1;
//  gripper_command_.rICF = 1;
//  gripper_command_.rFRB = saturate((unsigned int)(255 * force)); // Desired final grasping force.
//  gripper_command_.rSPB = saturate((unsigned int)(255 * speed)); // Gripper closing speed.
//  gripper_command_.rPRB = saturate((unsigned int)(255 * position)); // Gripper position.
//  gripper_command_.rPRA = gripper_state_.gPOA;
//  gripper_command_.rPRC = gripper_state_.gPOC;
//  PublishCommand();
//  ros::Duration(0.5).sleep();
//  while (IsInMotion()) {
//    ros::Duration(0.05).sleep();
//    ros::spinOnce();
//  }
//  return;
//}
//
//void GripperInterface::MoveFingerC(const double position, const double speed, const double force)
//{
//  ROS_INFO("GripperInterface: Move finger C.");
//  gripper_command_.rGTO = 1;
//  gripper_command_.rICF = 1;
//  gripper_command_.rFRC = saturate((unsigned int)(255 * force)); // Desired final grasping force.
//  gripper_command_.rSPC = saturate((unsigned int)(255 * speed)); // Gripper closing speed.
//  gripper_command_.rPRC = saturate((unsigned int)(255 * position)); // Gripper position.
//  gripper_command_.rPRA = gripper_state_.gPOA;
//  gripper_command_.rPRB = gripper_state_.gPOB;
//  PublishCommand();
//  ros::Duration(0.5).sleep();
//  while (IsInMotion()) {
//    ros::Duration(0.05).sleep();
//    ros::spinOnce();
//  }
//  return;
//}
//
//void GripperInterface::CloseAOpenBC(const double positionA, const double speedA, const double forceA,
//                                    const double positionBC, const double speedBC, const double forceBC)
//{
//  ROS_INFO("GripperInterface: Close A and open B and C.");
//  gripper_command_.rGTO = 1;
//  gripper_command_.rICF = 1;
//  gripper_command_.rFRA = saturate((unsigned int)(255 * forceA)); // Desired final grasping force.
//  gripper_command_.rSPA = saturate((unsigned int)(255 * speedA)); // Gripper closing speed.
//  gripper_command_.rPRA = saturate((unsigned int)(255 * positionA)); // Gripper position.
//  gripper_command_.rFRB = saturate((unsigned int)(255 * forceBC));
//  gripper_command_.rSPB = saturate((unsigned int)(255 * speedBC));
//  gripper_command_.rPRB = saturate((unsigned int)(255 * positionBC));
//  gripper_command_.rFRC = saturate((unsigned int)(255 * forceBC));
//  gripper_command_.rSPC = saturate((unsigned int)(255 * speedBC));
//  gripper_command_.rPRC = saturate((unsigned int)(255 * positionBC));
//  PublishCommand();
//  ros::Duration(0.5).sleep();
//  while (IsInMotion()) {
//    ros::Duration(0.05).sleep();
//    ros::spinOnce();
//  }
//  return;
//}
//
//bool GripperInterface::IsInMotion()
//{
//  return (gripper_state_.gGTO == 1 && gripper_state_.gSTA == 0);
//}
//
//void GripperInterface::PublishCommand()
//{
//  gripperCommandPublisher_.publish(gripper_command_);
//}

}
