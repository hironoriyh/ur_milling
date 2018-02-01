/*
 * GripperInterface.hpp
 *
 *  Created on: 2017.07.24
 *      Author: Hironori Yoshida
 */

#pragma once

// ROS
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <grasp_and_place_msgs/HandStates.h>

namespace grasp_and_place {

class GripperInterface
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle ros node handle.
   */
  GripperInterface(ros::NodeHandle& nh);

  /*!
   * Destructor.
   */
  virtual ~GripperInterface();

  void ActivateGripper();

  void ShutdownGripper();

  /*!
   * Opens all fingers of the gripper.
   * @param speed opening speed: 0 = min opening speed (22mm/s), 1 = max opening speed (110mm/s).
   * @param position target position: 0 = fully open, 1 = fully closed.
   */

  bool ClientHandCommand(std::string command);

  bool ClientHandCommand(std::string command, std::vector<double> positions);

  grasp_and_place_msgs::HandStates GetHandState();

//  void SetBasicMode();
//
//  void SetPinchMode();
//
//  void SetWideMode();
//
//  void SetScissorMode();
//
//  /*!
//   * Move finger A of the gripper.
//   * @param position target finger position: 0 = fully open, 1 = fully closed.
//   * @param speed closing speed: 0 = min closing speed (22mm/s), 1 = max closing speed (110mm/s).
//   * @param force grasping force: 0 = min grasping force (15N), 1 = max grasping force (60N).
//   */
//  void MoveFingerA(const double position = 1, const double speed = 0.5, const double force = 0.5);
//
//  void MoveFingerB(const double position = 1, const double speed = 0.5, const double force = 0.5);
//
//  void MoveFingerC(const double position = 1, const double speed = 0.5, const double force = 0.5);
//
//  void CloseAOpenBC(const double positionA = 1, const double speedA = 0.5, const double forceA = 0.5,
//                    const double positionBC = 1, const double speedBC = 0.5, const double forceBC = 0.5);

 private:
//  void PublishHandCommand(std::string command);



  void PublishCommand();

  void GripperCallback(const  grasp_and_place_msgs::HandStates::ConstPtr& msg);

  bool IsInMotion();

  unsigned int saturate(unsigned int value)
  {
    return std::min(std::max(value, min_), max_);
  }

  ros::NodeHandle nodeHandle_;

  ros::Subscriber gripperStateSubscriber_;

  ros::Publisher gripperCommandPublisher_;

  ros::Publisher handCommandPublisher_;

  grasp_and_place_msgs::HandStates hand_state_;


  //! Allowed range for gripper commands.
  unsigned int min_ = 0;
  unsigned int max_ = 255;
};

} /* grasp_and_place */
