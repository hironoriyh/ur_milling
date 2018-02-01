/*
 * Surface.hpp
 *
 *  Created on: 23.11.2017
 *      Author: Hironori Yoshida
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// c++
#include <string>
#include <vector>

namespace grasp_and_place {

class Surface
{
 public:
  /*!
   * Constructor.
   * @param name object name.
   */
	Surface(const std::string& name);

  /*!
   * Constructor.
   */
	Surface();

  /*!
   * Destructor.
   */
  virtual ~Surface();

  void setName(const std::string& name);
  std::string getName();

  void setPose(const geometry_msgs::Pose pose);
  geometry_msgs::Pose getPose();

  void setPlacingPose(const geometry_msgs::Pose pose);
  geometry_msgs::Pose getPlacingPose();

  void setRefinedGraspingPose(const geometry_msgs::Pose pose);
  geometry_msgs::Pose getRefinedGraspingPose();

  void addGraspingPose(const geometry_msgs::Pose pose);
  void clearGraspingPose();
  std::vector<geometry_msgs::Pose> getGraspingPoses();

  void setTargetGraspingPose(const geometry_msgs::Pose pose);
  geometry_msgs::Pose getTargetGraspingPose();

  bool isPoseSet();

 private:

  std::string name_;
  geometry_msgs::Pose pose_;
  bool pose_set_;
  geometry_msgs::Pose placing_pose_;

  // Grasping pose after refinement.
  geometry_msgs::Pose refined_grasping_pose_;

  // Target grasping pose. (Grasping pose used for placing)
  geometry_msgs::Pose target_grasping_pose_;

  // Possible grasping poses (in the stone frame).
  std::vector<geometry_msgs::Pose> grasping_poses_;


};

} /* grasp_and_place */
