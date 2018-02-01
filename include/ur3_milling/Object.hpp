/*
 * Object.hpp
 *
 *  Created on: 23.06.2016
 *      Author: Martin Wermelinger
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// c++
#include <string>
#include <vector>

namespace grasp_and_place {

enum GraspingMode {
  BASIC,
  PINCH,
  WIDE,
  SCISSOR
};

struct GraspingConfigruation {
  //! Grasping pose in object frame.
  geometry_msgs::Pose pose;
  int grasp_pose_index;
  std::vector<double> finger_positions;
  std::string object_index;
};

class Object
{
 public:
  /*!
   * Constructor.
   * @param name object name.
   */
  Object(const std::string& name);

  /*!
   * Constructor.
   */
  Object();

  /*!
   * Destructor.
   */
  virtual ~Object();

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
