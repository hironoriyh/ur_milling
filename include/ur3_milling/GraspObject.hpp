/*
 * GraspObject.hpp
 *
 *  Created on: 4.08.2017
 *      Author: Hironori Yoshida
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group.h>

// Eigen
#include <Eigen/Core>

// c++
#include <memory>
#include <vector>

#include <grasp_and_place/GripperInterface.hpp>
#include <grasp_and_place/Object.hpp>
#include "grasp_and_place_msgs/HandCommand.h"

#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>

namespace grasp_and_place {

// typedefs.
typedef Eigen::Vector3d Vector3D;

class GraspObject
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle ros node handle.
   */
  GraspObject(ros::NodeHandle& nh, std::shared_ptr<GripperInterface> gripper_interface,
              std::shared_ptr<moveit::planning_interface::MoveGroup> move_group,
              geometry_msgs::Pose& object_pose, geometry_msgs::Pose& grasping_pose,
              std::vector<double> finger_pos);

  GraspObject(ros::NodeHandle& nh, std::shared_ptr<GripperInterface> gripper_interface,
              std::shared_ptr<moveit::planning_interface::MoveGroup> move_group,
              geometry_msgs::Pose& object_pose,
              GraspingConfigruation grasping_config);
  /*!
   * Destructor.
   */
  virtual ~GraspObject();

  void SetModel(const Object& object); // TODO: Define what is need for the model.

  bool ExecuteGrasping();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool ReadParameters();

  bool MoveToPose(const geometry_msgs::PoseStamped& target_pose, double eef_step);

  bool PoseRefinement(geometry_msgs::PoseStamped& pose);

  visualization_msgs::Marker VisualizeMarker(const int marker_type,
                                                             const geometry_msgs::Pose pose,
                                                             const int id, const float r,
                                                             const float g, const float b,
                                                             const float a);

  ros::NodeHandle nh_;

  ros::Publisher approaching_pose_publisher_;
  ros::Publisher mesh_publisher_;

  //! Gripper interface.
  std::shared_ptr<GripperInterface> gripper_interface_;
  double current_threshold_;

  //! Moveit Planning interface.
  std::shared_ptr<moveit::planning_interface::MoveGroup> move_group_;

  //! Object pose.
  geometry_msgs::Pose object_pose_;

  //! Grasping pose (TCP) in world frame.
  geometry_msgs::Pose grasping_pose_;

  //! Grasping finger positions.
  std::vector<double> finger_pos_;

  visualization_msgs::MarkerArray refined_mesh_;

  //! Object.
  Object object_;

  //! Approaching distance.
  double approaching_distance_;

  tf::TransformListener tf_listener_;

  int grasping_index_;

};

} /* grasp_and_place */
