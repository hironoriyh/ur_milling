/*
 * PlaceObject.hpp
 *
 *  Created on: 21.04.2016
 *      Author: Martin Wermelinger
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>

// moveit
#include <moveit/move_group_interface/move_group.h>
//#include <moveit/move_group_interface/move_group_interface.h>


// Eigen
#include <Eigen/Core>

// c++
#include <memory>

#include <grasp_and_place/GripperInterface.hpp>
#include <grasp_and_place/Object.hpp>


#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>

namespace grasp_and_place {

// typedefs.
typedef Eigen::Vector3d Vector3D;

class PlaceObject
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle ros node handle.
   */
  PlaceObject(ros::NodeHandle& nh,
              std::shared_ptr<GripperInterface> gripper_interface,
			  moveit::planning_interface::MoveGroupPtr  move_group,
              geometry_msgs::Pose target_pose);
  /*!
   * Destructor.
   */
  virtual ~PlaceObject();

  void SetModel(const Object& object); // TODO: Define what is need for the model.

  bool ExecutePlacing();


  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool ReadParameters();

  bool MoveToPose(const geometry_msgs::PoseStamped& target_pose, double eef_step, bool check_collision);

  bool MoveToPose(std::vector<geometry_msgs::PoseStamped> target_poses);

  /*
   * Moves the arm incrementally in z direction until the max distance is reached or the
   * force exceeds the max allowed force. !!! Attention !!! This method does not check for collision.
   */
  bool IncrementalMoveTCP(const double threshold = 0, const double eef_step = 0, const double max_distance = 0, const double timeout = 0);

  bool PoseRefinement();

  // geometry_msgs::Pose GetTargetTCPPose(const int stack_model_index);

  visualization_msgs::Marker VisualizeMarker(const int marker_type,
                                                             const geometry_msgs::Pose pose,
                                                             const int id, const float r,
                                                             const float g, const float b,
                                                             const float a);

  ros::Publisher target_pose_publisher_;
  ros::Publisher refined_pose_publisher_;

   private:

  ros::NodeHandle nh_;

  //! Gripper interface.
  std::shared_ptr<GripperInterface> gripper_interface_;

  //! Moveit Planning interface.
  moveit::planning_interface::MoveGroupPtr move_group_;
//  moveit::planning_interface::MoveGroupInterfacePtr move_group_;

  //! Object pose.
  geometry_msgs::Pose target_pose_;

  //! Object.
  Object object_;
//  moveit_msgs::CollisionObject object_;

  //! Approaching distance.
  double approaching_distance_;

  double force_threshold_ = 1;
  double eef_step_ = 0.001;
  double incremental_movement_distance_ = -0.1;
  double approaching_distance_release_ = 0.1;
  double timeout_ = 10.0;
  double velocity_scaling_ = 0.1;

  bool joint_4_limited = true;

  tf::TransformListener tf_listener_;
  visualization_msgs::MarkerArray refined_mesh_;


};

} /* grasp_and_place */
