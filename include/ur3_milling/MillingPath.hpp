/*
 * MillingPath.hpp
 *
 *  Created on: Jul 26, 2017
 *      Author: Hironori Yoshida
 */

# pragma once

// ros
#include <ros/ros.h>
#include <ros/package.h>


#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>

// Eigen
#include <Eigen/Core>

// std
#include <string>
#include <vector>
#include <map>
#include <algorithm>

//ros msgs
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>


namespace ur3_milling {

// typedefs.
typedef Eigen::Vector3d Vector3D;

class MillingPath
{

public:
  MillingPath(ros::NodeHandle& nh);
	virtual ~MillingPath();

private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool ReadParameters();

  /*!
   * Load stacking configuration from file.
   * @return true if successful.
   */
  bool LoadStackingConfiguration();

  // bool DetectObject(std::vector<Object>& models_to_detect);

  // bool DetectObjectRotate(const int object_index);
  //
  // bool LocalizeObjects();

  bool MoveToPose(const geometry_msgs::PoseStamped& target_pose);

  bool MoveToPoses(std::vector<geometry_msgs::PoseStamped> target_poses);

  geometry_msgs::Pose GetTargetTCPPose(const int stack_model_index);

  visualization_msgs::Marker VisualizeMarker(const int marker_type, const geometry_msgs::Pose pose, const int id, const float r, const float g, const float b, const float a, const Vector3D scale = Vector3D::Ones());

	//! ROS Node handle.
	ros::NodeHandle nh_;

	//! Transform listener.
	tf::TransformListener tf_listener_;
	std::string camera_frame_;
	std::string world_frame_;

	//! Publisher.
	ros::Publisher mesh_publisher_;
	ros::Publisher move_pose_publisher_;

  //! Model locations from object localisation.
  std::vector<geometry_msgs::PoseStamped> model_locations_;

  //! MoveIt! move group interface.
  std::shared_ptr<moveit::planning_interface::MoveGroup> move_group_;
  double velocity_scaling_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;

  //! Allowed deviation to target pose.
  double position_diff_;
  double rotation_diff_;
  Vector3D min_localization_;
  Vector3D max_localization_;


};

} // ur3_milling
