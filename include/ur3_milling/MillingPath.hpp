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

// grasp_and_place interface
#include <grasp_and_place/Object.hpp>
#include <grasp_and_place/GripperInterface.hpp>

namespace grasp_and_place {

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

  bool LoadGraspingPoses();

  bool LoadCollisionEnvironment();

  bool ExecuteMillingPath(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  bool ExecuteLocalization(std::vector<Object> models_to_localize = std::vector<Object>());

  bool DetectObject(std::vector<Object>& models_to_detect);

  bool DetectObjectRotate(const int object_index);

  bool LocalizeObjects();

  bool MoveToPose(const geometry_msgs::PoseStamped& target_pose);

  bool LocateRedPoint();

  bool PickAndPlaceObject(const int object_index);

  bool EvaluatePlacedPose(const int object_index);

  bool PoseRefinement(const int stack_model_index);

  /*!
   * Add collision object to the planning scene. If collision object already exists
   * the position is updated.
   * @param object_name id of the object.
   * @param object_index index in the vector of the desired configuration.
   * @return true if successful
   */
  bool AddCollisionObject(const std::string object_name, const int object_index);

  bool Regrasp(const int desired_stacking_index);

  /*!
   * Checks if the desired grasping pose is valid, i.e. there are no collisions.
   * @param[in] pose the target TCP pose.
   * @return true if grasping pose is feasible, false otherwise.
   */
  bool CheckCollision(const geometry_msgs::Pose pose);

  /*!
   * Computes the target TCP pose of the robot dependent on the desired object pose.
   * @param stack_model_index index of the desired object to stack.
   * @return the target TCP pose to reach the desired object pose.
   */
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
	ros::Publisher refined_pose_publisher_;
	ros::Publisher move_pose_publisher_;
	ros::Publisher localised_objects_visualizer_;
	ros::Publisher target_config_publisher_;

	visualization_msgs::MarkerArray mesh_;
	visualization_msgs::MarkerArray refined_mesh_;

	//! Service clients.
  ros::ServiceClient check_validity_;

	//! Gripper interface.
  std::shared_ptr<GripperInterface> gripper_interface_;

  //! Grasping poses.
  std::map<std::string, std::vector<GraspingConfigruation>> grasping_poses_;

  //! Service server.
  ros::ServiceServer execute_stacking_;
  ros::ServiceServer open_gripper_;


  //! Desired stacking configuration.
  std::vector<Object> desired_stacking_configuration_;
  std::vector<Object> stacked_objects_;

  //! Model locations from object localisation.
  std::vector<geometry_msgs::PoseStamped> model_locations_;

  //! MoveIt! move group interface.
  std::shared_ptr<moveit::planning_interface::MoveGroup> move_group_;
  double velocity_scaling_;

  //! MoveIt! planning scene interface.
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;

  // Approaching distance for detection.
  double distance_to_objects_;

  //! Colored dot location.
  geometry_msgs::Point center_of_colored_dot_;

  //! Allowed deviation to target pose.
  double position_diff_;
  double rotation_diff_;
  Vector3D min_localization_;
  Vector3D max_localization_;


};

} // grasp_and_place
