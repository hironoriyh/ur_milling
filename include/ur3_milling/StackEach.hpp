/*
 * StackEach.hpp
 *
 *  Created on: Apr 21, 2016
 *      Author: Martin Wermelinger
 */

# pragma once

// ros
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>

// Eigen
#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <geometry_msgs/Point.h>
#include <grasp_and_place/Object.hpp>

#include <grasp_and_place/GripperInterface.hpp>
#include <grasp_and_place_msgs/GetObjectPose.h>
#include <grasp_and_place_msgs/SetObjectPose.h>
#include <grasp_and_place_msgs/SetObject.h>
#include <grasp_and_place_msgs/PoseAnalysis.h>
#include <grasp_and_place_msgs/ComputeNextPose.h>
#include <grasp_and_place_msgs/PlacingResult.h>

#include <grasp_and_place/GripperInterface.hpp>


namespace grasp_and_place {

// typedefs.
typedef Eigen::Vector3d Vector3D;
typedef Eigen::Vector2d Vector2D;

typedef grasp_and_place_msgs::GetObjectPose GetPose;
typedef grasp_and_place_msgs::SetObjectPose SetPose;
typedef grasp_and_place_msgs::SetObject SetObject;
typedef grasp_and_place_msgs::PoseAnalysis PoseAnalysis;
typedef grasp_and_place_msgs::ComputeNextPose ComputeNextPose;
typedef grasp_and_place_msgs::PlacingResult PlacingResult;


class StackEach
{

public:
  StackEach(ros::NodeHandle& nh);
	virtual ~StackEach();

private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool ReadParameters();

  /*!
   * Read parameters from parameter server.
   */
  bool LoadAvailableObjects();

  bool LoadGraspingPoses();

  bool ClearLocalizedObject(SetObject::Request& req, SetObject::Response& res);

  bool ExecuteStackObjectCallback(SetObject::Request& req, SetObject::Response& res);

  bool ExecuteOptimalStackCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  bool ExecuteLocalizationCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  bool InfiniteStackingCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  bool ExecuteLocalization(std::vector<Object> models_to_localize = std::vector<Object>());

  bool ExecuteSingleStack(const std::string object_name);

  bool ExecuteOptimalStack();

  bool PickObject(const int object_index);

  bool StackObject(const int object_index);

  bool PickAndPlaceObject(const int object_index);

  bool EvaluatePlacedPose(const int object_index);

  bool DetectObject(std::vector<Object>& models_to_detect, bool set_model_pose = true);

  bool DetectObjectRotate(const int object_index);

  bool LocalizeObjects();

  bool ComputeDesiredPose(const int object_index, double& cost);

  bool ComputeOptimalNextPose(int& object_index, double& cost, const int max_planning_horizon = 1);

  bool IsObjectStacked(const std::string object_name);

  /*!
   * Computes the target TCP pose of the robot dependent on the desired object pose.
   * @param stack_model_index index of the desired object to stack.
   * @return the target TCP pose to reach the desired object pose.
   */
  geometry_msgs::Pose GetTargetTCPPose(const int stack_model_index);

  geometry_msgs::Point GetStackingPoint();

  /*!
   * Moves the robot to the defined end effector (TCP) pose.
   * @param target_pose target pose of the end effector (TCP).
   * @return true if successful.
   */
  bool MoveToPose(const geometry_msgs::PoseStamped& target_pose);

  /*!
   * Checks if the desired grasping pose is valid, i.e. there are no collisions.
   * @param[in] pose the target TCP pose.
   * @return true if grasping pose is feasible, false otherwise.
   */
  bool CheckCollision(const geometry_msgs::Pose pose);

  bool PoseRefinement(const int scene_model_index);


  /*!
   * Add collision object to the planning scene. If collision object already exists
   * the position is updated.
   * @param object_name id of the object.
   * @param object_index index in the vector of detected objects.
   * @return true if successful
   */
  bool AddCollisionObject(const std::string object_name, const int object_index);

  visualization_msgs::Marker VisualizeMarker(int marker_type, geometry_msgs::Pose pose, int id, float r, float g, float b, float a, Vector3D scale = Vector3D::Ones());

  bool SaveDetectedPoses();

  //! ROS Node handle.
	ros::NodeHandle nh_;

  //! Transform listener.
  tf::TransformListener tf_listener_;
  std::string camera_frame_;
  std::string world_frame_;

  //! Publisher
	ros::Publisher mesh_publisher_;
	ros::Publisher refined_pose_publisher_;
	ros::Publisher localised_objects_visualizer_;
  ros::Publisher target_pose_publisher_;
  ros::Publisher target_config_publisher_;
  ros::Publisher move_pose_publisher_;

	visualization_msgs::MarkerArray meshes_;
	visualization_msgs::MarkerArray refined_mesh_;

  //! Service client.
  ros::ServiceClient find_first_pose_;
  ros::ServiceClient find_further_stack_;
  ros::ServiceClient set_pose_;
  ros::ServiceClient set_model_state_;
  ros::ServiceClient detect_object_;
  ros::ServiceClient localize_objects_;
  ros::ServiceClient refine_object_pose_;
  ros::ServiceClient check_validity_;
  ros::ServiceClient check_touch_table_;
  ros::ServiceClient set_stacking_position_;
  ros::ServiceClient remove_from_stack_;
  ros::ServiceClient set_detected_config_;
  ros::ServiceClient compute_next_pose_;
  ros::ServiceClient set_placing_result_;
  ros::ServiceClient save_config_;
	//! Gripper interface.
  std::shared_ptr<GripperInterface> gripper_interface_;

  //! Grasping poses.
  std::map<std::string, std::vector<GraspingConfigruation>> grasping_poses_;

  //! Service server.
  ros::ServiceServer stack_object_server_;
  ros::ServiceServer stack_optimal_object_server_;
  ros::ServiceServer localization_server_;
  ros::ServiceServer clear_localized_object_server_;
  ros::ServiceServer infinite_stacking_server_;

  //! Desired stacking configuration.
  std::vector<Object> available_objects_;
  std::vector<Object> detected_objects_;
  std::vector<Object> stacked_objects_;
  std::vector<Object> old_stack_;

  //! Model locations from object localisation.
  std::vector<geometry_msgs::PoseStamped> model_locations_;

  //! MoveIt! move group interface.
  std::shared_ptr<moveit::planning_interface::MoveGroup> move_group_;

  //! MoveIt! planning scene interface.
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;

  // Approaching distance for detection.
  double distance_to_objects_;

  //! Colored dot location.
  geometry_msgs::Point center_of_colored_dot_;

  //! Allowed deviation to target pose.
  double position_diff_;
  double rotation_diff_;

  // localization boundary
  Vector3D min_localization_;
  Vector3D max_localization_;

  //! Localize before stacking
  bool localize_before_stacking_ = false;

  //! Stacking point.
  geometry_msgs::Point stacking_point_;
  double stacking_point_threshold_ = 0.05;
};

} // grasp_and_place
