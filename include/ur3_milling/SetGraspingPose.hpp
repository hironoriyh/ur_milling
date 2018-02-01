/*
 * SetGraspingPose.hpp
 *
 *  Created on: Jul 14, 2016
 *      Author: Martin Wermelinger
 */

# pragma once

// ros
#include <ros/ros.h>
#include <std_srvs/Empty.h>
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
#include <grasp_and_place/GripperInterface.hpp>
#include <grasp_and_place/Object.hpp>

#include <grasp_and_place_msgs/SetObject.h>
#include <grasp_and_place_msgs/SetMode.h>

namespace grasp_and_place {

// typedefs.
typedef Eigen::Vector3d Vector3D;

typedef grasp_and_place_msgs::SetObject SetObject;
typedef grasp_and_place_msgs::SetMode SetMode;

class SetGraspingPose
{

public:
  SetGraspingPose(ros::NodeHandle& nh);
	virtual ~SetGraspingPose();

private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool ReadParameters();

  bool LoadAvailableObjects();

  bool ComputeGraspingPoseCallback(SetObject::Request& req, SetObject::Response& res);

  bool GoToHomePoseCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  bool TestPlacingCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  bool OpenGripper(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  bool CloseGripper(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  bool SetMode(SetMode::Request& req, SetMode::Response& res);

  bool MoveToPose(geometry_msgs::PoseStamped& target_pose);

  bool PoseRefinement(std::string object_name, geometry_msgs::Pose& pose);

  visualization_msgs::Marker VisualizeMarker(int marker_type, geometry_msgs::Pose pose, int id, float r, float g, float b, float a, Vector3D scale = Vector3D::Ones());

  bool PrintDetectedPose(std::string object_name, geometry_msgs::Pose& pose);

  //! ROS Node handle.
	ros::NodeHandle nh_;

  //! Transform listener.
  tf::TransformListener tf_listener_;
  std::string camera_frame_;
  std::string world_frame_;

  //! Publisher
	ros::Publisher mesh_publisher_;
	visualization_msgs::MarkerArray mesh_;


	//! Gripper interface.
  std::shared_ptr<GripperInterface> gripper_interface_;

  //! Service server.
  ros::ServiceServer set_mode_;
  ros::ServiceServer compute_grasping_pose_;
  ros::ServiceServer go_to_home_pose_;
  ros::ServiceServer test_placing_;
  ros::ServiceServer open_gripper_;
  ros::ServiceServer close_gripper_;

  //! Desired stacking configuration.
  std::vector<Object> available_objects_;
  std::vector<Object> detected_objects_;

  //! MoveIt! move group interface.
  std::shared_ptr<moveit::planning_interface::MoveGroup> move_group_;
  double velocity_scaling_;

  //! Mode.
  std::string mode_;

  int index_;

  //! Rotate sensor head after reaching home pose.
  bool rotate_;
};

} // grasp_and_place
