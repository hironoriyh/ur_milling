/*
 * MillingPath.hpp
 *
 *  Created on: Jul 26, 2017
 *      Author: Hironori Yoshida
 */

# pragma once

//#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
//        MoveGroup = moveit.planning_interface.MoveGroup;
//#pragma GCC diagnostic warning "-Wdeprecated-declarations"

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


using namespace std;

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

  bool LoadMillingPath();

  bool ExecuteMillingCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  bool SetSpindle(double state);

  // bool DetectObject(std::vector<Object>& models_to_detect);

  // bool DetectObjectRotate(const int object_index);
  //
  // bool LocalizeObjects();

  bool MoveTranslation(const double x_, const double y_, const double z_);

  bool MoveAbsTranslation(const double x_, const double y_, const double z_);

  bool LinearMoveToPose(const geometry_msgs::PoseStamped& target_pose);

  bool MoveToPoses(std::vector<geometry_msgs::PoseStamped> target_poses);

  bool DetectObject(std_msgs::String model_to_detect);


  geometry_msgs::Pose GetTargetTCPPose(const int stack_model_index);

  visualization_msgs::Marker VisualizeMarker(const int marker_type, const geometry_msgs::Pose pose, const int id, const float r, const float g, const float b, const float a, const Vector3D scale = Vector3D::Ones());

	//! ROS Node handle.
	ros::NodeHandle nh_;

	//! Transform listener.
	tf::TransformListener tf_listener_;
	string world_frame_;

	//! Publisher.
	ros::Publisher mesh_publisher_;
	ros::Publisher move_pose_publisher_;

  visualization_msgs::Marker mesh_;


  //! Model locations from object localisation.
  std::vector<geometry_msgs::PoseStamped> poses;

  //! MoveIt! move group interface.
  std::shared_ptr<moveit::planning_interface::MoveGroup> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;
  robot_state::RobotStateConstPtr rs_;
  moveit::planning_interface::MoveGroup::Plan my_plan_;


  ros::ServiceServer execute_milling_server_;


  double velocity_high_;
  double distance_to_object_;
  double velocity_cut_;
  double max_acceleration_;
  double eef_step_;
  string camera_frame_;

  //! Allowed deviation to target pose.
  double position_diff_;
  double rotation_diff_;
  Vector3D min_localization_;
  Vector3D max_localization_;


};

} // ur3_milling
