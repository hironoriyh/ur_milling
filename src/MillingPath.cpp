/*
 * MillingPath.cpp
 *
 *  Created on: Jul 26, 2017
 *      Author: Hironori Yoshida
 */

#include <ur3_milling/MillingPath.hpp>

// #include <ur3_milling/GraspObject.hpp>
// #include <ur3_milling/PlaceObject.hpp>

#include <object_detection/DetectObject.h>
#include <object_detection/LocaliseObjects.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/Mesh.h>
#include <geometric_shapes/shape_messages.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit/robot_state/conversions.h>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>


namespace ur3_milling {

MillingPath::MillingPath(ros::NodeHandle& nh)
    : nh_(nh),
      world_frame_("world")
{
  // Read parameters.
  ReadParameters();

  // Publisher
  mesh_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/object_mesh", 1, true);
  move_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_pose", 1, true);

  // Set moveit interface
  move_group_.reset(new moveit::planning_interface::MoveGroup("manipulator"));
  move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
  move_group_->setPlannerId("RRTConnectkConfigDefault");
  move_group_->setPlanningTime(10);
  move_group_->setNumPlanningAttempts(10);

  // Set planning scene interface.
  planning_scene_.reset(new moveit::planning_interface::PlanningSceneInterface());

  std::vector<double> group_variable_values;
  move_group_->getCurrentState()->copyJointGroupPositions(move_group_->getCurrentState()->getRobotModel()->getJointModelGroup(move_group_->getName()), group_variable_values);
  move_group_->setJointValueTarget(group_variable_values);
 //  bool success = move_group_->plan(my_plan);
//   ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");


}

MillingPath::~MillingPath()
{
}

//bool MillinPath::

bool MillingPath::ReadParameters()
{
  // nh_.getParam("/serial_stacking/velocity_scaling", velocity_scaling_);
  // nh_.getParam("/serial_stacking/camera_frame", camera_frame_);
  // nh_.getParam("/serial_stacking/distance_to_objects", distance_to_objects_);
  // nh_.getParam("/serial_stacking/allowed_position_deviation", position_diff_);
  // nh_.getParam("/serial_stacking/allowed_rotation_deviation", rotation_diff_);
  //
  // nh_.getParam("/serial_stacking/min_localization_x", min_localization_.x());
  // nh_.getParam("/serial_stacking/min_localization_y", min_localization_.y());
  // nh_.getParam("/serial_stacking/min_localization_z", min_localization_.z());
  // nh_.getParam("/serial_stacking/max_localization_x", max_localization_.x());
  // nh_.getParam("/serial_stacking/max_localization_y", max_localization_.y());
  // nh_.getParam("/serial_stacking/max_localization_z", max_localization_.z());

  return true;
}

bool MillingPath::LoadStackingConfiguration()
{
  // desired_stacking_configuration_.clear();
  // // Load stacking configuration.
  // std::string path = ros::package::getPath("ur3_milling");
  // path = path + "/config/stacking_configuration_serial.yaml";
  // std::string commandString = "rosparam load " + path + " /serial_stacking";
  // const char* command = commandString.c_str();
  // if (system(command) != 0) {
  //   ROS_ERROR("Can't load parameter.");
  //   return false;
  // }
  //
  // XmlRpc::XmlRpcValue stacking_configuration;
  // if (!nh_.getParam("configuration", stacking_configuration) || stacking_configuration.size() == 0)
  //   return false;
  // for (int i = 0; i < stacking_configuration.size(); ++i) {
  //   std::string name = static_cast<std::string>(stacking_configuration[i]["id"]);
  //   XmlRpc::XmlRpcValue pose = stacking_configuration[i]["pose"];
  //   geometry_msgs::Pose placing_pose;
  //   placing_pose.position.x = static_cast<double>(pose["position"]["x"]);
  //   placing_pose.position.y = static_cast<double>(pose["position"]["y"]);
  //   placing_pose.position.z = static_cast<double>(pose["position"]["z"]);
  //   placing_pose.orientation.x = static_cast<double>(pose["orientation"]["x"]);
  //   placing_pose.orientation.y = static_cast<double>(pose["orientation"]["y"]);
  //   placing_pose.orientation.z = static_cast<double>(pose["orientation"]["z"]);
  //   placing_pose.orientation.w = static_cast<double>(pose["orientation"]["w"]);
  //   Object object(name);
  //   object.setPlacingPose(placing_pose);
  //   desired_stacking_configuration_.push_back(object);
  // }
  return true;
}



bool MillingPath::MoveToPose(const geometry_msgs::PoseStamped& target_pose)
{
  move_pose_publisher_.publish(target_pose);
  geometry_msgs::PoseStamped current_pose = move_group_->getCurrentPose();
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose.pose);
  waypoints.push_back(target_pose.pose);

  robot_state::RobotState rs = *move_group_->getCurrentState();
  move_group_->setStartState(rs);

  // Plan trajectory.
  moveit_msgs::RobotTrajectory trajectory;
  moveit_msgs::MoveItErrorCodes error_code;
  double path_fraction = move_group_->computeCartesianPath(waypoints, 0.03, 0.0, trajectory, true, &error_code);
  if(path_fraction < 1.0) {
    ROS_WARN_STREAM("MillingPath: Could not calculate Cartesian Path.");
    return false;
  }

  robot_trajectory::RobotTrajectory robot_trajectory(rs.getRobotModel(), move_group_->getName());
  robot_trajectory.setRobotTrajectoryMsg(rs, trajectory);

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  iptp.computeTimeStamps(robot_trajectory, 1.0);
  robot_trajectory.getRobotTrajectoryMsg(trajectory);

  moveit::planning_interface::MoveGroup::Plan my_plan;
  moveit_msgs::RobotState robot_state_msg;
  robot_state::robotStateToRobotStateMsg(*move_group_->getCurrentState(), robot_state_msg);

  my_plan.trajectory_ = trajectory;
  my_plan.start_state_ = robot_state_msg;

  // removing points with velocity zero
  my_plan.trajectory_.joint_trajectory.points.erase(
      std::remove_if(my_plan.trajectory_.joint_trajectory.points.begin() + 1, my_plan.trajectory_.joint_trajectory.points.end(), [](const trajectory_msgs::JointTrajectoryPoint & p)
      { return p.time_from_start.toSec() == 0;}),
      my_plan.trajectory_.joint_trajectory.points.end());

  move_group_->execute(my_plan);
  return true;
}

bool MillingPath::MoveToPoses(std::vector<geometry_msgs::PoseStamped> target_poses)
{

  // target_pose_publisher_.publish(target_poses.back());
  geometry_msgs::PoseStamped current_pose = move_group_->getCurrentPose();
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose.pose);
  for(auto pt : target_poses) waypoints.push_back(pt.pose);

  robot_state::RobotState rs = *move_group_->getCurrentState();
  move_group_->setStartState(rs);

  // Plan trajectory.
  moveit_msgs::RobotTrajectory trajectory;
  moveit_msgs::MoveItErrorCodes error_code;
  double path_fraction = move_group_->computeCartesianPath(waypoints, 0.03, 0.0, trajectory, true, &error_code);
  if(path_fraction < 1.0) {
    ROS_WARN_STREAM("PlacesObject: Could not calculate Cartesian Path.");
    return false;
  }

  robot_trajectory::RobotTrajectory robot_trajectory(rs.getRobotModel(), move_group_->getName());
  robot_trajectory.setRobotTrajectoryMsg(rs, trajectory);

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  iptp.computeTimeStamps(robot_trajectory, 1.0);
  robot_trajectory.getRobotTrajectoryMsg(trajectory);

  moveit::planning_interface::MoveGroup::Plan my_plan;
  moveit_msgs::RobotState robot_state_msg;
  robot_state::robotStateToRobotStateMsg(*move_group_->getCurrentState(), robot_state_msg);

  my_plan.trajectory_ = trajectory;
  my_plan.start_state_ = robot_state_msg;


  // removing points with velocity zero
  my_plan.trajectory_.joint_trajectory.points.erase(
      std::remove_if(my_plan.trajectory_.joint_trajectory.points.begin() + 1, my_plan.trajectory_.joint_trajectory.points.end(), [](const trajectory_msgs::JointTrajectoryPoint & p)
      { return p.time_from_start.toSec() == 0;}),
      my_plan.trajectory_.joint_trajectory.points.end());

  // check the joint 6 rotation:
  // double diff_joint_6 =  my_plan.trajectory_.joint_trajectory.points.back().positions[5] - my_plan.trajectory_.joint_trajectory.points[0].positions[5];
  // if(diff_joint_6 > M_PI_2) ROS_WARN_STREAM( "joint 6 diff: " << diff_joint_6 );

  move_group_->setPlanningTime(60);
  move_group_->execute(my_plan);
  return true;
}




// bool MillingPath::CheckCollision(const geometry_msgs::Pose pose)
// {
//   ROS_INFO_STREAM("MillingPath: Check collision.");
//   move_group_->setJointValueTarget(pose);
//   robot_state::RobotState grasping_state = move_group_->getJointValueTarget();
//   moveit_msgs::RobotState grasping_state_msg;
//   robot_state::robotStateToRobotStateMsg(grasping_state, grasping_state_msg);
//   moveit_msgs::GetStateValidity get_state_validity;
//   get_state_validity.request.group_name = move_group_->getName();
//   get_state_validity.request.robot_state = grasping_state_msg;
//
//   if (!check_validity_.waitForExistence(ros::Duration(1.0))) {
//     ROS_ERROR_STREAM("CheckCollision: Check validity service not advertised.");
//     return false;
//   }
//   if (!check_validity_.call(get_state_validity)) {
//     ROS_ERROR_STREAM("CheckCollision: Could not call check validity service.");
//     return false;
//   }
// //  ROS_INFO_STREAM("state validity: " << get_state_validity.response);
// //  if(!get_state_validity.response.valid){
// //  ROS_INFO("CheckCollision not valid");
// //  ROS_INFO_STREAM("state validity: " << get_state_validity.response.constraint_result.size());
// //    for(int i =0; i< get_state_validity.response.constraint_result.size() ; i++){
// //      ROS_INFO_STREAM("state validity: " << get_state_validity.response.constraint_result.at(i) );
// //    }
// //  }
//
//   return get_state_validity.response.valid;
// }

// geometry_msgs::Pose MillingPath::GetTargetTCPPose(const int stack_model_index)
// {
//   tf::Transform object_target_pose;
//   tf::poseMsgToTF(desired_stacking_configuration_[stack_model_index].getPlacingPose(), object_target_pose);
//
//   tf::Transform object_to_tcp;
//   tf::poseMsgToTF(desired_stacking_configuration_[stack_model_index].getRefinedGraspingPose(), object_to_tcp);
//
//   tf::Transform tcp_target_tf = object_target_pose * object_to_tcp;
//   geometry_msgs::Pose tcp_target_pose;
//   tf::poseTFToMsg(tcp_target_tf, tcp_target_pose);
//
//   return tcp_target_pose;
// }


visualization_msgs::Marker MillingPath::VisualizeMarker(const int marker_type,
                                                           const geometry_msgs::Pose pose,
                                                           const int id, const float r,
                                                           const float g, const float b,
                                                           const float a, const Vector3D scale)
{
  visualization_msgs::Marker marker;
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  marker.header.frame_id = world_frame_;
  marker.id = id;
  marker.type = marker_type;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.stamp = ros::Time::now();
  marker.pose = pose;
  marker.scale.x = scale.x();
  marker.scale.y = scale.y();
  marker.scale.z = scale.z();
  return marker;
}

}
