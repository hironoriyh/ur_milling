/*
 * MillingPath.cpp
 *
 *  Created on: Jul 26, 2017
 *      Author: Hironori Yoshida
 */

#include <ur3_milling/MillingPath.hpp>

//#include <object_detection/DetectObject.h>
//#include <object_detection/LocaliseObjects.h>

#include <visualization_msgs/Marker.h>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/conversions.h>

#include <ur_msgs/SetIO.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <locale>
#include <string>

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
  move_group_->setMaxVelocityScalingFactor(velocity_high_);
  move_group_->setMaxAccelerationScalingFactor(max_acceleration_);
  move_group_->setPlannerId("RRTConnectkConfigDefault");
  move_group_->setPlanningTime(10);
  move_group_->setNumPlanningAttempts(10);


  // Set planning scene interface.
  planning_scene_.reset(new moveit::planning_interface::PlanningSceneInterface());
  ros::Duration(3.0).sleep();

  ////// move to the origin
  // rs_ = move_group_->getCurrentState();
  // vector<double> group_variable_values;
  // rs_->copyJointGroupPositions(rs_->getRobotModel()->
  //                             getJointModelGroup(move_group_->getName()),  group_variable_values);
  //
  // std::cout << "current joint state: ";
  // for (int i = 0; i < 6; i++)
  //   std::cout << group_variable_values[i] << " , ";
  // std::cout << std::endl;

  execute_milling_server_ = nh_.advertiseService("execute_milling", &MillingPath::ExecuteMillingCB, this);
}

MillingPath::~MillingPath()
{
}

bool MillingPath::ReadParameters()
{
   nh_.getParam("/milling_path/velocity_high", velocity_high_);
   nh_.getParam("/milling_path/camera_frame", camera_frame_);
   nh_.getParam("/milling_path/distance_to_object", distance_to_object_);
   nh_.getParam("/milling_path/eef_step", eef_step_);
   nh_.getParam("/milling_path/velocity_cut", velocity_cut_);
   nh_.getParam("/milling_path/max_acceleration", max_acceleration_);

  return true;
}

bool MillingPath::LoadMillingPath()
{

  std::string line;
  std::string ur_path = ros::package::getPath("ur3_milling") + "/data/test.txt";
  ROS_INFO_STREAM("ur_path: " << ur_path);
  std::ifstream infile(ur_path);

  while (std::getline(infile, line))  // To get you all the lines.
    {
      if (line.empty()) {
  //      std::cout << "Empty line." << std::endl;
  //      continue;
      } else {
        //    ROS_INFO_STREAM("line: " << line);
        typedef std::vector<std::string> Tokens;
        Tokens tokens;
        boost::split(tokens, line, boost::is_any_of(" "));

        if (line.size() < 2) {
          // std::cout << "reject: " << line.size() << std::endl;
          } else {
          std::string::size_type sz;
          geometry_msgs::PoseStamped pose = move_group_->getCurrentPose();
          pose.pose.position.x -= std::stod(tokens[0], &sz) * 0.001;
          pose.pose.position.y -= std::stod(tokens[1], &sz) * 0.001;
          pose.pose.position.z += std::stod(tokens[2], &sz) * 0.001;
          //    ROS_INFO_STREAM("position: " << pose.pose.position.x << " , " <<pose.pose.position.y << " , " <<pose.pose.position.z);
          poses.push_back(pose);
          // std::cout << "accept: " << line.size() << std::endl;
        }
      }
    }
    std::cout << "size of points: " << poses.size() << std::endl;
    ros::Duration(1.0).sleep();
  // XmlRpc::XmlRpcValue stacking_configuration;
  // if (!nh_.getParam("configuration", stacking_configuration) || stacking_configuration.size() == 0)
  //   return false;
  // for (int i = 0; i < stacking_configuration.size(); ++i) {
  //   std::string name = static_cast<std::string>(stacking_configuration[i]["id"]);
  //   XmlRpc::XmlRpcValue pose = stacking_configuration[i]["pose"];
  //   geometry_msgs::Pose placing_pose;
  //   placing_pose.position.x = static_cast<double>(pose["position"]["x"]);
  // }
  return true;
}

bool MillingPath::ExecuteMillingCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  SetSpindle(1.0);

  ROS_INFO("Move up");
  MoveTranslation(0, 0, distance_to_object_);

  //// go to...
  ROS_INFO("Move above the line");
  double height = move_group_->getCurrentPose().pose.position.z + distance_to_object_;
  MoveAbsTranslation(poses[0].pose.position.x, poses[0].pose.position.y, height);

    //// go down
  //// go to...
  ROS_INFO("Move above the line");
  MoveTranslation(0, 0,  -distance_to_object_);


  //// trajectory move
  move_group_->setMaxVelocityScalingFactor(velocity_cut_);
  move_group_->setMaxAccelerationScalingFactor(max_acceleration_*0.1);

  for (int i = 0; i < poses.size(); i++) {
    LinearMoveToPose(poses[i]);
  }

  SetSpindle(0.0);

  return 1;
}

bool MillingPath::MoveTranslation(const double x_, const double y_, const double z_)
{
  geometry_msgs::Pose target_pose = move_group_->getCurrentPose().pose;
  ROS_INFO_STREAM(
      "MoveRelTrans: current pos : " << target_pose.position.x << " , " << target_pose.position.y);
  target_pose.position.x += x_;
  target_pose.position.y += y_;
  target_pose.position.z += z_;

  ROS_INFO_STREAM("goto : " << target_pose.position.x << " , " << target_pose.position.y);
  move_group_->setPoseTarget(target_pose);
  bool success = move_group_->plan(my_plan_);
  if(!success) return 0;
  move_group_->move();
  ros::Duration(1.0).sleep();

  return 1;
}

bool MillingPath::MoveAbsTranslation(const double x_, const double y_, const double z_)
{
  geometry_msgs::Pose target_pose = move_group_->getCurrentPose().pose;
  ROS_INFO_STREAM(
      "MoveAbsTrans: current pos : " << target_pose.position.x << " , " << target_pose.position.y);
  target_pose.position.x = x_;
  target_pose.position.y = y_;
  target_pose.position.z = z_;

  ROS_INFO_STREAM("goto : " << target_pose.position.x << " , " << target_pose.position.y);
  move_group_->setPoseTarget(target_pose);
  bool success = move_group_->plan(my_plan_);
  if(!success) return 0;
  move_group_->move();
  ros::Duration(1.0).sleep();

  return 1;
}


bool MillingPath::SetSpindle(double state){
  // turn on the spindle
  ros::ServiceClient client = nh_.serviceClient<ur_msgs::SetIO>("/ur_driver/set_io");
  ur_msgs::SetIO srv;
  srv.request.fun = 1;
  srv.request.pin = 4;
  srv.request.state = state;
  if(client.call(srv)){
    ROS_INFO("success!");
  }
  else {
    ROS_ERROR("failed to call srv");
    return 0;
  }
  return 1;
}



bool MillingPath::LinearMoveToPose(const geometry_msgs::PoseStamped& target_pose)
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
  double path_fraction = move_group_->computeCartesianPath(waypoints, eef_step_, 0.0, trajectory, true, &error_code);
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


  ROS_INFO_STREAM(
      "goto: " << target_pose.pose.position.x << " , " << target_pose.pose.position.y << " , " << target_pose.pose.position.z << " ,  point num: " << trajectory.joint_trajectory.points.size());

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
