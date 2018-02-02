/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/conversions.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <locale>
#include <string>

#include <ros/package.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "milling_path");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();


  /* This sleep is ONLY to allow Rviz to come up */
  moveit::planning_interface::MoveGroup group("manipulator");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  moveit::planning_interface::MoveGroup::Plan my_plan;
  ros::Publisher target_pose_publisher_ = node_handle.advertise<geometry_msgs::PoseStamped>("/move_pose", 1, true);
  // target_pose_publisher_.publish(target_poses.back());

  group.setMaxVelocityScalingFactor(0.05);
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setPlanningTime(5);
  group.setNumPlanningAttempts(10);

  std::vector<double> group_variable_values;
  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

  group_variable_values= {0.4497567415237427, -1.8413293997394007, -1.5451634565936487, -2.9070408979998987, -1.8363669554339808, -3.1571934858905237};
  group.setJointValueTarget(group_variable_values);
  bool success = group.plan(my_plan);
  ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");

  if (1)
  {
    ROS_INFO("Visualizing plan 1 (again)");
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher.publish(display_trajectory);
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(1.0);
  }
  group.execute(my_plan);
  sleep(1.0);

  //------------------------ move up -------------------------
  ROS_INFO("Move up");
  geometry_msgs::Pose target_pose = group.getCurrentPose().pose;
  target_pose.position.z += 0.05;
  group.setPoseTarget(target_pose);
  success = group.plan(my_plan);
  group.move();
  sleep(1.0);


  std::string line;
  std::string ur_path = ros::package::getPath("ur3_milling") + "/data/test.txt";
  ROS_INFO_STREAM("ur_path: " << ur_path);
  std::ifstream infile(ur_path);

  std::vector<std::string> mylines;
  std::vector<geometry_msgs::PoseStamped> poses;
  while (std::getline(infile, line))  // To get you all the lines.
  {
    mylines.push_back(line);
//    std::cout<< count << " line: " <<  line << std::endl;
    typedef std::vector<std::string> Tokens;
    Tokens tokens;
    boost::split( tokens, line, boost::is_any_of(" ") );
    std::string::size_type sz;
    geometry_msgs::PoseStamped pose = group.getCurrentPose();
    pose.pose.position.x -= std::stod(tokens[1], &sz)*0.001;
    pose.pose.position.y += std::stod(tokens[0], &sz)*0.001;
    pose.pose.position.z += std::stod(tokens[2], &sz)*0.001;
    poses.push_back(pose);
  }
  std::cout << "size of string: " << mylines.size() <<  std::endl;
  std::cout << "size of points: " << poses.size() <<  std::endl;

  group.setMaxVelocityScalingFactor(0.001);

  for(int i=0; i< poses.size(); i++){
    geometry_msgs::PoseStamped current_pose = group.getCurrentPose();
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(current_pose.pose);
    waypoints.push_back(poses[i].pose);

    robot_state::RobotState rs = *group.getCurrentState();
    group.setStartState(rs);

    // Plan trajectory.
    moveit_msgs::RobotTrajectory trajectory;
    moveit_msgs::MoveItErrorCodes error_code;
    double path_fraction = group.computeCartesianPath(waypoints, 0.00002, 0.0, trajectory, true, &error_code);
    if(path_fraction < 1.0) {
      ROS_WARN_STREAM("Could not calculate Cartesian Path.");
      return false;
    }

    robot_trajectory::RobotTrajectory robot_trajectory(rs.getRobotModel(), group.getName());
    robot_trajectory.setRobotTrajectoryMsg(rs, trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(robot_trajectory, 1.0);
    robot_trajectory.getRobotTrajectoryMsg(trajectory);

//    moveit::planning_interface::MoveGroup::Plan my_plan;
    moveit_msgs::RobotState robot_state_msg;
    robot_state::robotStateToRobotStateMsg(*group.getCurrentState(), robot_state_msg);

    ROS_INFO_STREAM("joint trajectory size: " << trajectory.joint_trajectory.points.size() );
//    trajectory.joint_trajectory.points[1].time_from_start = trajectory.joint_trajectory.points[0].time_from_start + ros::Duration(1.0);
//    for(int j=0; j<trajectory.joint_trajectory.points.size() ; j++){
//      ROS_INFO_STREAM("joint trajectory points: " << trajectory.joint_trajectory.points[j].positions[0]
//                        << " time: "  << trajectory.joint_trajectory.points[j].time_from_start);
//    }

    my_plan.trajectory_ = trajectory;
    my_plan.start_state_ = robot_state_msg;

    // removing points with velocity zero
    my_plan.trajectory_.joint_trajectory.points.erase(
        std::remove_if(my_plan.trajectory_.joint_trajectory.points.begin() + 1, my_plan.trajectory_.joint_trajectory.points.end(), [](const trajectory_msgs::JointTrajectoryPoint & p)
        { return p.time_from_start.toSec() == 0;}),
        my_plan.trajectory_.joint_trajectory.points.end());
    group.execute(my_plan);
  }

  ros::shutdown();
  return 0;
}

int test_(){
  int x = 10;
  return x;
}

bool MoveToPose(moveit::planning_interface::MoveGroup move_group_, const geometry_msgs::PoseStamped& target_pose, double eef_step)
{

  // approaching_pose_publisher_.publish(target_pose);
  geometry_msgs::PoseStamped current_pose = move_group_.getCurrentPose();
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose.pose);
  waypoints.push_back(target_pose.pose);

  robot_state::RobotState rs = *move_group_.getCurrentState();
  move_group_.setStartState(rs);

  // Plan trajectory.
  moveit_msgs::RobotTrajectory trajectory;
  moveit_msgs::MoveItErrorCodes error_code;
  double path_fraction = move_group_.computeCartesianPath(waypoints, eef_step, 0.0, trajectory, true, &error_code);
  if(path_fraction < 1.0) {
    ROS_WARN_STREAM("GraspObject: Could not calculate Cartesian Path.");
    return false;
  }

  robot_trajectory::RobotTrajectory robot_trajectory(rs.getRobotModel(), move_group_.getName());
  robot_trajectory.setRobotTrajectoryMsg(rs, trajectory);

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  iptp.computeTimeStamps(robot_trajectory, 1.0);
  robot_trajectory.getRobotTrajectoryMsg(trajectory);

  moveit::planning_interface::MoveGroup::Plan my_plan;
  moveit_msgs::RobotState robot_state_msg;
  robot_state::robotStateToRobotStateMsg(*move_group_.getCurrentState(), robot_state_msg);

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

  move_group_.execute(my_plan);
  return true;
}

bool MoveToPoses(moveit::planning_interface::MoveGroup move_group_,
                 std::vector<geometry_msgs::PoseStamped> target_poses)
{

  //  move_group_.setJointValueTarget(target_pose);
  geometry_msgs::PoseStamped current_pose = move_group_.getCurrentPose();
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose.pose);
  for(auto pt : target_poses) waypoints.push_back(pt.pose);

  robot_state::RobotState rs = *move_group_.getCurrentState();
  move_group_.setStartState(rs);

  // Plan trajectory.
  moveit_msgs::RobotTrajectory trajectory;
  moveit_msgs::MoveItErrorCodes error_code;
  double path_fraction = move_group_.computeCartesianPath(waypoints, 0.03, 0.0, trajectory, true, &error_code);
  if(path_fraction < 1.0) {
    ROS_WARN_STREAM("PlacesObject: Could not calculate Cartesian Path.");
    return false;
  }

  robot_trajectory::RobotTrajectory robot_trajectory(rs.getRobotModel(), move_group_.getName());
  robot_trajectory.setRobotTrajectoryMsg(rs, trajectory);

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  iptp.computeTimeStamps(robot_trajectory, 1.0);
  robot_trajectory.getRobotTrajectoryMsg(trajectory);

  moveit::planning_interface::MoveGroup::Plan my_plan;
  moveit_msgs::RobotState robot_state_msg;
  robot_state::robotStateToRobotStateMsg(*move_group_.getCurrentState(), robot_state_msg);

  my_plan.trajectory_ = trajectory;
  my_plan.start_state_ = robot_state_msg;


  // removing points with velocity zero
  my_plan.trajectory_.joint_trajectory.points.erase(
      std::remove_if(my_plan.trajectory_.joint_trajectory.points.begin() + 1, my_plan.trajectory_.joint_trajectory.points.end(), [](const trajectory_msgs::JointTrajectoryPoint & p)
      { return p.time_from_start.toSec() == 0;}),
      my_plan.trajectory_.joint_trajectory.points.end());

  // check the joint 6 rotation:
  double diff_joint_6 =  my_plan.trajectory_.joint_trajectory.points.back().positions[5] - my_plan.trajectory_.joint_trajectory.points[0].positions[5];
  if(diff_joint_6 > M_PI_2) ROS_WARN_STREAM( "joint 6 diff: " << diff_joint_6 );

  move_group_.setPlanningTime(60);
  move_group_.execute(my_plan);
  return true;
}
