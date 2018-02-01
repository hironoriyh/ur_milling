/*
 * PlaceObject.cpp
 *
 *  Created on: 4.08.2017
 *      Author: Hironori Yoshida
 */
#include <grasp_and_place/PlaceObject.hpp>
#include <moveit/robot_state/conversions.h>
#include <grasp_and_place_msgs/HandStates.h>
#include <inttypes.h>
#include <object_detection/DetectObject.h>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>


namespace grasp_and_place {

PlaceObject::PlaceObject(ros::NodeHandle& nh,
                         std::shared_ptr<GripperInterface> gripper_interface,
						 moveit::planning_interface::MoveGroupPtr  move_group,
                         geometry_msgs::Pose target_pose)
    : nh_(nh),
      gripper_interface_(gripper_interface),
      move_group_(move_group),
      target_pose_(target_pose)
{
  ReadParameters();
  target_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_pose", 1, true);
  refined_pose_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/object_mesh_refined", 1, true);
}


PlaceObject::~PlaceObject()
{
}

bool PlaceObject::ReadParameters()
{
  if(!nh_.getParam("/serial_stacking/place_object/approaching_distance", approaching_distance_)) approaching_distance_ = 0.01;
  if(!nh_.getParam("/serial_stacking/place_object/eef_step", eef_step_)) eef_step_ = 0.001;
  if(!nh_.getParam("/serial_stacking/place_object/incremental_movement_timeout", timeout_)) timeout_ = 10;
  if(!nh_.getParam("/serial_stacking/place_object/incremental_movement_distance", incremental_movement_distance_)) incremental_movement_distance_ = 0.1;
  if(!nh_.getParam("/serial_stacking/place_object/approaching_distance_release", approaching_distance_release_)) approaching_distance_release_ = 0.1;
  return true;
}

bool PlaceObject::ExecutePlacing()
{
  ROS_INFO("PlaceObject");
  // geometry_msgs::Pose target_pose = GetTargetTCPPose(object_index);

  // Set target pose.
  geometry_msgs::PoseStamped current_pose = move_group_->getCurrentPose();

  ROS_INFO("PlaceObject : move up and rotate" );
  // Do first translation and then rotation
  geometry_msgs::PoseStamped approaching_pose_origin, approaching_pose_intermediate, approaching_pose_above;
  approaching_pose_origin = current_pose;
  approaching_pose_origin.pose.orientation = target_pose_.orientation;
  if (!MoveToPose(approaching_pose_origin, 0.05, true)) {
    return false;
  }

  ROS_INFO("move above target through intermediate" );
  approaching_pose_intermediate = approaching_pose_origin;
  std::vector< geometry_msgs::PoseStamped> target_poses_;
  // move to the intermediate point
  approaching_pose_intermediate.pose.position.x = 0.35;
  approaching_pose_intermediate.pose.position.y = 0.25;
//  approaching_pose_intermediate.pose.position.z = move_group_->getCurrentPose().pose.position.z;
  target_poses_.push_back(approaching_pose_intermediate);

  approaching_pose_above = approaching_pose_intermediate;
//  PoseRefinement(); // target_pose must be fulfilled first
  approaching_pose_above.pose.position.x = target_pose_.position.x;
  approaching_pose_above.pose.position.y = target_pose_.position.y;
  approaching_pose_above.pose.position.z = target_pose_.position.z + approaching_distance_;
  target_poses_.push_back(approaching_pose_above);

  int cnt = 0;
  while(!MoveToPose(target_poses_)) {
    ROS_WARN("failed to plan");
    cnt ++;
    current_pose = move_group_->getCurrentPose();
    current_pose.pose.position.x += 0.1;
    MoveToPose(current_pose, 0.005, true);
    if(cnt > 10) {
      ROS_ERROR("coldn't find after 10 trials");
      return false;
    }
  }


  if(!IncrementalMoveTCP(force_threshold_, eef_step_, incremental_movement_distance_, timeout_)){
    ROS_WARN("IncrementalMoveTCP was failed");
    return false;
  }

  ros::Duration(1.0).sleep();

  // Detach object.
  move_group_->detachObject();

  ROS_INFO("After placing and move to position above the object.");
  current_pose = move_group_->getCurrentPose();
  current_pose.pose.position.z += approaching_distance_release_;

  if (!MoveToPose(current_pose, 0.03, true))
  {
    ROS_WARN_STREAM("PlaceObject: Could not move to pose: " << current_pose);
    return false;
  }

  return true;
}

bool PlaceObject::IncrementalMoveTCP(const double threshold, const double eef_step, const double max_distance, const double timeout)
{

  ROS_INFO("PlaceObject: Start incremental move in TCP frame.");
  move_group_->setMaxVelocityScalingFactor(0.001);

  ros::Time start = ros::Time::now();

  bool collision = false;
  geometry_msgs::PoseStamped current_pose = move_group_->getCurrentPose();
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header = current_pose.header;
  goal_pose.pose = target_pose_;
  goal_pose.pose.position.z -= 0.003;
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose.pose);
  waypoints.push_back(goal_pose.pose);

  robot_state::RobotState rs = *move_group_->getCurrentState();
  move_group_->setStartState(rs);

  // Plan trajectory.
  moveit_msgs::RobotTrajectory trajectory;
  moveit_msgs::MoveItErrorCodes error_code;
  double path_fraction = move_group_->computeCartesianPath(waypoints, eef_step, 0.0, trajectory, false, &error_code);

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
      { return p.time_from_start.toSec() == 0;}), my_plan.trajectory_.joint_trajectory.points.end());


  // start sampling
  if (!gripper_interface_->ClientHandCommand("sampling")) {
    ROS_WARN("Execute Grasping: sampling failed");
    return false;
  }

  int8_t collision_check = gripper_interface_->GetHandState().collision;
  printf("pre collision result: %d \n", collision_check);

//  if(collision_check) {
//    ROS_WARN("collision is already detected!");
//    return true;
//  }
//  ros::Duration(0.5).sleep();

  // removing points with velocity zero
  my_plan.trajectory_.joint_trajectory.points.erase(
    std::remove_if(my_plan.trajectory_.joint_trajectory.points.begin() + 1, my_plan.trajectory_.joint_trajectory.points.end(), [](const trajectory_msgs::JointTrajectoryPoint & p)
    { return p.time_from_start.toSec() == 0;}),
    my_plan.trajectory_.joint_trajectory.points.end());

  move_group_->execute(my_plan);

  ROS_INFO_STREAM("robot state: effort: " << move_group_->getCurrentState()->hasVelocities());
  while(move_group_->getCurrentState()->hasVelocities() ){
	ROS_INFO_STREAM("robot state: effort: " << move_group_->getCurrentState()->hasEffort());
    collision_check = gripper_interface_->GetHandState().collision;
    printf("%d \n", collision_check);
    if ((ros::Time::now() - start).toSec() > timeout) {
      ROS_WARN("Incremental movement: Timeout.");
      // move_group_->stop();
      gripper_interface_->ClientHandCommand("c-open");
      move_group_->stop();
      return true;
    }
     ros::spinOnce();
     ros::Duration(0.1).sleep();
  }
  gripper_interface_->ClientHandCommand("c-open");
  move_group_->stop();

  ROS_WARN("collision was detected!");
  return true;
}


bool PlaceObject::PoseRefinement()
{
  ROS_INFO("SerialStacking: RefinePose");
  ros::ServiceClient client = nh_.serviceClient<object_detection::DetectObject>("/object_pose_refinement");
  object_detection::DetectObject srv;

  std_msgs::String model_id;
  model_id.data = object_.getName();
  srv.request.models_to_detect.push_back(model_id);

  if (!client.waitForExistence(ros::Duration(2.0))) {
    return false;
  }

  bool stone_pose_refined = false;
  int iter = 0;

  while (!stone_pose_refined && iter <= 3) {
    iter++;
    client.call(srv);
    if (srv.response.model_ids.size() == 1) {
      stone_pose_refined = true;
    }
  }

  if (srv.response.model_ids.size() == 0) stone_pose_refined = false;
  std::string world_frame_ ="world";
  std::string camera_frame_ ="sr300_depth_optical_frame";

  const ros::Time time = ros::Time::now();
  geometry_msgs::PoseStamped model_tf, object_pose;
  // Model Pose in TCP frame.
  model_tf.header.frame_id = "tool0";
  model_tf.header.stamp = time;
  // Model Pose in world frame.
  object_pose.header.frame_id = world_frame_;
  object_pose.header.stamp = time;

  if (stone_pose_refined) {
    ROS_INFO("SerialStacking: Pose refinement executed.");

    // Model pose in camera frame.
    geometry_msgs::PoseStamped model_camera_pose;
    model_camera_pose.header.frame_id = camera_frame_;
    model_camera_pose.header.stamp = time;
    model_camera_pose.pose = srv.response.detected_model_poses[0];
    // model_camera_pose.pose.position.x -= 0.03;

    try {
      const ros::Duration timeout(1);
      const ros::Duration polling_sleep_duration(4);
      std::string* error_msg = NULL;

      tf_listener_.waitForTransform("tool0", camera_frame_, time, timeout, polling_sleep_duration, error_msg);
      tf_listener_.transformPose("tool0", model_camera_pose, model_tf);
      tf_listener_.waitForTransform(world_frame_, camera_frame_, time, timeout, polling_sleep_duration, error_msg);
      tf_listener_.transformPose(world_frame_, model_camera_pose, object_pose);

    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    // Set updated model position.
    object_.setPose(object_pose.pose);
  } else {
    ROS_WARN("SerialStacking: Refinement not successful.");
    // Assume grasp without displacement.
    geometry_msgs::PoseStamped grasping_pose, object_pose_old;
    grasping_pose.header.stamp = time;
    grasping_pose.header.frame_id = world_frame_;
    object_pose_old.header.stamp = time;
    object_pose_old.header.frame_id = world_frame_;
    grasping_pose.pose = object_.getGraspingPoses().at(0);
    object_pose_old.pose = object_.getPose();
    tf::Transform world_to_tcp;
    tf::Transform world_to_model;
    tf::poseMsgToTF(grasping_pose.pose, world_to_tcp);
    tf::poseMsgToTF(object_pose_old.pose, world_to_model);
    tf::Transform tcp_to_model = world_to_tcp.inverse() * world_to_model;
    tf::poseTFToMsg(tcp_to_model, model_tf.pose);

    tf_listener_.waitForTransform(world_frame_, "tool0", time, ros::Duration(1.0));
    tf_listener_.transformPose(world_frame_, model_tf, object_pose);
    object_.setPose(object_pose.pose);
  }

  // Visualize object mesh.
  const int marker_id = 0;
  visualization_msgs::Marker target_marker = VisualizeMarker(visualization_msgs::Marker::MESH_RESOURCE, object_pose.pose, marker_id, 0, 1, 0, 0.3);
  std::string target_pose_name = object_.getName();
  std::string model_path =  "package://urdf_models/models/"  + object_.getName() + "/mesh/" + object_.getName() + ".stl";
  target_marker.mesh_resource = model_path;
  refined_mesh_.markers.clear();
  refined_mesh_.markers.push_back(target_marker);
  refined_pose_publisher_.publish(refined_mesh_);

  return stone_pose_refined;
}

visualization_msgs::Marker PlaceObject::VisualizeMarker(const int marker_type,
                                                           const geometry_msgs::Pose pose,
                                                           const int id, const float r,
                                                           const float g, const float b,
                                                           const float a)
{
  visualization_msgs::Marker marker;
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  marker.header.frame_id = "world";
  marker.id = id;
  marker.type = marker_type;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.stamp = ros::Time::now();
  marker.pose = pose;
  return marker;
}


bool PlaceObject::MoveToPose(const geometry_msgs::PoseStamped& target_pose, double eef_step, bool check_collision)
{

  //  move_group_->setJointValueTarget(target_pose);
  target_pose_publisher_.publish(target_pose);
  geometry_msgs::PoseStamped current_pose = move_group_->getCurrentPose();
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose.pose);
  waypoints.push_back(target_pose.pose);

  robot_state::RobotState rs = *move_group_->getCurrentState();
  move_group_->setStartState(rs);

  // Plan trajectory.
  moveit_msgs::RobotTrajectory trajectory;
  moveit_msgs::MoveItErrorCodes error_code;
  double path_fraction = move_group_->computeCartesianPath(waypoints, eef_step, 0.0, trajectory, check_collision, &error_code);
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

  move_group_->execute(my_plan);
  return true;
}

bool PlaceObject::MoveToPose(std::vector<geometry_msgs::PoseStamped> target_poses)
{

  //  move_group_->setJointValueTarget(target_pose);
  target_pose_publisher_.publish(target_poses.back());
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
  double diff_joint_6 =  my_plan.trajectory_.joint_trajectory.points.back().positions[5] - my_plan.trajectory_.joint_trajectory.points[0].positions[5];
  if(diff_joint_6 > M_PI_2) ROS_WARN_STREAM( "joint 6 diff: " << diff_joint_6 );

  move_group_->setPlanningTime(60);
  move_group_->execute(my_plan);
  return true;
}



void PlaceObject::SetModel(const Object& object)
{
  object_ = object;
}

}
