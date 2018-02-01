/*
 * GraspObject.cpp
 *
 *  Created on: 31.07.2017
 *      Author: Hironori Yoshida
 */

#include "grasp_and_place/GraspObject.hpp"
#include <visualization_msgs/Marker.h>
#include <object_detection/DetectObject.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/conversions.h>



namespace grasp_and_place {

GraspObject::GraspObject(ros::NodeHandle& nh, std::shared_ptr<GripperInterface> gripper_interface,
                         std::shared_ptr<moveit::planning_interface::MoveGroup> move_group,
                         geometry_msgs::Pose& object_pose, geometry_msgs::Pose& grasping_pose,
                         std::vector<double> finger_pos)
    : nh_(nh),
      gripper_interface_(gripper_interface),
      move_group_(move_group),
      object_pose_(object_pose),
      grasping_pose_(grasping_pose),
      finger_pos_(finger_pos)
{
  if (!ReadParameters()) ROS_WARN("GraspObject: could not read all parameters.");
  approaching_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_pose", 1, true);
  mesh_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/object_mesh", 1, true);
}

// pass grasping config
GraspObject::GraspObject(ros::NodeHandle& nh, std::shared_ptr<GripperInterface> gripper_interface,
                         std::shared_ptr<moveit::planning_interface::MoveGroup> move_group,
                         geometry_msgs::Pose& object_pose, GraspingConfigruation grasping_config)
    : nh_(nh),
      gripper_interface_(gripper_interface),
      move_group_(move_group),
      object_pose_(object_pose),
      grasping_pose_(grasping_config.pose),
      finger_pos_(grasping_config.finger_positions),
      grasping_index_(grasping_config.grasp_pose_index)
{
  if (!ReadParameters()) ROS_WARN("GraspObject: could not read all parameters.");
  approaching_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_pose", 1, true);
  mesh_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/object_mesh", 1, true);
}

GraspObject::~GraspObject()
{
}

bool GraspObject::ReadParameters()
{
  if(!nh_.getParam("/serial_stacking/grasp_object/approaching_distance", approaching_distance_)) approaching_distance_ = 0.01;

  return true;
}

bool GraspObject::ExecuteGrasping()
{
  // Add gripper command publisher
  ROS_INFO_STREAM("GraspObject: ExecuteGrasping.");
  ROS_WARN_STREAM("Grasping Index is " <<grasping_index_);

  // Get target pose.
  geometry_msgs::PoseStamped target_pose, current_pose;

  current_pose = move_group_->getCurrentPose();
  target_pose.header = current_pose.header; // note that it is in tool0 frame (z is inverted!)
  target_pose.pose = grasping_pose_;
  // Move to position above object.
  ROS_INFO("Move to position above the object.");
  geometry_msgs::PoseStamped approaching_pose_translation;
  approaching_pose_translation.header = current_pose.header;
  approaching_pose_translation.pose.position = target_pose.pose.position;
  approaching_pose_translation.pose.position.z = current_pose.pose.position.z;
  approaching_pose_translation.pose.orientation =current_pose.pose.orientation;
//  ROS_INFO_STREAM("Execute Grasping: move to approaching_pose_translation \n" << approaching_pose_translation.pose);
  if (!MoveToPose(approaching_pose_translation, 0.03)) {
    return false;
  }

  approaching_pose_translation.pose.position.z = target_pose.pose.position.z + 0.05;
  approaching_pose_translation.pose.orientation = target_pose.pose.orientation;

  grasp_failed:
  if (!MoveToPose(approaching_pose_translation, 0.03)) {
    return false;
  }
  // pose refinement
  geometry_msgs::PoseStamped refined_pose = current_pose;
  if(!PoseRefinement(refined_pose)){
    ROS_WARN("pose refinement failed");
    return false;
  }

  // Prepare Gripper for grasping.
  gripper_interface_->ClientHandCommand("set_positions", finger_pos_);
  ros::Duration(1.0).sleep();

  // go to the specified target pose
  if (!MoveToPose(refined_pose, 0.01))
  {
    return false;
  }

  // close gripper
  bool grasp_success = false;
  int trial = 3;
  for (int i =0; i < trial; i++){
    grasp_success = gripper_interface_->ClientHandCommand("c-close");
    if(grasp_success) break;
    else
    {
      ROS_WARN("Execute Grasping: grasp failed");
      grasp_success = gripper_interface_->ClientHandCommand("open2");
      ros::Duration(2.0).sleep();
      gripper_interface_->ClientHandCommand("set_positions", finger_pos_);
      ros::Duration(1.0).sleep();
      grasp_success = gripper_interface_->ClientHandCommand("c-close");
      if(i == trial-1){
        ROS_WARN("Execute Grasping: grasp failed 3 times!");
        return false;
      }
    }
    ros::Duration(1.0).sleep();
  }

  current_pose = move_group_->getCurrentPose();
  current_pose.pose.position.z += 0.1; // 0.1 cannot find trajectory
//  ROS_INFO_STREAM("Execute Grasping: moving up \n " << current_pose); // same as target_pose
  if (!MoveToPose(current_pose, 0.005))
  {
    return false;
  }

  ros::Duration(1.0).sleep();
  int8_t object_slipped = gripper_interface_->GetHandState().object_slipped;
  ROS_WARN("object slippage: ");
  printf("%d \n", object_slipped);

  if(object_slipped == 1){
    ROS_WARN("object is slipped");
    goto grasp_failed;
//    return false;
  }

  ROS_INFO("Execute Grasping: grasp success!");

  return true;
}

bool GraspObject::MoveToPose(const geometry_msgs::PoseStamped& target_pose, double eef_step)
{

  approaching_pose_publisher_.publish(target_pose);
  geometry_msgs::PoseStamped current_pose = move_group_->getCurrentPose();
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose.pose);
  waypoints.push_back(target_pose.pose);

  robot_state::RobotState rs = *move_group_->getCurrentState();
  move_group_->setStartState(rs);

  // Plan trajectory.
  moveit_msgs::RobotTrajectory trajectory;
  moveit_msgs::MoveItErrorCodes error_code;
  double path_fraction = move_group_->computeCartesianPath(waypoints, eef_step, 0.0, trajectory, true, &error_code);
  if(path_fraction < 1.0) {
    ROS_WARN_STREAM("GraspObject: Could not calculate Cartesian Path.");
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

  move_group_->execute(my_plan);
  return true;
}

bool GraspObject::PoseRefinement(geometry_msgs::PoseStamped & pose)
{
  ROS_INFO("GraspObject: RefinePose");
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

   while (!stone_pose_refined && iter <= 0) {
     iter++;
     client.call(srv);
     if (srv.response.model_ids.size() == 1) {
       stone_pose_refined = true;
     }
   }

   if (srv.response.model_ids.size() == 0) stone_pose_refined = false;

   const ros::Time time = ros::Time::now();
   geometry_msgs::PoseStamped model_tf, object_pose;
   // Model Pose in TCP frame.
   model_tf.header.frame_id = "tool0";
   model_tf.header.stamp = time;
   // Model Pose in world frame.
   std::string world_frame_ ="world";
   object_pose.header.frame_id = world_frame_;
   object_pose.header.stamp = time;

   if (stone_pose_refined) {
     ROS_INFO("GraspObject: Pose refinement executed.");
     // Model pose in camera frame.
     geometry_msgs::PoseStamped model_camera_pose;
     std::string camera_frame_ ="sr300_depth_optical_frame";
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

     // Visualize object mesh.
    ROS_INFO("GraspObject: Refinement successful.");
    const int marker_id = 0;
    visualization_msgs::Marker target_marker = VisualizeMarker(visualization_msgs::Marker::MESH_RESOURCE,
                                                               object_pose.pose, marker_id, 0, 0, 1, 0.5);
    std::string model_path =  "package://urdf_models/models/"  + object_.getName() + "/mesh/" + object_.getName() + ".stl";
    target_marker.mesh_resource = model_path;
    refined_mesh_.markers.clear();
    refined_mesh_.markers.push_back(target_marker);
    mesh_publisher_.publish(refined_mesh_);

    // get grasping pose
    geometry_msgs::Pose grasp_pose_original = object_.getTargetGraspingPose();
    geometry_msgs::Pose grasp_pose_transformed;
    Eigen::Isometry3d object_pose_eigen, grasp_pose_original_eigen, grasp_pose_transformed_eigen;
    tf::poseMsgToEigen(object_pose.pose, object_pose_eigen);
    tf::poseMsgToEigen(grasp_pose_original, grasp_pose_original_eigen);
    grasp_pose_transformed_eigen = object_pose_eigen * grasp_pose_original_eigen;
    tf::poseEigenToMsg(grasp_pose_transformed_eigen, grasp_pose_transformed);
    pose.pose= grasp_pose_transformed;
   }

  return stone_pose_refined;
}

visualization_msgs::Marker GraspObject::VisualizeMarker(const int marker_type,
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

void GraspObject::SetModel(const Object& object)
{
  object_ = object;
}

}
