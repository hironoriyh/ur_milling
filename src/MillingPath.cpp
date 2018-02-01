/*
 * MillingPath.cpp
 *
 *  Created on: Jul 26, 2017
 *      Author: Hironori Yoshida
 */

#include <ur3_milling/MillingPath.hpp>

#include <ur3_milling/GraspObject.hpp>
#include <ur3_milling/PlaceObject.hpp>
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
#include "ur3_milling_msgs/HandCommand.h"

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
  refined_pose_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/object_mesh_refined", 1, true);
  move_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_pose", 1, true);
  localised_objects_visualizer_ = nh_.advertise<visualization_msgs::Marker>("/localised_objects", 1, true);
  target_config_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/target_poses", 1, true);

  // Service clients.
  check_validity_ = nh_.serviceClient<moveit_msgs::GetStateValidity>("/check_state_validity");

  // Service server.
  execute_stacking_ = nh_.advertiseService("execute_stacking", &MillingPath::ExecuteMillingPath, this);

  // Set moveit interface
  move_group_.reset(new moveit::planning_interface::MoveGroup("manipulator"));
  move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
  move_group_->setPlannerId("RRTConnectkConfigDefault");
  move_group_->setPlanningTime(10);
  move_group_->setNumPlanningAttempts(10);

  // Set planning scene interface.
  planning_scene_.reset(new moveit::planning_interface::PlanningSceneInterface());

  // Set gripper interface
  gripper_interface_.reset(new GripperInterface(nh_));

  // Load grasping poses.
  if (!LoadGraspingPoses()) {
    ROS_INFO("MillingPath: No grasping poses loaded.");
  }

}

MillingPath::~MillingPath()
{

}

bool MillingPath::ReadParameters()
{
  nh_.getParam("/serial_stacking/velocity_scaling", velocity_scaling_);
  nh_.getParam("/serial_stacking/camera_frame", camera_frame_);
  nh_.getParam("/serial_stacking/distance_to_objects", distance_to_objects_);
  nh_.getParam("/serial_stacking/allowed_position_deviation", position_diff_);
  nh_.getParam("/serial_stacking/allowed_rotation_deviation", rotation_diff_);

  nh_.getParam("/serial_stacking/min_localization_x", min_localization_.x());
  nh_.getParam("/serial_stacking/min_localization_y", min_localization_.y());
  nh_.getParam("/serial_stacking/min_localization_z", min_localization_.z());
  nh_.getParam("/serial_stacking/max_localization_x", max_localization_.x());
  nh_.getParam("/serial_stacking/max_localization_y", max_localization_.y());
  nh_.getParam("/serial_stacking/max_localization_z", max_localization_.z());

  return true;
}

bool MillingPath::LoadStackingConfiguration()
{
  desired_stacking_configuration_.clear();
  // Load stacking configuration.
  std::string path = ros::package::getPath("ur3_milling");
  path = path + "/config/stacking_configuration_serial.yaml";
  std::string commandString = "rosparam load " + path + " /serial_stacking";
  const char* command = commandString.c_str();
  if (system(command) != 0) {
    ROS_ERROR("Can't load parameter.");
    return false;
  }

  XmlRpc::XmlRpcValue stacking_configuration;
  if (!nh_.getParam("configuration", stacking_configuration) || stacking_configuration.size() == 0)
    return false;
  for (int i = 0; i < stacking_configuration.size(); ++i) {
    std::string name = static_cast<std::string>(stacking_configuration[i]["id"]);
    XmlRpc::XmlRpcValue pose = stacking_configuration[i]["pose"];
    geometry_msgs::Pose placing_pose;
    placing_pose.position.x = static_cast<double>(pose["position"]["x"]);
    placing_pose.position.y = static_cast<double>(pose["position"]["y"]);
    placing_pose.position.z = static_cast<double>(pose["position"]["z"]);
    placing_pose.orientation.x = static_cast<double>(pose["orientation"]["x"]);
    placing_pose.orientation.y = static_cast<double>(pose["orientation"]["y"]);
    placing_pose.orientation.z = static_cast<double>(pose["orientation"]["z"]);
    placing_pose.orientation.w = static_cast<double>(pose["orientation"]["w"]);
    Object object(name);
    object.setPlacingPose(placing_pose);
    desired_stacking_configuration_.push_back(object);
  }
  return true;
}

bool MillingPath::LoadGraspingPoses()
{
  XmlRpc::XmlRpcValue grasping_poses;
  if (!nh_.getParam("grasping_poses", grasping_poses) || grasping_poses.size() == 0)
    return false;
  for (int i = 0; i < grasping_poses.size(); ++i) {
    // Get grasping poses.
    geometry_msgs::Pose grasping_pose;
    std::string id = static_cast<std::string>(grasping_poses[i]["id"]);
    int grasp_pose_index = static_cast<int>(grasping_poses[i]["index"]);
    XmlRpc::XmlRpcValue pose = grasping_poses[i]["pose"];
    grasping_pose.position.x = static_cast<double>(pose["position"]["x"]);
    grasping_pose.position.y = static_cast<double>(pose["position"]["y"]);
    grasping_pose.position.z = static_cast<double>(pose["position"]["z"]);
    grasping_pose.orientation.x = static_cast<double>(pose["orientation"]["x"]);
    grasping_pose.orientation.y = static_cast<double>(pose["orientation"]["y"]);
    grasping_pose.orientation.z = static_cast<double>(pose["orientation"]["z"]);
    grasping_pose.orientation.w = static_cast<double>(pose["orientation"]["w"]);

    std::vector<double> finger_positions; // =grasping_poses[i]["finger_pos"][0];
    for(int j=0; j < grasping_poses[i]["finger_pos"].size(); j++) {
      finger_positions.push_back(static_cast<double>(grasping_poses[i]["finger_pos"][j]));
    }

    GraspingConfigruation grasping_conf;
    grasping_conf.pose = grasping_pose;
    grasping_conf.finger_positions = finger_positions;
    grasping_conf.grasp_pose_index = grasp_pose_index;

    std::map<std::string, std::vector<GraspingConfigruation>>::iterator it;
    it = grasping_poses_.find(id);
    if (it != grasping_poses_.end()) {
      it->second.push_back(grasping_conf);
    } else {
      std::vector<GraspingConfigruation> vector;
      vector.push_back(grasping_conf);
      grasping_poses_.insert(std::pair<std::string, std::vector<GraspingConfigruation>>(id, vector));
    }
  }
  return true;
}

bool MillingPath::LoadCollisionEnvironment()
{
  // Add object to the planning scene.
  ROS_INFO_STREAM("MillingPath: Load environment collision objects.");

  // Load box primitives.
  XmlRpc::XmlRpcValue collision_boxes;
  if (!nh_.getParam("collision_boxes", collision_boxes) || collision_boxes.size() == 0) return false;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  for (int i = 0; i < collision_boxes.size(); ++i) {
    moveit_msgs::CollisionObject box;
    box.header.frame_id = world_frame_;
    box.header.stamp = ros::Time::now();
    box.operation = moveit_msgs::CollisionObject::ADD;
    // Get collision box.
    box.id = static_cast<std::string>(collision_boxes[i]["id"]);

    geometry_msgs::Pose box_pose;
    XmlRpc::XmlRpcValue pose = collision_boxes[i]["pose"];
    box_pose.position.x = static_cast<double>(pose["position"]["x"]);
    box_pose.position.y = static_cast<double>(pose["position"]["y"]);
    box_pose.position.z = static_cast<double>(pose["position"]["z"]);
    box_pose.orientation.x = static_cast<double>(pose["orientation"]["x"]);
    box_pose.orientation.y = static_cast<double>(pose["orientation"]["y"]);
    box_pose.orientation.z = static_cast<double>(pose["orientation"]["z"]);
    box_pose.orientation.w = static_cast<double>(pose["orientation"]["w"]);

    shape_msgs::SolidPrimitive box_primitive;
    box_primitive.type = box_primitive.BOX;
    box_primitive.dimensions.resize(3);
    box_primitive.dimensions[0] = static_cast<double>(collision_boxes[i]["dimension"]["x"]);
    box_primitive.dimensions[1] = static_cast<double>(collision_boxes[i]["dimension"]["y"]);
    box_primitive.dimensions[2] = static_cast<double>(collision_boxes[i]["dimension"]["z"]);
    box.primitives.push_back(box_primitive);
    box.primitive_poses.push_back(box_pose);
    collision_objects.push_back(box);
  }

  planning_scene_->applyCollisionObjects(collision_objects);
  planning_scene_->addCollisionObjects(collision_objects);
  return true;
}

bool MillingPath::ExecuteMillingPath(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{

  ROS_INFO("MillingPath: ExecuteMillingPath");
  stacked_objects_.clear();
  move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
  mesh_.markers.clear();

  // Check if stacking configuration exists.
  if (!LoadStackingConfiguration()) {
    ROS_INFO("MillingPath: ExecuteMillingPath: No stacking configuration loaded.");
    return true;
  }

  // Load the needed models in the vector of the undetected models
  std::vector<Object> models_to_detect = desired_stacking_configuration_;
  // detach objects
  for (int i = 0; i < desired_stacking_configuration_.size(); ++i) {
    move_group_->detachObject(desired_stacking_configuration_[i].getName());
  }

  // visulaize target poses
  visualization_msgs::MarkerArray target_poses;
  for (int i = 0; i < desired_stacking_configuration_.size(); ++i) {
    visualization_msgs::Marker target_marker = VisualizeMarker(10, desired_stacking_configuration_[i].getPlacingPose(), i, 0.5, 0, 0, 0.8);
      std::string object_name = desired_stacking_configuration_[i].getName();
      std::string model_path = "package://urdf_models/models/"  + object_name + "/mesh/" + object_name + ".stl";
      target_marker.mesh_resource = model_path;
      target_poses.markers.push_back(target_marker);
  }
  target_config_publisher_.publish(target_poses);

  if (!ExecuteLocalization(desired_stacking_configuration_)) {
    ROS_WARN("Could not localize all objects.");
    return false;
  }

  // stack the objects
  for (int i = 0; i < desired_stacking_configuration_.size(); ++i) {
    ROS_INFO_STREAM("MillingPath: Stack object " << desired_stacking_configuration_[i].getName());

    if (!PickAndPlaceObject(i)) {
      ROS_WARN_STREAM("Could not stack objects " << desired_stacking_configuration_[i].getName());
      return false;
    }
  }
  return true;
}

bool MillingPath::LocalizeObjects()
{
  ROS_INFO_STREAM("MillingPath: LocaliseObjects.");
  model_locations_.clear();

  ros::ServiceClient client = nh_.serviceClient<object_detection::LocaliseObjects>("/object_localisation");
  object_detection::LocaliseObjects srv;

  srv.request.min_object_length = 0;

  std::cout << "Calling object_localisation service." << std::endl;

  if (!client.waitForExistence(ros::Duration(2.0))) {
    return false;
  }

  if (client.call(srv)) {
    ROS_INFO("Object localisation executed.");
    for (int j = 0; j < srv.response.detected_model_poses.size(); j++) {

      const ros::Time time = ros::Time::now();
      geometry_msgs::PoseStamped model_camera_pose;
      geometry_msgs::PoseStamped model_pose;
      model_camera_pose.header.frame_id = camera_frame_;
      model_camera_pose.header.stamp = time;
      model_camera_pose.pose = srv.response.detected_model_poses[j];
      model_pose.header.frame_id = world_frame_;
      model_pose.header.stamp = time;

      try {
        const ros::Time time = ros::Time::now();
        const ros::Duration timeout(1);
        const ros::Duration polling_sleep_duration(4);
        std::string* error_msg = NULL;

        tf_listener_.waitForTransform(world_frame_, camera_frame_, time, timeout, polling_sleep_duration, error_msg);
        tf_listener_.transformPose(world_frame_, model_camera_pose, model_pose);
      } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }

      if( model_pose.pose.position.z < min_localization_.z() || model_pose.pose.position.z > max_localization_.z()||
          model_pose.pose.position.x < min_localization_.x() || model_pose.pose.position.x > max_localization_.x() ||
          model_pose.pose.position.y < min_localization_.y() || model_pose.pose.position.y > max_localization_.y()) {
        ROS_WARN_STREAM("localization skipped the object" << model_pose.pose.position);
        continue;
      }

      model_pose.pose.position.x -= 0.02;
      model_pose.pose.position.z = move_group_->getCurrentPose().pose.position.z;
      model_pose.pose.orientation = move_group_->getCurrentPose().pose.orientation;

      // Publish pose.to
      Vector3D scale(0.05, 0.05, 0.03);
      visualization_msgs::Marker localised_object_pose = VisualizeMarker(
          visualization_msgs::Marker::CYLINDER, model_pose.pose, j, 0, 1, 0, 1, scale);
      localised_objects_visualizer_.publish(localised_object_pose);

      bool check_dist = false;
      for (int i = 0; i < desired_stacking_configuration_.size(); i++) {
        if(desired_stacking_configuration_[i].isPoseSet()){
        	Vector3D diff(desired_stacking_configuration_[i].getPose().position.x - model_pose.pose.position.x,
        			desired_stacking_configuration_[i].getPose().position.y - model_pose.pose.position.y,
					0);
        	if(diff.norm() < 0.05) {
        		check_dist = true;
        		ROS_WARN("localization point was skipped!");
        		break;
        	}
        }
      }

      if(!check_dist) model_locations_.push_back(model_pose);

    }
  } else {
    ROS_WARN("MillingPath: Could not call client.");
    return false;
  }

  return true;
}

bool MillingPath::ExecuteLocalization(std::vector<Object> models_to_localize)
{
  ROS_INFO("MillingPath: ExecuteLocalization");

  // move to pose defined by joint values
  move_group_->setMaxVelocityScalingFactor(0.3);
  geometry_msgs::PoseStamped localize_pose = move_group_->getCurrentPose();
  localize_pose.pose.position.x = 0.41;
  localize_pose.pose.position.y = 0.0;
  localize_pose.pose.position.z = 0.41;
  localize_pose.pose.orientation.x = -0.02816;
  localize_pose.pose.orientation.y = 1.0;
  localize_pose.pose.orientation.z = 0.0;
  localize_pose.pose.orientation.w = 0.06;
  // move to pose defined by joint values
  MoveToPose(localize_pose);

  gripper_interface_->ClientHandCommand("open2");
  ros::Duration(1.0).sleep();

  DetectObject(models_to_localize);

  if (!LocalizeObjects()) {
    ROS_WARN_STREAM("Could not call service LocaliseObjects.");
    return false;
  }

  geometry_msgs::PoseStamped inter_mediate_pose = move_group_->getCurrentPose();
  inter_mediate_pose.pose.orientation.y -= 0.2;
  inter_mediate_pose.pose.position.z -= 0.05;
  if (!MoveToPose(inter_mediate_pose)) {
    ROS_WARN("couldn't move");
  }

  DetectObject(models_to_localize);

  // Detect the models at the localised object locations
  while (models_to_localize.size() > 0) {
    for (int i = 0; i < model_locations_.size(); i++) {
      if (models_to_localize.size() > 0) {
        geometry_msgs::PoseStamped detection_pose = model_locations_[i];

        bool check_dist = false;
        for (int j = 0; j < desired_stacking_configuration_.size(); j++) {
//					ROS_INFO_STREAM("is pose set? " << desired_stacking_configuration_[j].isPoseSet());
          if (desired_stacking_configuration_[j].isPoseSet()) {
            Vector3D diff(
                desired_stacking_configuration_[j].getPose().position.x
                    - model_locations_[i].pose.position.x,
                desired_stacking_configuration_[j].getPose().position.y
                    - model_locations_[i].pose.position.y,
                0);
            if (diff.norm() < 0.03) {
              check_dist = true;
              ROS_INFO_STREAM("diff \n" << diff.norm());
              ROS_WARN("localization point was skipped!");
              continue;
            }
          }
        }

        if (check_dist) {
          ROS_WARN("localization point was skipped!");
          continue;
        }

        detection_pose.pose.position.z += distance_to_objects_;
        ROS_INFO_STREAM("move to \n" << detection_pose.pose.position);
        if (!MoveToPose(detection_pose)) {
          ROS_WARN("couldn't move");
          continue;
        }
        ros::Duration(1.0).sleep();
        ROS_INFO_STREAM("Detecting objects.");
        bool detect_success = DetectObject(models_to_localize);
        ROS_WARN_STREAM(" detect success: " << detect_success);
        if (!detect_success) {
          Eigen::Isometry3d current_pose_eigen;
          tf::poseMsgToEigen(detection_pose.pose, current_pose_eigen);
          Eigen::Quaterniond rot_quat(0.985, 0, 0, 0.174);  //w x y z
          current_pose_eigen *= rot_quat;
          tf::poseEigenToMsg(current_pose_eigen, detection_pose.pose);
          detection_pose.pose.position.z -= 0.05;
          MoveToPose(detection_pose);
          ros::Duration(1.0).sleep();
          DetectObject(models_to_localize);
          ros::Duration(1.0).sleep();
          detection_pose.pose.position.z += 0.05;
          MoveToPose(detection_pose);
        }

      }
    }
  }

  ROS_INFO("detected poses");
  for (int j = 0; j < desired_stacking_configuration_.size(); j++) {
    std::cout << desired_stacking_configuration_[j].getName() << "\n"
              << desired_stacking_configuration_[j].getPose() << std::endl;
  }

  return true;
}

bool MillingPath::PickAndPlaceObject(const int object_index)
{
  geometry_msgs::Pose object_pose = desired_stacking_configuration_[object_index].getPose(); // detected object pose in the world frame
  const std::string object_name = desired_stacking_configuration_[object_index].getName();
  ROS_INFO_STREAM("MillingPath: PickAndPlaceObject " << object_name);

  // TODO implement grasp pose selection using grasp pose detection library.
  GraspingConfigruation grasping_config;
  std::string actual_id = object_name + "_actual";
  std::map<std::string, std::vector<GraspingConfigruation>>::iterator map_it;
  map_it = grasping_poses_.find(actual_id);
  if (map_it == grasping_poses_.end()) {
    grasping_config.pose = object_pose;
  } else {
    std::vector<GraspingConfigruation> grasp_poses = map_it->second;
    Eigen::Isometry3d current_pose_eigen;
    tf::poseMsgToEigen(move_group_->getCurrentPose().pose, current_pose_eigen);
    Eigen::Quaterniond quat_current(current_pose_eigen.rotation());

    for(std::vector<GraspingConfigruation>::iterator it = grasp_poses.begin(); it != grasp_poses.end(); ++it) {
      geometry_msgs::Pose grasp_pose_original = it->pose;
      desired_stacking_configuration_[object_index].setTargetGraspingPose(grasp_pose_original);
      geometry_msgs::Pose grasp_pose_transformed;

      Eigen::Isometry3d object_pose_eigen, grasp_pose_original_eigen, grasp_pose_transformed_eigen;
      tf::poseMsgToEigen(object_pose, object_pose_eigen);
      tf::poseMsgToEigen(grasp_pose_original, grasp_pose_original_eigen);
      grasp_pose_transformed_eigen = object_pose_eigen * grasp_pose_original_eigen;
      tf::poseEigenToMsg(grasp_pose_transformed_eigen, grasp_pose_transformed);

      // evaluate the effort to the grasping pose
      if(grasp_pose_transformed_eigen.linear().col(2).z() > 0 ) {
        ROS_WARN("SerialStack: grasp pose z component is positive. skipping this pose");
        continue;
      }
      Eigen::Quaterniond quat_grasp(grasp_pose_transformed_eigen.rotation());
      double ang_dist = quat_grasp.angularDistance(quat_current);
      ROS_WARN_STREAM("grasp index: " << it->grasp_pose_index << " ,Ang dist:" << ang_dist);

      // add info to grasp_config
      if (CheckCollision(grasp_pose_transformed)) {
        grasping_config.object_index = actual_id;
        grasping_config.pose = grasp_pose_transformed;
        grasping_config.finger_positions = it->finger_positions;
        grasping_config.grasp_pose_index = it->grasp_pose_index;
        ROS_WARN_STREAM("PickAndPlace: grasp_pose_index: " << it->grasp_pose_index);
        break;
      } else {
        ROS_WARN_STREAM("MillingPath: Transformed grasping pose is not valid. \n" << grasp_pose_transformed);
      }
    }
  }

  desired_stacking_configuration_[object_index].addGraspingPose(grasping_config.pose);
//  ROS_INFO_STREAM("MillingPath: grasping_config \n" << grasping_config.pose);
  GraspObject grasp_task(nh_, gripper_interface_, move_group_, object_pose, grasping_config);
  grasp_task.SetModel(desired_stacking_configuration_[object_index]);

  if (!grasp_task.ExecuteGrasping()) {
    return false;
  }

  // update model position within gripper.
  if (PoseRefinement(object_index)) {
    ROS_INFO_STREAM("MillingPath: Pose refinement successful.");
  } else {
//    return false;
    ROS_WARN_STREAM("MillingPath: Pose refinement not successful. Assume no displacement during grasping.");
  }

  // Place object.
  ROS_INFO_STREAM("MillingPath: Start placing object.");
  geometry_msgs::Pose target_pose = GetTargetTCPPose(object_index);
  PlaceObject place_task(nh_, gripper_interface_, move_group_, target_pose);
  place_task.SetModel(desired_stacking_configuration_[object_index]);

  if (!place_task.ExecutePlacing()) {
    ROS_INFO_STREAM("MillingPath: PickAndPlaceObject: failed placing object.");
    return false;
  }

  // Update the pose of the placed object
  bool object_pose_updated = EvaluatePlacedPose(object_index);

  // move to intermediate pose
  ROS_INFO_STREAM("MillingPath: move to intermediate_pose: " );
  geometry_msgs::PoseStamped intermediate_pose = move_group_->getCurrentPose();
  intermediate_pose.pose.position.z += 0.1;
  if (!MoveToPose(intermediate_pose)) {
    return false;
  }
  move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
  ROS_INFO_STREAM("MillingPath: move to localization pose: " );
  bool localize_any_object = false;
  std::vector<double> pose_1 = {-0.049119651317596436, 0.375019371509552, -0.5441534519195557, -0.004059905651956797, -0.6474179625511169, 3.120441198348999};
  move_group_->setJointValueTarget(pose_1);
  robot_state::RobotState joint_value_target = move_group_->getJointValueTarget();
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = move_group_->plan(my_plan);
  success = move_group_->move();

  bool stacking_successful = true;
  return object_pose_updated && stacking_successful;
}

bool MillingPath::EvaluatePlacedPose(const int object_index){
  ROS_INFO_STREAM("EvaluatePlacedPose: Update pose of " << desired_stacking_configuration_[object_index].getName());
  gripper_interface_->ClientHandCommand("open2");

  bool object_pose_updated = DetectObjectRotate(object_index);
  if (object_pose_updated) {
    // Check if stacking was successful.
    double diff_x, diff_y;
    diff_x = desired_stacking_configuration_[object_index].getPose().position.x -
        desired_stacking_configuration_[object_index].getPlacingPose().position.x;
    diff_y = desired_stacking_configuration_[object_index].getPose().position.y -
        desired_stacking_configuration_[object_index].getPlacingPose().position.y;
    // compute next pose based on the detected pose
    if (object_index < desired_stacking_configuration_.size()) {
      int next_object_index = object_index + 1;
      if (std::abs(diff_x) > position_diff_ || std::abs(diff_y) > position_diff_) {
        ROS_WARN_STREAM("Desired pose diff : " << diff_x << " , " << diff_y);
        geometry_msgs::Pose next_placing_pose = desired_stacking_configuration_[next_object_index].getPlacingPose();
        next_placing_pose.position.x += diff_x;
        next_placing_pose.position.y += diff_y;
        desired_stacking_configuration_[next_object_index].setPlacingPose(next_placing_pose);
      }
    }

    // update the pose in visualizer
    visualization_msgs::MarkerArray target_poses;
    for (int i = 0; i < desired_stacking_configuration_.size(); ++i) {
      geometry_msgs::Pose target_pose_;
      target_pose_ = desired_stacking_configuration_[i].getPlacingPose();
      visualization_msgs::Marker target_marker = VisualizeMarker(10, target_pose_, i, 0.5, 0, 0, 0.8);
      std::string object_name = desired_stacking_configuration_[i].getName();
      std::string model_path = "package://urdf_models/models/" + object_name + "/mesh/" + object_name + ".stl";
      target_marker.mesh_resource = model_path;
      target_poses.markers.push_back(target_marker);
    }
    target_config_publisher_.publish(target_poses);
    desired_stacking_configuration_[object_index].setPose(desired_stacking_configuration_[object_index].getPlacingPose());
    // Visualize object mesh.
//    mesh_.markers[object_index].pose = desired_stacking_configuration_[object_index].getPose();
//    mesh_.markers[object_index].action = visualization_msgs::Marker::ADD;
//    mesh_publisher_.publish(mesh_);
  } else {
    geometry_msgs::PoseStamped pose_down = move_group_->getCurrentPose();
    pose_down.pose.position.z -= 0.02;
    pose_down.pose.orientation.y += 0.4;
    MoveToPose(pose_down);
    object_pose_updated = DetectObjectRotate(object_index);
  }

  stacked_objects_.push_back(desired_stacking_configuration_[object_index]);
  return true;
}

bool MillingPath::DetectObjectRotate(const int object_index)
{
  bool object_pose_updated = false;
  int iter = 0;

  geometry_msgs::PoseStamped current_pose = move_group_->getCurrentPose();
  std::vector<Object> object_to_update;
  object_to_update.push_back(desired_stacking_configuration_[object_index]);

  while (!object_pose_updated && iter <= 2) {
    if (DetectObject(object_to_update) && object_to_update.empty()) {
      object_pose_updated = true;
      break;
    }

    //Rotate view to get an other perspective at next try
    ros::Duration(0.5).sleep();
    Eigen::Isometry3d current_pose_eigen;
    tf::poseMsgToEigen(current_pose.pose, current_pose_eigen);
    Eigen::Quaterniond rot_quat(0.996,0, 0, -0.087 ); //w x y z
    current_pose_eigen *= rot_quat;
    tf::poseEigenToMsg(current_pose_eigen, current_pose.pose);
//    current_pose.pose.position.x -= 0.03;
    MoveToPose(current_pose);
    iter++;
  }


  if(!object_pose_updated) {
    desired_stacking_configuration_[object_index].setPose(desired_stacking_configuration_[object_index].getPlacingPose());
    object_pose_updated = true;
  }

  return  object_pose_updated;
}

bool MillingPath::DetectObject(std::vector<Object>& models_to_detect)
{
  ROS_INFO("MillingPath: DetectObject");
  ros::ServiceClient client = nh_.serviceClient<object_detection::DetectObject>("/object_detection");
  object_detection::DetectObject srv; // TODO: Add string to msg.
  for (int i = 0; i < models_to_detect.size(); i++) {
    std_msgs::String model_id;
    model_id.data = models_to_detect[i].getName();
//    ROS_INFO_STREAM("model name check" << model_id.data);
    srv.request.models_to_detect.push_back(model_id);
  }

  if (!client.waitForExistence(ros::Duration(2.0))) {
    return false;
  }

  if (client.call(srv)) {
    ROS_INFO("MillingPath::Object detection executed.");
    if(srv.response.model_ids.size() == 0) return false;

    for (int j = 0; j < srv.response.model_ids.size(); j++) {
      std::string id = srv.response.model_ids[j].data;
      const ros::Time time = ros::Time::now();
      geometry_msgs::PoseStamped model_camera_pose;
      geometry_msgs::PoseStamped model_pose;
      model_camera_pose.header.frame_id = camera_frame_;
      model_camera_pose.header.stamp = time;
      model_camera_pose.pose = srv.response.detected_model_poses[j];
      // model_camera_pose.pose.position.x -= 0.03;
      model_pose.header.frame_id = world_frame_;
      model_pose.header.stamp = time;

      try {
        const ros::Time time = ros::Time::now();
        const ros::Duration timeout(1);
        const ros::Duration polling_sleep_duration(4);
        std::string* error_msg = NULL;
        tf_listener_.waitForTransform(world_frame_, camera_frame_, time, timeout, polling_sleep_duration, error_msg);
        tf_listener_.transformPose(world_frame_, model_camera_pose, model_pose);
      } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }

      // Visualize object mesh.
//            ROS_INFO_STREAM("MillingPath::DetectObject \n "   << model_pose.pose);
      visualization_msgs::Marker object_mesh_marker = VisualizeMarker(
          visualization_msgs::Marker::MESH_RESOURCE, model_pose.pose, j, .5, .5, .5, .8);
      std::string model_path = "package://urdf_models/models/"  + id + "/mesh/" + id + ".stl";
      object_mesh_marker.mesh_resource = model_path;
      object_mesh_marker.ns = id;
      mesh_.markers.push_back(object_mesh_marker);
      mesh_publisher_.publish(mesh_);

      // saving detected models to the world
      for (int i = 0; i < desired_stacking_configuration_.size(); i++) {
        if (desired_stacking_configuration_[i].getName() == id) {
          desired_stacking_configuration_[i].setPose(model_pose.pose);
          AddCollisionObject(id, i);
        }
      }

      std::vector<Object> new_models_to_detect;
      for (int i = 0; i < models_to_detect.size(); i++) {
        if (!(models_to_detect[i].getName() == srv.response.model_ids[j].data))
          new_models_to_detect.push_back(models_to_detect[i]);
          ROS_WARN_STREAM("new model to detect " << models_to_detect[i].getName());
      }

      models_to_detect = new_models_to_detect;
    }
  } else {
    ROS_WARN("MillingPath: Could not call client.");
  }
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

bool MillingPath::PoseRefinement(const int stack_model_index)
{
  ROS_INFO("MillingPath: RefinePose");
  ros::ServiceClient client = nh_.serviceClient<object_detection::DetectObject>("/object_pose_refinement");
  object_detection::DetectObject srv;

  std_msgs::String model_id;
  model_id.data = desired_stacking_configuration_[stack_model_index].getName();
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

  const ros::Time time = ros::Time::now();
  geometry_msgs::PoseStamped model_tf, object_pose;
  // Model Pose in TCP frame.
  model_tf.header.frame_id = "tool0";
  model_tf.header.stamp = time;
  // Model Pose in world frame.
  object_pose.header.frame_id = world_frame_;
  object_pose.header.stamp = time;

  if (stone_pose_refined) {
    ROS_INFO("MillingPath: Pose refinement executed.");

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

      // check if the pose is too far off from already detecte pose
      Eigen::Isometry3d refined_pose_eigen, original_pose_eigen, diff_pose_eigen;
      tf::poseMsgToEigen(desired_stacking_configuration_[stack_model_index].getPose(), original_pose_eigen);
      tf::poseMsgToEigen(object_pose.pose, refined_pose_eigen);
      double diff_z = std::abs(original_pose_eigen.rotation().eulerAngles(0,1,2)[2] -
                               refined_pose_eigen.rotation().eulerAngles(0,1,2)[2]);
      if(diff_z > 0.1) {
        ROS_WARN_STREAM("GraspObject: diff world pose: " << diff_z);
        goto fail;
      }


    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    // save transfer function between stone and end-effector
    geometry_msgs::Pose refined_grasping_pose;
    tf::Transform tcp_to_model;
    tf::poseMsgToTF(model_tf.pose, tcp_to_model);
    tf::poseTFToMsg(tcp_to_model.inverse(), refined_grasping_pose);

    // Set updated model position.
    desired_stacking_configuration_[stack_model_index].setPose(object_pose.pose);
    // publish the pose
    desired_stacking_configuration_[stack_model_index].setRefinedGraspingPose(refined_grasping_pose);
    // Visualize object mesh.
    visualization_msgs::Marker target_marker = VisualizeMarker(visualization_msgs::Marker::MESH_RESOURCE, object_pose.pose, stack_model_index, 0, 1, 0, 0.3);
    std::string target_pose_name = desired_stacking_configuration_[stack_model_index].getName();
    std::string model_path =  "package://urdf_models/models/"  + target_pose_name + "/mesh/" + target_pose_name + ".stl";
    target_marker.mesh_resource = model_path;
    refined_mesh_.markers.clear();
    refined_mesh_.markers.push_back(target_marker);
    refined_pose_publisher_.publish(refined_mesh_);

  } else {
fail:
    ROS_WARN("MillingPath: Refinement not successful.");
    // Assume grasp without displacement.
    geometry_msgs::PoseStamped grasping_pose, object_pose_old;
    grasping_pose.header.stamp = time;
    grasping_pose.header.frame_id = world_frame_;
    object_pose_old.header.stamp = time;
    object_pose_old.header.frame_id = world_frame_;
    grasping_pose.pose = desired_stacking_configuration_[stack_model_index].getGraspingPoses().at(0);
    object_pose_old.pose = desired_stacking_configuration_[stack_model_index].getPose();
    tf::Transform world_to_tcp;
    tf::Transform world_to_model;
    tf::poseMsgToTF(grasping_pose.pose, world_to_tcp);
    tf::poseMsgToTF(object_pose_old.pose, world_to_model);
    tf::Transform tcp_to_model = world_to_tcp.inverse() * world_to_model;
    tf::poseTFToMsg(tcp_to_model, model_tf.pose);

    tf_listener_.waitForTransform(world_frame_, "tool0", time, ros::Duration(1.0));
    tf_listener_.transformPose(world_frame_, model_tf, object_pose);
    desired_stacking_configuration_[stack_model_index].setPose(object_pose.pose);

    // Set refined grasping pose to desired grasping pose.
    geometry_msgs::Pose refined_grasping_pose;
    tf::poseTFToMsg(tcp_to_model.inverse(), refined_grasping_pose);
    desired_stacking_configuration_[stack_model_index].setRefinedGraspingPose(refined_grasping_pose);
  }



  return stone_pose_refined;
}

bool MillingPath::CheckCollision(const geometry_msgs::Pose pose)
{
  ROS_INFO_STREAM("MillingPath: Check collision.");
  move_group_->setJointValueTarget(pose);
  robot_state::RobotState grasping_state = move_group_->getJointValueTarget();
  moveit_msgs::RobotState grasping_state_msg;
  robot_state::robotStateToRobotStateMsg(grasping_state, grasping_state_msg);
  moveit_msgs::GetStateValidity get_state_validity;
  get_state_validity.request.group_name = move_group_->getName();
  get_state_validity.request.robot_state = grasping_state_msg;

  if (!check_validity_.waitForExistence(ros::Duration(1.0))) {
    ROS_ERROR_STREAM("CheckCollision: Check validity service not advertised.");
    return false;
  }
  if (!check_validity_.call(get_state_validity)) {
    ROS_ERROR_STREAM("CheckCollision: Could not call check validity service.");
    return false;
  }
//  ROS_INFO_STREAM("state validity: " << get_state_validity.response);
//  if(!get_state_validity.response.valid){
//  ROS_INFO("CheckCollision not valid");
//  ROS_INFO_STREAM("state validity: " << get_state_validity.response.constraint_result.size());
//    for(int i =0; i< get_state_validity.response.constraint_result.size() ; i++){
//      ROS_INFO_STREAM("state validity: " << get_state_validity.response.constraint_result.at(i) );
//    }
//  }

  return get_state_validity.response.valid;
}

geometry_msgs::Pose MillingPath::GetTargetTCPPose(const int stack_model_index)
{
  tf::Transform object_target_pose;
  tf::poseMsgToTF(desired_stacking_configuration_[stack_model_index].getPlacingPose(), object_target_pose);

  tf::Transform object_to_tcp;
  tf::poseMsgToTF(desired_stacking_configuration_[stack_model_index].getRefinedGraspingPose(), object_to_tcp);

  tf::Transform tcp_target_tf = object_target_pose * object_to_tcp;
  geometry_msgs::Pose tcp_target_pose;
  tf::poseTFToMsg(tcp_target_tf, tcp_target_pose);

  return tcp_target_pose;
}

bool MillingPath::AddCollisionObject(const std::string object_name, const int object_index)
{
  // Create collision object.
  moveit_msgs::CollisionObject object;
  object.header.frame_id = world_frame_;
  object.header.stamp = ros::Time::now();
  object.id = object_name;
  object.mesh_poses.push_back(desired_stacking_configuration_[object_index].getPose());
  // Get known collision objects.
  std::vector<std::string> collision_object_ids = planning_scene_->getKnownObjectNames();
  std::vector<std::string>::iterator it;

  it = std::find(collision_object_ids.begin(), collision_object_ids.end(), object_name);
  if (it != collision_object_ids.end()) {
    ROS_INFO_STREAM("MillingPath: AddCollisionObject: Move known object.");
    object.operation = moveit_msgs::CollisionObject::MOVE;
  } else {
    ROS_INFO_STREAM("MillingPath: AddCollisionObject: Add new object.");
    // Get mesh.
    shapes::Mesh* shape = shapes::createMeshFromResource( "package://urdf_models/models/"  + object_name + "/mesh/" + object_name + ".stl");
    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg = mesh;
    shapes::constructMsgFromShape(shape,mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    object.operation = moveit_msgs::CollisionObject::ADD;
    object.meshes.push_back(mesh);
  }
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(object);
  planning_scene_->addCollisionObjects(collision_objects);
  return true;
}


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
