/*
 * SerialStacking.cpp
 *
 *  Created on: Jul 26, 2017
 *      Author: Hironori Yoshida
 */

#include <grasp_and_place/CameraCalibration.hpp>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/conversions.h>


namespace grasp_and_place {

CameraCalibration::CameraCalibration(ros::NodeHandle& nh)
    : nh_(nh),
	  it_(nh),
      world_frame_("world")
{

  // Service server.
  execute_moves_ = nh_.advertiseService("execute_moves", &CameraCalibration::ExecuteMoves, this);

  // Set moveit interface
  move_group_.reset(new  moveit::planning_interface::MoveGroupInterface("manipulator"));
  move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
  move_group_->setPlannerId("RRTConnectkConfigDefault");
  move_group_->setPlanningTime(10);
  move_group_->setNumPlanningAttempts(10);
  rgb_image_sub_ = it_.subscribe("/sr300/depth/image_raw", 1, &CameraCalibration::SensorCallbackRGB, this);
//  depth_image_sub_ = it_.subscribe("/sr300/depth/image_raw", 1, &CameraCalibration::SensorCallbackRGB, this);

  // Set planning scene interface.
  planning_scene_.reset(new moveit::planning_interface::PlanningSceneInterface());
  count = 0;

}

CameraCalibration::~CameraCalibration()
{

}

bool CameraCalibration::ExecuteMoves(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {

	ROS_INFO("SerialStacking: ExecuteLocalization");

	bool localize_any_object = false;
	// move to pose defined by joint values
	std::vector<double> pose_1 = { -0.04388939216732979, 0.3110213577747345,
			-0.3654710352420807, -0.06915736198425293, -0.7540712356567383,
			3.193213701248169 };
	move_group_->setJointValueTarget(pose_1);
	robot_state::RobotState joint_value_target =
			move_group_->getJointValueTarget();
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = move_group_->plan(my_plan);
	success = move_group_->move();
	ros::Duration(1.0).sleep();

	MoveToPose(0.472, 0.171, 0.380, 0.636, 0.666, -0.295, 0.257);
	MoveToPose(0.390, 0.260, 0.272, 0.572, 0.594, -0.421, 0.377);
	MoveToPose(0.514, 0.259, 0.272, 0.411, 0.726, -0.538, 0.120);
	MoveToPose(0.369, -0.022, 0.380, 0.810, 0.577, -0.006, 0.108);
	MoveToPose(0.344, -0.063, 0.354, 0.799, 0.600, -0.012, 0.035);
	MoveToPose(0.428, -0.066, 0.449, 0.752, 0.648, -0.092, 0.079);
	MoveToPose(0.481, -0.059, 0.387, 0.732, 0.653, -0.194, 0.009);
	MoveToPose(0.502, -0.098, 0.387, 0.714, 0.674, -0.169, -0.090);
	MoveToPose(0.385, -0.002, 0.230, 	0.975, 0.223, -0.024, 0.006);
	MoveToPose(0.324, -0.111, 0.251, 	0.813, 0.575, 0.068, -0.069);
	MoveToPose(0.436, -0.151, 0.247, 	0.431, 0.881, 0.157, -0.114);
	MoveToPose(0.587, 0.018, 0.285, 	-0.032, 0.996, 0.049, -0.068);
	MoveToPose(0.539, 0.086, 0.236, 	-0.266, 0.943, -0.156, -0.127);
  MoveToPose(0.411, 0.125, 0.248,   -0.250, 0.954, -0.163, 0.028);
  MoveToPose(0.406, 0.044, 0.248,  0.043, 0.996, -0.002, 0.078);
	MoveToPose(0.379, 0.107, 0.249, 	0.785, -0.617, 0.048, 0.035);
  MoveToPose(0.473, 0.015, 0.317,   0.037, 0.999, 0.001, 0.002);
  MoveToPose(0.465, 0.179, 0.253,   -0.082, 0.980, -0.182, 0.005);
  MoveToPose(0.505, 0.034, 0.243,   0.606, 0.769, -0.192, -0.071);
  MoveToPose(0.416, -0.072, 0.248,   0.983, 0.079, -0.094, -0.133);

	return true;
}

bool CameraCalibration::MoveToPose(double px, double py, double pz,
		double rx, double ry, double rz, double rw)
{

  geometry_msgs::Pose target_pose;
  target_pose.position.x = px;
  target_pose.position.y = py;
  target_pose.position.z = pz;
  target_pose.orientation.x = rx;
  target_pose.orientation.y = ry;
  target_pose.orientation.z = rz;
  target_pose.orientation.w = rw;

  approaching_pose_publisher_.publish(target_pose);
  geometry_msgs::PoseStamped current_pose = move_group_->getCurrentPose();
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose.pose);
  waypoints.push_back(target_pose);

  robot_state::RobotState rs = *move_group_->getCurrentState();
  move_group_->setStartState(rs);

  // Plan trajectory.
  moveit_msgs::RobotTrajectory trajectory;
  moveit_msgs::MoveItErrorCodes error_code;
  double path_fraction = move_group_->computeCartesianPath(waypoints, 0.03, 0.0, trajectory, true, &error_code);
  if(path_fraction < 1.0) {
    ROS_WARN_STREAM("GraspObject: Could not calculate Cartesian Path.");
    return false;
  }

  robot_trajectory::RobotTrajectory robot_trajectory(rs.getRobotModel(), move_group_->getName());
  robot_trajectory.setRobotTrajectoryMsg(rs, trajectory);

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  iptp.computeTimeStamps(robot_trajectory, 1.0);
  robot_trajectory.getRobotTrajectoryMsg(trajectory);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
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
  ros::Duration(2.0).sleep();


	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(sensor_img_, sensor_msgs::image_encodings::BGR8);
		std::string filename = "/home/hyoshdia/Documents/calibration/image/cv/chessboard/" + std::to_string(count) + ".jpg";
		cv::imwrite(filename, cv_ptr->image);
		ROS_INFO("img saved!");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return false;
	}

  ros::Duration(1.0).sleep();
  count ++;

  return true;
}

void CameraCalibration::SensorCallbackRGB(const sensor_msgs::ImageConstPtr& msg) {
	sensor_img_ = *msg;
}

}
