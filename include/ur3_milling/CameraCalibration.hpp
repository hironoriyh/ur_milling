/*
 * SerialStacking.hpp
 *
 *  Created on: Nov, 15, 2017
 *      Author: Hironori Yoshida
 */

# pragma once

// ros
#include <ros/ros.h>
#include <ros/package.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
//#include <opencv-3.2.0-dev/opencv2/opencv.hpp>
#include <opencv-3.2.0-dev/opencv2/imgproc/imgproc.hpp>
#include <opencv-3.2.0-dev/opencv2/highgui/highgui.hpp>

// Eigen
#include <Eigen/Core>

// std
#include <string>
#include <memory>
#include <vector>

//ros msgs
#include <geometry_msgs/Point.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>

namespace grasp_and_place {

// typedefs.
typedef Eigen::Vector3d Vector3D;

class CameraCalibration {

public:
	CameraCalibration(ros::NodeHandle& nh);
	virtual ~CameraCalibration();

private:

	bool ExecuteMoves(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

	bool MoveToPose(const geometry_msgs::Pose& target_pose);
	bool MoveToPose(double px, double py, double pz,
					double rx, double ry, double rz, double rw);

	void SensorCallbackRGB(const sensor_msgs::ImageConstPtr& msg);

	ros::ServiceServer execute_moves_;
	ros::Publisher approaching_pose_publisher_;

	//! ROS Node handle.
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
  image_transport::Subscriber rgb_image_sub_;
  image_transport::Subscriber depth_image_sub_;

	//! Transform listener.
	tf::TransformListener tf_listener_;
	std::string camera_frame_;
	std::string world_frame_;

	//! MoveIt! move group interface.
	moveit::planning_interface::MoveGroupInterfacePtr move_group_;

	//! MoveIt! planning scene interface.
	std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;

	double velocity_scaling_ = 0.3;

	sensor_msgs::Image sensor_img_;

	int count;

};

} // grasp_and_place
