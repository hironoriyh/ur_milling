/*
 * Object.cpp
 *
 *  Created on: 23.06.2016
 *      Author: Martin Wermelinger
 */

#include <grasp_and_place/Object.hpp>

namespace grasp_and_place {

Object::Object(const std::string& name)
    : name_(name),
      pose_set_(false)
{

}

Object::Object()
    : pose_set_(false)
{
}

Object::~Object()
{
}

void Object::setName(const std::string& name)
{
  name_ = name;
}

std::string Object::getName()
{
  return name_;
}

void Object::setPose(const geometry_msgs::Pose pose)
{
  pose_ = pose;
  pose_set_ = true;
}

geometry_msgs::Pose Object::getPose()
{
  return pose_;
}

void Object::setPlacingPose(const geometry_msgs::Pose pose)
{
  placing_pose_ = pose;
}

geometry_msgs::Pose Object::getPlacingPose()
{
  return placing_pose_;
}

void Object::setRefinedGraspingPose(const geometry_msgs::Pose pose)
{
  refined_grasping_pose_ = pose;
}

geometry_msgs::Pose Object::getRefinedGraspingPose()
{
  return refined_grasping_pose_;
}

void Object::addGraspingPose(const geometry_msgs::Pose pose)
{
  grasping_poses_.push_back(pose);
}

void Object::clearGraspingPose()
{
  grasping_poses_.clear();
}

std::vector<geometry_msgs::Pose> Object::getGraspingPoses()
{
  return grasping_poses_;
}

void Object::setTargetGraspingPose(const geometry_msgs::Pose pose)
{
  target_grasping_pose_ = pose;
}

geometry_msgs::Pose Object::getTargetGraspingPose()
{
  return target_grasping_pose_;
}

bool Object::isPoseSet()
{
  return pose_set_;
}

}
