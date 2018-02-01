/*
 * ObjectTFPublisher.cpp
 *
 *  Created on: May 18, 2016
 *      Author: Yves Zimmermann
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>

void poseCallback(const gazebo_msgs::ModelStates model_states)
{
  static tf::TransformBroadcaster br;
  int object_index = 0;

  for (int i = 0; i < model_states.name.size(); i++) {
    if (!(model_states.name[i] == "robot") && !(model_states.name[i] == "ground_plane")) {
      object_index = i;
      break;
    }
  }

  for (int i = object_index; i < model_states.name.size(); i++) {
    tf::Transform transform;
    transform.setOrigin(
        tf::Vector3(model_states.pose[i].position.x, model_states.pose[i].position.y,
                    model_states.pose[i].position.z));

    tf::Quaternion q;
    q.setX(model_states.pose[i].orientation.x);
    q.setY(model_states.pose[i].orientation.y);
    q.setZ(model_states.pose[i].orientation.z);
    q.setW(model_states.pose[i].orientation.w);
    transform.setRotation(q);
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world",
                             model_states.name[i] + "/base_link"));
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world",
                             model_states.name[i] + "/base_link_inertia"));
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_tf_broadcaster");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/gazebo/model_states", 100, &poseCallback);
  ros::spin();
  return 0;
}
;
