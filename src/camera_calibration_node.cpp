/*
 * serial_stacking_node.cpp
 *
 *  Created on: Apr 21, 2016
 *      Author: Martin Wermelinger
 */


// motion execution script
#include "grasp_and_place/CameraCalibration.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "stack_each");
  ros::NodeHandle nodeHandle("~");

  grasp_and_place::CameraCalibration camera_calibration(nodeHandle);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate loop_rate(1000);

  while (ros::ok()) {
     ros::spinOnce();
     loop_rate.sleep();
  }

  ros::waitForShutdown();
  return 0;
}

