
// motion execution script
#include "ur3_milling/MillingPath.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur3_milling");
  ros::NodeHandle nodeHandle("~");

  ur3_milling::MillingPath milling_path(nodeHandle);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::Rate loop_rate(1000);

  while (ros::ok()) {
     ros::spinOnce();
     loop_rate.sleep();
  }

  ros::waitForShutdown();

  return 0;
}
