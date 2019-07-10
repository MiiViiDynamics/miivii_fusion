#include "ros/ros.h"
#include "fusion_detector.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fusion_detector");

  fusion_detector fd;
  ros::spin();

  return 0;
}



