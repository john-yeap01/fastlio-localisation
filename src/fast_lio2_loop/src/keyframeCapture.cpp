// Obtain pose and point cloud keyframes to store during flight
#include <ros/ros.h>
#include "fast_lio_sub.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fastlio_keyframe_tap");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  FastLioPoll node(nh, pnh);

  ros::spin();
  return 0;
}
