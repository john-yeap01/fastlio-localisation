#pragma once 

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

class FastLioPoll {
public:
  FastLioPoll(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
  void odomCb(const nav_msgs::Odometry::ConstPtr& msg);
  void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg);

  void checkKeyFrameTrigger();
  void createKeyFrame();

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber odom_sub_, cloud_sub_;
  std::string odom_topic_, cloud_topic_;

  nav_msgs::Odometry       latest_odom_;
  sensor_msgs::PointCloud2 latest_cloud_;
  geometry_msgs::Pose      latest_pose_;

  bool               has_last_keyf_ = false;
  geometry_msgs::Pose last_keyf_pose_;

  double keyf_translation_thresh_;   // metres
  double keyf_rotation_thresh_;      // reserved for later, not used yet

  int keyf_count_ = 0;               // number of keyframes created
};
