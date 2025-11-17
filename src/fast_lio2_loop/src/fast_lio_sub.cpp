#include "fast_lio_sub.hpp"
#include <cmath>

FastLioPoll::FastLioPoll(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
{
  // Topics
  pnh_.param<std::string>("odom_topic",  odom_topic_,  std::string("/Odometry"));
  pnh_.param<std::string>("cloud_topic", cloud_topic_, std::string("/cloud_registered"));

  // Keyframe thresholds (translation used, rotation reserved for later)
  pnh_.param<double>("keyf_translation_thresh", keyf_translation_thresh_, 0.5); // metres
  pnh_.param<double>("keyf_rotation_thresh",    keyf_rotation_thresh_,    10.0); // degrees (unused for now)

  odom_sub_  = nh_.subscribe(odom_topic_, 100, &FastLioPoll::odomCb,  this);
  cloud_sub_ = nh_.subscribe(cloud_topic_, 10,  &FastLioPoll::cloudCb, this);

  ROS_INFO_STREAM("FastLioPoll subscribing to " << odom_topic_ << " and " << cloud_topic_);
  ROS_INFO_STREAM("Keyframe translation threshold: " << keyf_translation_thresh_ << " m");
}

// Odometry: update pose and drive keyframe logic
void FastLioPoll::odomCb(const nav_msgs::Odometry::ConstPtr& msg)
{
  latest_odom_ = *msg;
  latest_pose_ = latest_odom_.pose.pose;

  checkKeyFrameTrigger();
}

// Clouds: just keep the latest cloud
void FastLioPoll::cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  latest_cloud_ = *msg;
}

void FastLioPoll::checkKeyFrameTrigger()
{
  // 1) No cloud yet? Don't create keyframes.
  if (latest_cloud_.data.empty()) {
    return;
  }

  // 2) First keyframe ever.
  if (!has_last_keyf_) {
    last_keyf_pose_ = latest_pose_;
    has_last_keyf_  = true;
    createKeyFrame();
    ROS_INFO_STREAM("FIRST KEYFRAME CREATED");
    return;
  }

  // 3) Compute translation distance from last keyframe.
  const double x_now = latest_pose_.position.x;
  const double y_now = latest_pose_.position.y;
  const double z_now = latest_pose_.position.z;

  const double x_last = last_keyf_pose_.position.x;
  const double y_last = last_keyf_pose_.position.y;
  const double z_last = last_keyf_pose_.position.z;

  const double dx = x_now - x_last;
  const double dy = y_now - y_last;
  const double dz = z_now - z_last;

  const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

  if (dist >= keyf_translation_thresh_) {
    createKeyFrame();
    last_keyf_pose_ = latest_pose_;
    ROS_INFO_STREAM("KEYFRAME TRIGGER: dist = " << dist << " m (threshold "
                    << keyf_translation_thresh_ << " m)");
  }
}

void FastLioPoll::createKeyFrame()
{
  ++keyf_count_;
  ROS_INFO_STREAM("Keyframe " << keyf_count_
                  << " at pose ("
                  << latest_pose_.position.x << ", "
                  << latest_pose_.position.y << ", "
                  << latest_pose_.position.z << ")");
}
