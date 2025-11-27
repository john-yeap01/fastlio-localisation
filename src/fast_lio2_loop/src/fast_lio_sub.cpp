#include "fast_lio_sub.hpp"

#include <cmath>

FastLioPoll::FastLioPoll(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
{
  // Load parameters (with reasonable defaults)
  pnh_.param<std::string>("odom_topic",  odom_topic_,  std::string("/Odometry"));
  pnh_.param<std::string>("cloud_topic", cloud_topic_, std::string("/cloud_registered"));
  pnh_.param<double>("keyf_translation_thresh", keyf_translation_thresh_, 0.5); // 0.5 m
  pnh_.param<double>("keyf_rotation_thresh",    keyf_rotation_thresh_,    15.0); // degrees, not used yet

  ROS_INFO_STREAM("[FastLioPoll] Subscribing to odom:  " << odom_topic_);
  ROS_INFO_STREAM("[FastLioPoll] Subscribing to cloud: " << cloud_topic_);
  ROS_INFO_STREAM("[FastLioPoll] Keyframe translation threshold: "
                  << keyf_translation_thresh_ << " m");

  odom_sub_  = nh_.subscribe(odom_topic_,  50, &FastLioPoll::odomCb,  this);
  cloud_sub_ = nh_.subscribe(cloud_topic_, 10, &FastLioPoll::cloudCb, this);

  // Optional publishers
  keyf_pose_pub_  = pnh_.advertise<nav_msgs::Odometry>("keyframe_odom", 10, true);
  keyf_cloud_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("keyframe_cloud", 10, true);
}

void FastLioPoll::odomCb(const nav_msgs::Odometry::ConstPtr& msg)
{
  latest_odom_ = *msg;
  latest_pose_ = msg->pose.pose;
  has_odom_    = true;

  // Try triggering keyframe on each odom update
  checkKeyFrameTrigger();
}

void FastLioPoll::cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  latest_cloud_ = *msg;
  has_cloud_    = true;

  // Optional: you can also trigger from here if you want "cloud-driven" keyframes
  // checkKeyFrameTrigger();
}

double FastLioPoll::computeTranslation(const geometry_msgs::Pose& a,
                                       const geometry_msgs::Pose& b) const
{
  double dx = a.position.x - b.position.x;
  double dy = a.position.y - b.position.y;
  double dz = a.position.z - b.position.z;
  ROS_INFO_STREAM("Translation" << dx << " " << dy << " " << dz);
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

void FastLioPoll::checkKeyFrameTrigger()
{
  // Debug: print latest pose
  ROS_INFO_STREAM("[FastLioPoll] Latest pose: "
    << latest_pose_.position.x << ", "
    << latest_pose_.position.y << ", "
    << latest_pose_.position.z);

  // Need both pose and cloud to make a meaningful keyframe
  if (!has_odom_ || !has_cloud_) {
    return;
  }

  // Debug: if we already have a keyframe, print its pose
  if (has_last_keyf_) {
    ROS_INFO_STREAM("[FastLioPoll] Last keyframe pose: "
      << last_keyf_pose_.position.x << ", "
      << last_keyf_pose_.position.y << ", "
      << last_keyf_pose_.position.z);
  }

  if (!has_last_keyf_) {
    // First keyframe
    createKeyFrame();
    return;
  }

  double trans = computeTranslation(latest_pose_, last_keyf_pose_);

  ROS_INFO_STREAM("[FastLioPoll] Δtranslation = " << trans
                    << " (threshold = " << keyf_translation_thresh_ << ")");

  if (trans > keyf_translation_thresh_) {
    createKeyFrame();
  }
}


void FastLioPoll::createKeyFrame()
{
  if (!has_odom_ || !has_cloud_) {
    ROS_WARN_THROTTLE(1.0, "[FastLioPoll] Tried to create keyframe without both odom and cloud.");
    return;
  }

  KeyFrame kf;
  kf.pose  = latest_pose_;
  kf.cloud = latest_cloud_;
  kf.stamp = latest_odom_.header.stamp;  // or latest_cloud_.header.stamp

  keyframes_.push_back(kf);

  last_keyf_pose_ = latest_pose_;
  has_last_keyf_  = true;
  keyf_count_++;

  ROS_INFO_STREAM("[FastLioPoll] Created keyframe #" << keyf_count_
                  << " at t=" << kf.stamp.toSec()
                  << " pos=(" << kf.pose.position.x << ", "
                               << kf.pose.position.y << ", "
                               << kf.pose.position.z << ")");

  // Optional: publish for visualization/recording
  if (keyf_pose_pub_.getNumSubscribers() > 0) {
    nav_msgs::Odometry kf_odom = latest_odom_;
    kf_odom.header.stamp = kf.stamp;  // ensure consistent
    keyf_pose_pub_.publish(kf_odom);
  }

  if (keyf_cloud_pub_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 cloud = latest_cloud_;
    cloud.header.stamp = kf.stamp;
    keyf_cloud_pub_.publish(cloud);
  }
}
