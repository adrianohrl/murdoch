#include "utilities/task_generator.h"
#include <talmech_msgs/Task.h>
#include <ros/node_handle.h>
#include <sstream>

namespace utilities
{
TaskGenerator::TaskGenerator(const ros::NodeHandlePtr& nh,
                             const NoisyDurationPtr& cycle_duration, int max,
                             const NoisyLongPtr& waypoints,
                             const NoisyDoublePtr& waypoint_x,
                             const NoisyDoublePtr& waypoint_y,
                             const NoisyFeaturesPtr& features)
    : RandomGenerator(cycle_duration, max), nh_(nh),
      waypoint_x_(waypoint_x), waypoints_(waypoints), waypoint_y_(waypoint_y),
      features_(features)
{
  publisher_ = nh_->advertise<talmech_msgs::Task>("/murdoch/task", 1);
}

void TaskGenerator::generate()
{
  talmech_msgs::Task msg;
  std::stringstream ss;
  ss << "task" << counter_;
  msg.id = ss.str();
  ROS_INFO_STREAM("[TaskGenerator::generate()] id: " << msg.id);
  for (int i(0); i < features_.size(); i++)
  {
      NoisyFeaturePtr feature(features_.at(i));
      if (feature->hasBeenChosen())
      {
        ROS_INFO_STREAM("[TaskGenerator::generate()] index: " << i << ", feature: " << *feature->feature_);
        msg.features.push_back(feature->toMsg());
      }
  }
  long size(waypoints_->random());
  if (size < 0)
  {
    size = 0;
  }
  ROS_INFO_STREAM("[TaskGenerator::generate()] waypoints size: " << size);
  for (int i(0); i < size; i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = waypoint_x_->random();
    pose.pose.position.y = waypoint_y_->random();
    ROS_INFO_STREAM("[TaskGenerator::generate()] waypoint "
                    << i << ": (" << pose.pose.position.x << ", "
                    << pose.pose.position.y << ")");
    msg.waypoints.poses.push_back(pose);
  }
  publisher_.publish(msg);
}
}
