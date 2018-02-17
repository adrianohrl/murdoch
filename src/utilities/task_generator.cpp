#include "utilities/task_generator.h"
#include <talmech_msgs/Task.h>
#include <ros/node_handle.h>
#include <sstream>

namespace utilities
{
TaskGenerator::TaskGenerator(const ros::NodeHandlePtr& nh,
                             const NoisyDurationPtr& cycle_duration,
                             const NoisyLongPtr& waypoints,
                             const NoisyDoublePtr& waypoint_x,
                             const NoisyDoublePtr& waypoint_y,
                             const NoisySkillsPtr& skills)
    : RandomGenerator(cycle_duration), nh_(nh), counter_(1),
      waypoint_x_(waypoint_x), waypoints_(waypoints), waypoint_y_(waypoint_y),
      skills_(skills)
{
  publisher_ = nh_->advertise<talmech_msgs::Task>("/murdoch/task", 1);
}

void TaskGenerator::generate()
{
  talmech_msgs::Task msg;
  std::stringstream ss;
  ss << "task" << counter_++;
  msg.id = ss.str();
  long size(skills_.size());
  srand(time(NULL));
  for (int i(0); i < size; i++)
  {
    std::size_t index(rand() % skills_.size());
    ROS_INFO_STREAM("[TaskGenerator::generate()] index: " << index);
    NoisySkillPtr skill(skills_.at(index));
    msg.skills.push_back(skill->toMsg());
  }
  size = waypoints_->random();
  for (int i(0); i < size; i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = waypoint_x_->random();
    pose.pose.position.y = waypoint_y_->random();
    msg.waypoints.poses.push_back(pose);
  }
  publisher_.publish(msg);
}
}
