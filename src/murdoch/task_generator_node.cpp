#include "murdoch/task_generator_node.h"
#include "utilities/task_generator.h"
#include <talmech/discrete_skill.h>
#include <talmech/continuous_skill.h>

namespace murdoch
{
TaskGeneratorNode::TaskGeneratorNode(const ros::NodeHandlePtr& nh, const ros::Rate& rate)
    : ROSNode::ROSNode(nh, rate)
{
}

void TaskGeneratorNode::readParameters()
{
  ROS_INFO("Reading parameters...");
  ros::NodeHandle pnh("~");
  double cycle_duration_mean, cycle_duration_std;
  pnh.param("cycle_duration/mean", cycle_duration_mean, 0.0);
  ROS_INFO_STREAM("   ~/cycle_duration/mean: " << cycle_duration_mean);
  pnh.param("cycle_duration/standard_deviation", cycle_duration_std, 0.0);
  ROS_INFO_STREAM("   ~/cycle_duration/standard_deviation: " << cycle_duration_std);
  utilities::NoisyDurationPtr cycle_duration(
      new utilities::NoisyDuration(ros::Duration(cycle_duration_mean), cycle_duration_std));
  int waypoints_mean, waypoints_std;
  pnh = ros::NodeHandle("~/waypoints");
  pnh.param("mean", waypoints_mean, 0);
  ROS_INFO_STREAM("   ~/waypoints/mean: " << waypoints_mean);
  pnh.param("standard_deviation", waypoints_std, 1);
  ROS_INFO_STREAM("   ~/waypoints/standard_deviation: " << waypoints_std);
  utilities::NoisyLongPtr waypoints(
        new utilities::NoisyLong(waypoints_mean, waypoints_std));
  double waypoints_x_mean, waypoints_x_std;
  pnh.param("x/mean", waypoints_x_mean, 0.0);
  ROS_INFO_STREAM("   ~/waypoints/x/mean: " << waypoints_x_mean);
  pnh.param("x/standard_deviation", waypoints_x_std, 1.0);
  ROS_INFO_STREAM("   ~/waypoints/x/standard_deviation: " << waypoints_x_std);
  utilities::NoisyDoublePtr waypoints_x(
        new utilities::NoisyDouble(waypoints_x_mean, waypoints_x_std));
  double waypoints_y_mean, waypoints_y_std;
  pnh.param("y/mean", waypoints_y_mean, 0.0);
  ROS_INFO_STREAM("   ~/waypoints/y/mean: " << waypoints_y_mean);
  pnh.param("y/standard_deviation", waypoints_y_std, 1.0);
  ROS_INFO_STREAM("   ~/waypoints/y/standard_deviation: " << waypoints_y_std);
  utilities::NoisyDoublePtr waypoints_y(
        new utilities::NoisyDouble(waypoints_y_mean, waypoints_y_std));
  pnh = ros::NodeHandle("~/skills");
  utilities::NoisySkillsPtr skills;
  int size;
  pnh.param("size", size, 0);
  ROS_INFO_STREAM("   ~/skills/size: " << size);
  for (int i(0); i < size; i++)
  {
    std::stringstream ss;
    ss << "skill" << i << "/";
    std::string resource_id;
    pnh.param(ss.str() + "resource", resource_id, std::string(""));
    ROS_INFO_STREAM("   ~/skills/skill" << i << "/resource: " << resource_id);
    talmech::ResourcePtr resource(new talmech::Resource(resource_id));
    int type;
    pnh.param(ss.str() + "type", type, 0);
    ROS_INFO_STREAM("   ~/skills/skill" << i << "/type: " << type);
    double level_mean, level_std;
    pnh.param(ss.str() + "level/mean", level_mean, 0.0);
    ROS_INFO_STREAM("   ~/skills/skill" << i << "/level/mean: " << level_mean);
    pnh.param(ss.str() + "level/standard_deviation", level_std, 1.0);
    ROS_INFO_STREAM("   ~/skills/skill" << i << "/level/standard_deviation: " << level_std);
    utilities::NoisyDoublePtr noisy_level(
          new utilities::NoisyDouble(level_mean, level_std));
    talmech::SkillPtr skill;
    if (type == 0)
    {
      skill.reset(new talmech::Skill(resource));
    }
    else if (type == 1)
    {
      long level(round(noisy_level->random()));
      ROS_INFO_STREAM("   ~/skills/skill" << i << "/level: " << level);
      skill.reset(new talmech::DiscreteSkill(resource, level));
    }
    else if (type == 2)
    {
      double level(noisy_level->random());
      ROS_INFO_STREAM("   ~/skills/skill" << i << "/level: " << level);
      skill.reset(new talmech::ContinuousSkill(resource, level));
    }
    else
    {
      throw utilities::Exception("Invalid type of skill.");
    }
    utilities::NoisySkillPtr noisy_skill(new utilities::NoisySkill(skill, noisy_level));
    skills.push_back(noisy_skill);
  }
  generator_.reset(new utilities::TaskGenerator(nh_, cycle_duration, waypoints, waypoints_x, waypoints_y, skills));
}
}
