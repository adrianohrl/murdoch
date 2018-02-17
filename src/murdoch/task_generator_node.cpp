#include "murdoch/task_generator_node.h"
#include "utilities/task_generator.h"

namespace murdoch
{
TaskGeneratorNode::TaskGeneratorNode(const ros::NodeHandlePtr& nh, const ros::Rate& rate)
    : ROSNode::ROSNode(nh, rate)
{
}

void TaskGeneratorNode::readParameters()
{
  ros::NodeHandle pnh("~");
  double cycle_duration_mean, cycle_duration_std;
  pnh.param("cycle_duration/mean", cycle_duration_mean, 0.0);
  pnh.param("cycle_duration/standard_deviation", cycle_duration_std, 0.0);
  utilities::NoisyDurationPtr cycle_duration(
      new utilities::NoisyDuration(ros::Duration(cycle_duration_mean), cycle_duration_std));
  int waypoints_mean, waypoints_std;
  pnh = ros::NodeHandle("~/waypoints");
  pnh.param("mean", waypoints_mean, 0);
  pnh.param("standard_deviation", waypoints_std, 1);
  utilities::NoisyLongPtr waypoints(
        new utilities::NoisyLong(waypoints_mean, waypoints_std));
  double waypoints_x_mean, waypoints_x_std;
  pnh.param("x/mean", waypoints_x_mean, 0.0);
  pnh.param("x/standard_deviation", waypoints_x_std, 1.0);
  utilities::NoisyDoublePtr waypoints_x(
        new utilities::NoisyDouble(waypoints_x_mean, waypoints_x_std));
  double waypoints_y_mean, waypoints_y_std;
  pnh.param("y/mean", waypoints_y_mean, 0.0);
  pnh.param("y/standard_deviation", waypoints_y_std, 1.0);
  utilities::NoisyDoublePtr waypoints_y(
        new utilities::NoisyDouble(waypoints_y_mean, waypoints_y_std));
  utilities::NoisySkillsPtr skills;
  int size;
  pnh.param("size", size, 0);
  for (int i(0); i < size; i++)
  {
    std::stringstream ss;
    ss << "skill" << i << "/";
    std::string resource_id;
    pnh.param(ss.str() + "resource", resource_id, std::string(""));
    talmech::ResourcePtr resource(new talmech::Resource(resource_id));
    talmech::SkillPtr skill(new talmech::Skill(resource));
    double level_mean, level_std;
    pnh.param(ss.str() + "level/mean", level_mean, 0.0);
    pnh.param(ss.str() + "level/standard_deviation", level_std, 1.0);
    utilities::NoisyDoublePtr level(
          new utilities::NoisyDouble(level_mean, level_std));
    utilities::NoisySkillPtr noisy_skill(new utilities::NoisySkill(skill, level));
    skills.push_back(noisy_skill);
  }
  generator_.reset(new utilities::TaskGenerator(nh_, cycle_duration, waypoints, waypoints_x, waypoints_y, skills));
}
}
