#ifndef _UTILITIES_TASK_GENERATOR_H_
#define _UTILITIES_TASK_GENERATOR_H_

#include "random_generator.h"
#include <ros/publisher.h>
#include "noisy_double.h"
#include "noisy_long.h"
#include <talmech/skill.h>

namespace utilities
{
struct NoisySkill
{
public:
  typedef boost::shared_ptr<NoisySkill> Ptr;
  typedef boost::shared_ptr<const NoisySkill> ConstPtr;
  NoisySkill(const talmech::SkillPtr& skill, const NoisyDoublePtr& level)
    : skill_(skill), level_(level)
  {}
  talmech::SkillPtr skill_;
  NoisyDoublePtr level_;
  talmech_msgs::Skill toMsg() const
  {
    talmech_msgs::Skill msg(skill_->toMsg());
    if (msg.type != 0)
    {
      msg.level = level_->random();
    }
    return msg;
  }
};
typedef NoisySkill::Ptr NoisySkillPtr;
typedef NoisySkill::ConstPtr NoisySkillConstPtr;
typedef std::vector<NoisySkillPtr> NoisySkillsPtr;
class TaskGenerator : public RandomGenerator
{
public:
  typedef boost::shared_ptr<TaskGenerator> Ptr;
  typedef boost::shared_ptr<const TaskGenerator> ConstPtr;
  TaskGenerator(const ros::NodeHandlePtr& nh,
                const NoisyDurationPtr& cycle_duration,
                const NoisyLongPtr& waypoints,
                const NoisyDoublePtr& waypoint_x,
                const NoisyDoublePtr& waypoint_y,
                const NoisySkillsPtr& skills);
  virtual ~TaskGenerator() { publisher_.shutdown(); }
private:
  ros::NodeHandlePtr nh_;
  ros::Publisher publisher_;
  int counter_;
  NoisyLongPtr waypoints_;
  NoisyDoublePtr waypoint_x_;
  NoisyDoublePtr waypoint_y_;
  NoisySkillsPtr skills_;
  virtual void generate();
};
typedef TaskGenerator::Ptr TaskGeneratorPtr;
typedef TaskGenerator::ConstPtr TaskGeneratorConstPtr;
}

#endif // _UTILITIES_TASK_GENERATOR_H_
