#ifndef _UTILITIES_TASK_GENERATOR_H_
#define _UTILITIES_TASK_GENERATOR_H_

#include "random_generator.h"
#include <ros/publisher.h>
#include "noisy_bool.h"
#include "noisy_double.h"
#include "noisy_long.h"
#include <talmech/feature.h>

namespace utilities
{
struct NoisyFeature
{
public:
  typedef boost::shared_ptr<NoisyFeature> Ptr;
  typedef boost::shared_ptr<const NoisyFeature> ConstPtr;
  NoisyFeature(const talmech::FeaturePtr& feature, const NoisyDoublePtr& level, double probability)
    : feature_(feature), level_(level), probability_(probability), distribution_(0.0, 1.0)
  {
    if (probability_ < 0.0 || probability_ > 1.0)
    {
      throw Exception("Invalid probability value. It must be in [0, 1].");
    }
  }
  talmech::FeaturePtr feature_;
  NoisyDoublePtr level_;
  double probability_;
  talmech_msgs::Feature toMsg() const
  {
    feature_->setLevel(level_->random());
    talmech_msgs::Feature msg(feature_->toMsg());
    return msg;
  }
  bool hasBeenChosen()
  {
    return distribution_(generator_) <= probability_;
  }
private:
  std::default_random_engine generator_;
  std::uniform_real_distribution<double> distribution_;

};
typedef NoisyFeature::Ptr NoisyFeaturePtr;
typedef NoisyFeature::ConstPtr NoisyFeatureConstPtr;
typedef std::vector<NoisyFeaturePtr> NoisyFeaturesPtr;
class TaskGenerator : public RandomGenerator
{
public:
  typedef boost::shared_ptr<TaskGenerator> Ptr;
  typedef boost::shared_ptr<const TaskGenerator> ConstPtr;
  TaskGenerator(const ros::NodeHandlePtr& nh,
                const NoisyDurationPtr& cycle_duration, int max,
                const NoisyLongPtr& waypoints,
                const NoisyDoublePtr& waypoint_x,
                const NoisyDoublePtr& waypoint_y,
                const NoisyFeaturesPtr& features);
  virtual ~TaskGenerator() { publisher_.shutdown(); }
private:
  ros::NodeHandlePtr nh_;
  ros::Publisher publisher_;
  NoisyLongPtr waypoints_;
  NoisyDoublePtr waypoint_x_;
  NoisyDoublePtr waypoint_y_;
  NoisyFeaturesPtr features_;
  virtual void generate();
};
typedef TaskGenerator::Ptr TaskGeneratorPtr;
typedef TaskGenerator::ConstPtr TaskGeneratorConstPtr;
}

#endif // _UTILITIES_TASK_GENERATOR_H_
