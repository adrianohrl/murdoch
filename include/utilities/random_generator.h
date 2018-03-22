#ifndef _UTILITIES_RANDOM_GENERATOR_H_
#define _UTILITIES_RANDOM_GENERATOR_H_

#include <ros/time.h>
#include "noisy_duration.h"
#include <ros/console.h>

namespace utilities
{
class RandomGenerator
{
public:
  typedef boost::shared_ptr<RandomGenerator> Ptr;
  typedef boost::shared_ptr<const RandomGenerator> ConstPtr;
  virtual ~RandomGenerator() {}
  void process()
  {
    ros::Time timestamp(ros::Time::now());
    if (timestamp_ < timestamp)
    {
      if (!isDone())
      {
        counter_++;
        timestamp_ = timestamp + cycle_duration_->random();
        ROS_INFO_STREAM("Randomly generating ... (silence duration: " << (timestamp_ - timestamp).toSec() << "[s]).");
        generate();
      }
    }
  }
  bool isDone() const { return max_ >= 0 && counter_ >= max_; }
protected:
  RandomGenerator(const NoisyDurationPtr& cycle_duration, int max = -1)
    : cycle_duration_(cycle_duration), max_(max), counter_(0)
  {
    if (!cycle_duration_)
    {
      throw Exception("The RandomGenerator cycle duration must not be null.");
    }
    timestamp_ = ros::Time::now() + cycle_duration_->random();
  }
  int max_;
  int counter_;
private:
  virtual void generate() = 0;
  ros::Time timestamp_;
  NoisyDurationPtr cycle_duration_;
};
typedef RandomGenerator::Ptr RandomGeneratorPtr;
typedef RandomGenerator::ConstPtr RandomGeneratorConstPtr;
}

#endif // _UTILITIES_RANDOM_GENERATOR_H_
