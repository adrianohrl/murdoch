#include "utilities/noisy_duration.h"

namespace utilities
{
NoisyDuration::NoisyDuration(const Interval<ros::Duration>& interval)
    : Noisy<ros::Duration>::Noisy(interval.getMin().toSec(),
                                  interval.getMax().toSec(), 3.0)
{
}

NoisyDuration::NoisyDuration(const ros::Duration& min, const ros::Duration& max)
    : Noisy<ros::Duration>::Noisy(min.toSec(), max.toSec(), 3.0)
{
}

NoisyDuration::NoisyDuration(const ros::Duration& mean,
                             const double& standard_deviation)
    : Noisy<ros::Duration>::Noisy(mean.toSec(), standard_deviation)
{
}

NoisyDuration::NoisyDuration(const NoisyDuration& noisy)
    : Noisy<ros::Duration>::Noisy(noisy)
{
}

NoisyDuration::~NoisyDuration() {}

double NoisyDuration::probability(const ros::Duration& d) const
{
  return ProbabilityDensityFunction::probability(d.toSec());
}

ros::Duration NoisyDuration::random()
{
  return ros::Duration(ProbabilityDensityFunction::random());
}

ros::Duration NoisyDuration::getMean() const
{
  return ros::Duration(ProbabilityDensityFunction::getMean());
}

Interval<ros::Duration> NoisyDuration::getFakeInterval() const
{
  return Interval<ros::Duration>(
      ros::Duration(mean_ - 3 * standard_deviation_),
      ros::Duration(mean_ + 3 * standard_deviation_));
}
}
