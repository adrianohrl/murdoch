/**
 * This source file implements the NoisyTime class, which
 *is
 *based on the ProbabilityDensityFunction class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/noisy_time.h"

namespace utilities
{
NoisyTime::NoisyTime(const Interval<ros::Time>& interval)
    : Noisy<ros::Time>::Noisy(interval.getMin().toSec(),
                              interval.getMax().toSec(), 3.0)
{
}

NoisyTime::NoisyTime(const ros::Time& min_t, const ros::Time& max)
    : Noisy<ros::Time>::Noisy(min_t.toSec(), max.toSec(), 3.0)
{
}

NoisyTime::NoisyTime(const ros::Time& mean,
                     const ros::Duration& standard_deviation)
    : Noisy<ros::Time>::Noisy(mean.toSec(), standard_deviation.toSec())
{
}

NoisyTime::NoisyTime(const NoisyTime& noisy) : Noisy<ros::Time>::Noisy(noisy) {}

NoisyTime::~NoisyTime() {}

double NoisyTime::probability(const ros::Time& t) const
{
  return ProbabilityDensityFunction::probability(t.toSec());
}

ros::Time NoisyTime::random()
{
  return ros::Time(ProbabilityDensityFunction::random());
}

ros::Time NoisyTime::getMean() const
{
  return ros::Time(ProbabilityDensityFunction::getMean());
}

ros::Duration NoisyTime::getStandardDeviation() const
{
  return ros::Duration(ProbabilityDensityFunction::getStandardDeviation());
}

Interval<ros::Time> NoisyTime::getFakeInterval() const
{
  return Interval<ros::Time>(ros::Time(mean_ - 3 * standard_deviation_),
                             ros::Time(mean_ + 3 * standard_deviation_));
}
}
