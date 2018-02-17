/**
 * This header file defines the NoisyTime class, which is
 *based on the ProbabilityDensityFunction class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_NOISY_TIME_H_
#define _UTILITIES_NOISY_TIME_H_

#include <ros/time.h>
#include "utilities/noisy.h"

namespace utilities
{
class NoisyTime : public Noisy<ros::Time>
{
public:
  NoisyTime(const Interval<ros::Time>& interval);
  NoisyTime(const ros::Time& min, const ros::Time& max);
  NoisyTime(const ros::Time& mean, const ros::Duration& standard_deviation);
  NoisyTime(const NoisyTime& noisy);
  virtual ~NoisyTime();
  virtual double probability(const ros::Time& t = ros::Time::now()) const;
  virtual ros::Time random();
  virtual ros::Time getMean() const;
  virtual ros::Duration getStandardDeviation() const;
  virtual Interval<ros::Time> getFakeInterval() const;
};

typedef boost::shared_ptr<NoisyTime> NoisyTimePtr;
typedef boost::shared_ptr<NoisyTime const> NoisyTimeConstPtr;
}

#endif // _UTILITIES_NOISY_TIME_H_
