#ifndef _UTILITIES_NOISY_BOOL_H_
#define _UTILITIES_NOISY_BOOL_H_

#include "noisy.h"

namespace utilities
{
class NoisyBool : public Noisy<double>
{
public:
  typedef boost::shared_ptr<NoisyBool> Ptr;
  typedef boost::shared_ptr<const NoisyBool> ConstPtr;
  NoisyBool(const Interval<double>& interval);
  NoisyBool(const double& mean, const double& standard_deviation);
  NoisyBool(const NoisyBool& noisy);
  virtual ~NoisyBool() {}
  virtual double probability(const ros::Duration& d) const;
  virtual ros::Duration random();
  virtual ros::Duration getMean() const;
  virtual Interval<ros::Duration> getFakeInterval() const;
};
typedef NoisyBool::Ptr NoisyBoolPtr;
typedef NoisyBool::ConstPtr NoisyBoolConstPtr;
}

#endif // _UTILITIES_NOISY_BOOL_H_
