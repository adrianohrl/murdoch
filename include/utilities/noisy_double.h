#ifndef _UTILITIES_NOISY_DOUBLE_H_
#define _UTILITIES_NOISY_DOUBLE_H_

#include "noisy.h"

namespace utilities
{
class NoisyDouble : public Noisy<double>
{
public:
  typedef boost::shared_ptr<NoisyDouble> Ptr;
  typedef boost::shared_ptr<const NoisyDouble> ConstPtr;
  NoisyDouble(const Interval<double>& interval)
    : Noisy<double>::Noisy(interval)
  {}
  NoisyDouble(const double& mean, const double& standard_deviation)
    : Noisy<double>::Noisy(mean, standard_deviation)
  {}
  NoisyDouble(const NoisyDouble& noisy)
    : Noisy<double>::Noisy(noisy)
  {}
  virtual ~NoisyDouble() {}
  virtual double probability(const double& value) const
  {
    return ProbabilityDensityFunction::probability(value);
  }
  virtual double random()
  {
    return ProbabilityDensityFunction::random();
  }
  virtual Interval<double> getFakeInterval() const
  {
    return Interval<double>(mean_ - 3 * standard_deviation_,
                            mean_ + 3 * standard_deviation_);
  }
};
typedef NoisyDouble::Ptr NoisyDoublePtr;
typedef NoisyDouble::ConstPtr NoisyDoubleConstPtr;
}

#endif // _UTILITIES_NOISY_DOUBLE_H_
