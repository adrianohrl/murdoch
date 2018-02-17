#ifndef _UTILITIES_NOISY_LONG_H_
#define _UTILITIES_NOISY_LONG_H_

#include "noisy.h"

namespace utilities
{
class NoisyLong : public Noisy<long>
{
public:
  typedef boost::shared_ptr<NoisyLong> Ptr;
  typedef boost::shared_ptr<const NoisyLong> ConstPtr;
  NoisyLong(const Interval<long>& interval)
    : Noisy<long>::Noisy(interval)
  {}
  NoisyLong(const long& mean, const long& standard_deviation)
    : Noisy<long>::Noisy(mean, standard_deviation)
  {}
  NoisyLong(const NoisyLong& noisy)
    : Noisy<long>::Noisy(noisy)
  {}
  virtual ~NoisyLong() {}
  virtual double probability(const long& value) const
  {
    return ProbabilityDensityFunction::probability(value);
  }
  virtual long random() { return round(ProbabilityDensityFunction::random()); }
  virtual Interval<long> getFakeInterval() const
  {
    return Interval<long>(mean_ - 3 * standard_deviation_,
                          mean_ + 3 * standard_deviation_);
  }
};
typedef NoisyLong::Ptr NoisyLongPtr;
typedef NoisyLong::ConstPtr NoisyLongConstPtr;
}

#endif // _UTILITIES_NOISY_LONG_H_
