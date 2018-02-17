#ifndef _UTILITIES_NOISY_BOOL_H_
#define _UTILITIES_NOISY_BOOL_H_

#include "noisy.h"

namespace utilities
{
class NoisyBool : public Noisy<bool>
{
public:
  typedef boost::shared_ptr<NoisyBool> Ptr;
  typedef boost::shared_ptr<const NoisyBool> ConstPtr;
  NoisyBool()
    : Noisy<bool>::Noisy(5000, 5000)
  {}
  NoisyBool(const NoisyBool& noisy)
    : Noisy<bool>::Noisy(noisy)
  {}
  virtual ~NoisyBool() {}
  virtual double probability(const bool& d) const { return 0.5; }
  virtual bool random()
  {
    long value(round(ProbabilityDensityFunction::random()));
    return value % 2 == 0;
  }
  virtual bool getMean() const { return 1.0; }
  virtual Interval<bool> getFakeInterval() const
  {
    return Interval<bool>(false, true);
  }
};
typedef NoisyBool::Ptr NoisyBoolPtr;
typedef NoisyBool::ConstPtr NoisyBoolConstPtr;
}

#endif // _UTILITIES_NOISY_BOOL_H_
