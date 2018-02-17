#ifndef _UTILITIES_NOISY_H_
#define _UTILITIES_NOISY_H_

#include "utilities/probability_density_function.h"

namespace utilities
{
template <typename T> class Noisy : public ProbabilityDensityFunction
{
public:
  virtual ~Noisy();
  virtual double probability(const T& value) const = 0;
  virtual T random() = 0;
  virtual Interval<T> getFakeInterval() const = 0;
  T operator+(const T& value);

protected:
  Noisy(double mean, double standard_deviation);
  Noisy(const Interval<T>& interval, double n = 3.0);
  Noisy(double min, double max, double n);
  Noisy(const Noisy& noisy);
};

template <typename T>
Noisy<T>::Noisy(double mean, double standard_deviation)
    : ProbabilityDensityFunction::ProbabilityDensityFunction(mean,
                                                             standard_deviation)
{
}

template <typename T>
Noisy<T>::Noisy(const Interval<T>& interval, double n)
    : ProbabilityDensityFunction::ProbabilityDensityFunction(
          interval.getMin(), interval.getMax(), n)
{
}

template <typename T>
Noisy<T>::Noisy(double min, double max, double n)
    : ProbabilityDensityFunction::ProbabilityDensityFunction(min, max, n)
{
}

template <typename T>
Noisy<T>::Noisy(const Noisy& noisy)
    : ProbabilityDensityFunction::ProbabilityDensityFunction(noisy)
{
}

template <typename T> Noisy<T>::~Noisy() {}

template <typename T> T Noisy<T>::operator+(const T& value)
{
  return value + random();
}
}

#endif // _UTILITIES_NOISY_H_
