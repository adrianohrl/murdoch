/**
 * This header file defines the ProbabilityDensityFunction class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_PROBABILITY_DENSITY_FUNCTION_H_
#define _UTILITIES_PROBABILITY_DENSITY_FUNCTION_H_

#include <boost/shared_ptr.hpp>
#include <random>
#include <sstream>
#include "utilities/interval.h"

namespace utilities
{
class ProbabilityDensityFunction
{
public:
  ProbabilityDensityFunction(double mean, double standard_deviation);
  ProbabilityDensityFunction(double neg_n_standard_deviation,
                             double pos_n_standard_deviation, double n);
  ProbabilityDensityFunction(const ProbabilityDensityFunction& pdf);
  virtual ~ProbabilityDensityFunction();
  double probability(double x) const;
  double random();
  double getMean() const;
  double getStandardDeviation() const;
  double getFakeMininum() const;
  double getFakeMaximum() const;
  Interval<double> getFakeInterval() const;
  std::string str() const;
  const char* c_str() const;

protected:
  const double mean_;
  const double standard_deviation_;

private:
  static constexpr double TOLERANCE = 1e-4;
  std::default_random_engine generator_;
  std::normal_distribution<double> distribution_;
  Interval<double> fake_interval_;
};

typedef boost::shared_ptr<ProbabilityDensityFunction>
    ProbabilityDensityFunctionPtr;
typedef boost::shared_ptr<ProbabilityDensityFunction const>
    ProbabilityDensityFunctionConstPtr;
}

#endif // _UTILITIES_PROBABILITY_DENSITY_FUNCTION_H_
