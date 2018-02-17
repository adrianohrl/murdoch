/**
 * This source file implements the ProbabilityDensityFunction class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/probability_density_function.h"
#include <cmath>

namespace utilities
{
ProbabilityDensityFunction::ProbabilityDensityFunction(
    double mean, double standard_deviation)
    : mean_(mean),
      standard_deviation_(fabs(standard_deviation == 0.0
                                   ? ProbabilityDensityFunction::TOLERANCE
                                   : standard_deviation)),
      distribution_(mean, standard_deviation),
      fake_interval_(mean_ - 3 * standard_deviation_,
                     mean_ + 3 * standard_deviation_)
{
}

ProbabilityDensityFunction::ProbabilityDensityFunction(
    double neg_n_standard_deviation, double pos_n_standard_deviation, double n)
    : mean_((pos_n_standard_deviation + neg_n_standard_deviation) / 2),
      standard_deviation_(
          fabs(pos_n_standard_deviation == neg_n_standard_deviation
                   ? ProbabilityDensityFunction::TOLERANCE
                   : pos_n_standard_deviation - neg_n_standard_deviation) /
          (2 * fabs(n == 0 ? ProbabilityDensityFunction::TOLERANCE : n))),
      distribution_(mean_, standard_deviation_),
      fake_interval_(mean_ - 3 * standard_deviation_,
                     mean_ + 3 * standard_deviation_)
{
}

ProbabilityDensityFunction::ProbabilityDensityFunction(
    const ProbabilityDensityFunction& pdf)
    : mean_(pdf.mean_), standard_deviation_(pdf.standard_deviation_),
      distribution_(pdf.distribution_), fake_interval_(pdf.fake_interval_)
{
}

ProbabilityDensityFunction::~ProbabilityDensityFunction() {}

double ProbabilityDensityFunction::probability(double x) const
{
  return 1 / (sqrt(2 * M_PI) * standard_deviation_) *
         exp(-0.5 * pow((x - mean_) / standard_deviation_, 2));
}

double ProbabilityDensityFunction::random()
{
  return distribution_(generator_);
}

double ProbabilityDensityFunction::getMean() const { return mean_; }

double ProbabilityDensityFunction::getStandardDeviation() const
{
  return standard_deviation_;
}

double ProbabilityDensityFunction::getFakeMininum() const
{
  return fake_interval_.getMin();
}

double ProbabilityDensityFunction::getFakeMaximum() const
{
  return fake_interval_.getMax();
}

Interval<double> ProbabilityDensityFunction::getFakeInterval() const
{
  return fake_interval_;
}

std::string ProbabilityDensityFunction::str() const
{
  std::stringstream ss;
  ss << "X ~ N(" << mean_ << "; " << standard_deviation_ * standard_deviation_
     << ")";
  return ss.str();
}

const char* ProbabilityDensityFunction::c_str() const { return str().c_str(); }
}
