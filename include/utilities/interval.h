/**
 * This header file defines the Interval class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_INTERVAL_H_
#define _UTILITIES_INTERVAL_H_

#include <boost/shared_ptr.hpp>
#include <ros/time.h>
#include <sstream>
#include "utilities/exception.h"

namespace utilities
{
template <typename T> class Interval
{
public:
  typedef boost::shared_ptr<Interval<T> > Ptr;
  typedef boost::shared_ptr<Interval<T> const> ConstPtr;
  Interval(const T& min, const T& max, bool including_min = true,
           bool including_max = true);
  Interval(const Interval<T>& interval);
  virtual ~Interval();
  bool belongs(const T& value) const;
  T getValid(const T& value) const;
  T getMin() const;
  T getMax() const;
  void setMin(const T& min);
  void setMax(const T& max);
  std::string str() const;
  const char* c_str() const;
  template <typename U>
  friend std::ostream& operator<<(std::ostream& out,
                                  const Interval<U>& interval);

private:
  T min_;
  T max_;
  bool including_min_;
  bool including_max_;
};

typedef Interval<ros::Duration> DurationInterval;
typedef DurationInterval::Ptr DurationIntervalPtr;
typedef DurationInterval::ConstPtr DurationIntervalConstPtr;
typedef Interval<ros::Time> TimeInterval;
typedef TimeInterval::Ptr TimeIntervalPtr;
typedef TimeInterval::ConstPtr TimeIntervalConstPtr;

// Time, Duration, Continuous, Discrete and Unary

template <typename T>
Interval<T>::Interval(const T& min, const T& max, bool including_min,
                      bool including_max)
    : min_(min), max_(max), including_min_(including_min),
      including_max_(including_max)
{
  if (min_ > max_)
  {
    throw utilities::Exception(
        "The interval minimum bound must not be greater than its maximum one.");
  }
}

template <typename T>
Interval<T>::Interval(const Interval<T>& interval)
    : min_(interval.min_), max_(interval.max_),
      including_min_(interval.including_min_),
      including_max_(interval.including_max_)
{
}

template <typename T> Interval<T>::~Interval() {}

template <typename T> bool Interval<T>::belongs(const T& value) const
{
  return including_min_ ? value >= min_ : value > min_ && including_max_
                                              ? value <= max_
                                              : value < max_;
}

template <typename T> T Interval<T>::getValid(const T& value) const
{
  return value < min_ ? min_ : value > max_ ? max_ : value;
}

template <typename T> T Interval<T>::getMin() const { return min_; }

template <typename T> T Interval<T>::getMax() const { return max_; }

template <typename T> void Interval<T>::setMin(const T& min)
{
  if (min <= max_)
  {
    min_ = min;
  }
}

template <typename T> void Interval<T>::setMax(const T& max)
{
  if (max >= min_)
  {
    max_ = max;
  }
}

template <typename T> std::string Interval<T>::str() const
{
  std::stringstream ss;
  ss << (including_min_ ? "[" : "(") << min_;
  ss << " ; " << max_ << (including_max_ ? "]" : ")");
  return ss.str();
}

template <typename T> const char* Interval<T>::c_str() const
{
  return str().c_str();
}

template <typename T>
std::ostream& operator<<(std::ostream& out, const Interval<T>& interval)
{
  out << interval.str();
  return out;
}
}

#endif // _UTILITIES_INTERVAL_H_
