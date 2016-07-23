#ifndef ROLLING_SUM_H
#define ROLLING_SUM_H

#include <boost/circular_buffer.hpp>

// Boost.Accumulators' rolling_sum is in 1.38 :(
template <typename T>
class RollingSum
{
public:
  RollingSum(size_t capacity)
    : buffer_(capacity), sum_(0)
  {
    buffer_.push_back(0);
  }

  void add(T sample)
  {
    sum_ += sample;
    sum_ -= buffer_[0];
    buffer_.push_back(sample);
  }

  T sum()
  {
    return sum_;
  }
  
private:
  boost::circular_buffer<T> buffer_;
  T sum_;
};

#endif
