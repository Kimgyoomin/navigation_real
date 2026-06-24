/**
 * @file Utilities.h
 * @author SungJoon Yoon  (densee250@gmail.com)
 * @brief Common utility functions (Quadruped)
 * @version 1.0
 * @date 2024-11-21
 *
 * @copyright Copyright (c) 2024, Robotics & Control Lab.
 *
 */

#include <cassert>
#include <string>

#ifndef UTILITIES_H
#define UTILITIES_H


namespace pongbot_heightmap_cupy {

/*!
 * Force input values ​​to min or max
 */
template <typename T>
T coerce(T value, T min, T max) 
{
  if (value < min) 
  {
    value = min;
  }
  if (value > max) 
  {
    value = max;
  }
  return value;
}

/*!
 * Apply deadband
 * @param value : input
 * @param range : deadband (+/- range around 0)
 * @return result
 */
template <typename T>
T deadband(T value, T range) 
{
  if (value < range && value > -range) value = T(0);
  return value;
}

/*!
 * Get the sign of a number
 * 1 for positive, 0 for 0, -1 for negative...
 */
template <typename T>
int sgn(T value) 
{
  return (T(0) < value) - (value < T(0));
}

std::string getLCMUrl(int64_t ttl);

} 
#endif /* UTILITIES_H */
