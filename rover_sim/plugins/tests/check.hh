// A minimal test harness for the plugin unit tests: CHECK and CHECK_NEAR count failures and say where; a test
// program's main() returns Failures() so ctest sees it.
#pragma once

#include <cmath>
#include <iostream>

inline int& Failures() {
  static int count = 0;
  return count;
}

#define CHECK(condition)                                                                    \
  do {                                                                                      \
    if (!(condition)) {                                                                     \
      std::cerr << __FILE__ << ":" << __LINE__ << ": CHECK(" #condition ") failed\n";       \
      ++Failures();                                                                         \
    }                                                                                       \
  } while (0)

#define CHECK_NEAR(value, expected, tolerance)                                              \
  do {                                                                                      \
    const double v_ = (value), e_ = (expected);                                             \
    if (!(std::abs(v_ - e_) <= (tolerance))) {                                              \
      std::cerr << __FILE__ << ":" << __LINE__ << ": " #value " = " << v_ << ", expected "  \
                << e_ << " +- " << (tolerance) << "\n";                                     \
      ++Failures();                                                                         \
    }                                                                                       \
  } while (0)
