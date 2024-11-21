#pragma once

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <iostream>  // for std::cerr
#include <optional>
#include <vector>

#ifndef CHECK1

#define CHECK1(condition, message)                                                                                        \
  (!(condition)) ? (std::cerr << "Assertion failed: (" << #condition << "), " << "function " << __FUNCTION__ << ", file " \
                              << __FILE__ << ", line " << __LINE__ << "." << std::endl                                    \
                              << message << std::endl,                                                                    \
                    abort(), 0)                                                                                           \
                 : 1

#endif  // CHECK1

#ifndef CHECK2
#if defined DEBUG || defined _DEBUG
#define CHECK2(condition, message)                                                                                        \
  (!(condition)) ? (std::cerr << "Assertion failed: (" << #condition << "), " << "function " << __FUNCTION__ << ", file " \
                              << __FILE__ << ", line " << __LINE__ << "." << std::endl                                    \
                              << message << std::endl,                                                                    \
                    abort(), 0)                                                                                           \
                 : 1
#else
#define CHECK2(condition, message) (void)0
#endif
#endif  // CHECK2

#include <algorithm>  // for std::max

// https://bitbashing.io/comparing-floats.html
// https://www.learncpp.com/cpp-tutorial/relational-operators-and-floating-point-comparisons/

template <typename T>
constexpr T constAbs(T x) {
  return (x < 0 ? -x : x);
}

// Return true if the difference between a and b is within epsilon percent of
// the larger of a and b
constexpr bool approximatelyEqualRel(double a, double b, double relEpsilon) {
  return (constAbs(a - b) <= (std::max(constAbs(a), constAbs(b)) * relEpsilon));
}

// Return true if the difference between a and b is less than or equal to
// absEpsilon, or within relEpsilon percent of the larger of a and b
constexpr bool approximatelyEqualAbsRel(double a, double b, double absEpsilon, double relEpsilon) {
  // Check if the numbers are really close -- needed when comparing numbers near zero.
  if (constAbs(a - b) <= absEpsilon) return true;

  // Otherwise fall back to Knuth's algorithm
  return approximatelyEqualRel(a, b, relEpsilon);
}

template <typename ConditionFunc, typename ActionFunc>
inline void condition_exec(ConditionFunc condFunc, ActionFunc actFunc) {
  if (condFunc()) {
    actFunc();
  }
}

namespace std::extend {

template <typename T>
class vector_ext_find {
 public:
  vector_ext_find() = default;
  vector_ext_find(T value) : m_value{value} {}

  template <typename Func>
  const vector_ext_find<T>& and_then(const Func&& func) const {
    if (m_value.has_value()) {
      func(m_value.value());
    }
    return *this;
  }

  template <typename Func>
  void or_else(const Func&& func) const {
    if (!m_value.has_value()) {
      func();
    }
  }

 private:
  std::optional<T> m_value{std::nullopt};
};

template <typename T>
vector_ext_find<T> find(const std::vector<T>& v, const T value) {
  auto it = std::find(v.begin(), v.end(), value);
  if (it == v.end()) {
    return vector_ext_find<T>();
  } else {
    return vector_ext_find<T>(*it);
  }
}

template <typename T>
struct atomwrapper {
  std::atomic<T> val_;

  atomwrapper() : val_() {}
  atomwrapper(const std::atomic<T>& a) : val_(a.load()) {}
  atomwrapper(const atomwrapper& other) : val_(other.val_.load()) {}
  atomwrapper& operator=(const atomwrapper& other) { val_.store(other.val_.load()); }
};

}  // namespace std::extend