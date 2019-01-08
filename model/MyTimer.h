#ifndef CODEBALL_MYTIMER_H
#define CODEBALL_MYTIMER_H

//#define CHRONO

#ifdef CHRONO
#include <chrono>
#else
#ifdef LOCAL
#include <model/getCPUTime.h>
#else
#include "getCPUTime.h"
#endif
#endif

struct MyTimer {
#ifdef CHRONO
  std::chrono::time_point<std::chrono::high_resolution_clock> _start, _end;
#else
  double _start = 0, _end = 0;
#endif
  MyTimer() {
    start();
    end();
  }

  double cumulative = 0;
  int k = 0;
  double max_ = 0;

  void start() {
#ifdef CHRONO
    _start = std::chrono::high_resolution_clock::now();
#else
    _start = CPUTime::getCPUTime();
#endif
  }
  void end() {
#ifdef CHRONO
    _end = std::chrono::high_resolution_clock::now();
#else
    _end = CPUTime::getCPUTime();
#endif
  }

  double delta(bool accumulate = false) {
#ifdef CHRONO
    double res = 1e-9 * std::chrono::duration_cast<std::chrono::nanoseconds>(_end - _start).count();
#else
    double res = _end - _start;
#endif
    if (accumulate) {
      cumulative += res;
      k++;
      return cumulative;
    }
    return res;
  }
  double cur(bool accumulate = false, bool counter = false, bool upd_max_ = false) {
#ifdef CHRONO
    double res = 1e-9 * std::chrono::duration_cast<std::chrono::nanoseconds>
        (std::chrono::high_resolution_clock::now() - _start).count();
    cumulative -= 640e-10;
#else
    double res = CPUTime::getCPUTime() - _start;
    cumulative -= 0.0000008715;
#endif
    if (counter) {
      k++;
    }
    if (upd_max_) {
      max_ = std::max(max_, res);
    }
    if (accumulate) {
      cumulative += res;
    }
    return res;
  }

  void clear() {
    cumulative = 0;
    k = 0;
    max_ = 0;
  }

  double avg() {
    return cumulative / k;
  }

  double max() {
    return max_;
  }

  double getCumulative(bool with_cur = false) {
    if (!with_cur) {
      return cumulative;
    }
    return cumulative + cur();
  }

};

#endif //CODEBALL_MYTIMER_H
