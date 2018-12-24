#ifndef CODEBALL_MYTIMER_H
#define CODEBALL_MYTIMER_H

#include <chrono>

struct MyTimer {
  std::chrono::time_point<std::chrono::high_resolution_clock> _start, _end;
  MyTimer() {
    start();
    end();
  }

  double cumulative = 0;
  int k = 0;
  double max_ = 0;

  void start() {
    _start = std::chrono::high_resolution_clock::now();
  }
  void end() {
    _end = std::chrono::high_resolution_clock::now();
  }

  double delta(bool accumulate = false) {
    double res = 1e-9 * std::chrono::duration_cast<std::chrono::nanoseconds>(_end - _start).count();
    if (accumulate) {
      cumulative += res;
      k++;
      return cumulative;
    }
    return res;
  }
  double cur(bool accumulate = false, bool counter = false, bool upd_max_ = false) {
    double res = 1e-9 * std::chrono::duration_cast<std::chrono::nanoseconds>
        (std::chrono::high_resolution_clock::now() - _start).count();
    cumulative -= 645e-10;
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
