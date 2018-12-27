#ifndef CODEBALL_GETCPUTIME_H
#define CODEBALL_GETCPUTIME_H

struct CPUTime {
  static double getCPUTime() {
    clock_t cl = clock();
    return (double) cl / (double) CLOCKS_PER_SEC;
  }
};

#endif //CODEBALL_GETCPUTIME_H
