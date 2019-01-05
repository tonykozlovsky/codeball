#ifndef CODEBALL_COLLISIONTRIGGER_H
#define CODEBALL_COLLISIONTRIGGER_H

struct CollisionTrigger {
  double prev_distance, cur_distance;
  bool yes;

  int calculateNumberOfMicrotick(double mictoticks_per_tick) {
    double delta = (prev_distance - cur_distance) / mictoticks_per_tick;
    return floor(prev_distance / delta);
  }

};

#endif //CODEBALL_COLLISIONTRIGGER_H
