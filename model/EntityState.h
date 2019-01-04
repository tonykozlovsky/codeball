#ifndef CODEBALL_ENTITY_STATE_H
#define CODEBALL_ENTITY_STATE_H


#ifdef LOCAL
#include <model/C.h>
#else
#include "C.h"
#endif


struct EntityState {
  Point position;
  Point velocity;
  double radius;

  bool touch;
  Point touch_normal;

  double radius_change_speed;

  EntityState() {}

  EntityState(const Point& position, const Point& velocity, const double radius, const bool touch, const Point& touch_normal, const double radius_change_speed) {
    this->position = position;
    this->velocity = velocity;
    this->radius = radius;

    this->touch = touch;
    this->touch_normal = touch_normal;

    this->radius_change_speed = radius_change_speed;
  }

};

#endif //CODEBALL_ENTITY_STATE_H
