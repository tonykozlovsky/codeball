#ifndef CODEBALL_CONSTANTS_H
#define CODEBALL_CONSTANTS_H

#ifdef LOCAL
#include <model/Arena.h>
#include <model/Action.h>
#include <model/Robot.h>
#include <model/Rules.h>
#include <model/Game.h>
#include <model/Player.h>
#include <model/Ball.h>
#include <model/MyTimer.h>
#include <model/Point.h>
#include <model/Point2d.h>
#include <model/MyAction.h>
#else
#include "Arena.h"
#include "Action.h"
#include "Robot.h"
#include "Rules.h"
#include "Game.h"
#include "Player.h"
#include "Ball.h"
#include "MyTimer.h"
#include "Point.h"
#include "Point2d.h"
#include "MyAction.h"
#endif

#include <math.h>
#include <algorithm>
#include <iostream>
#include <map>
#include <iomanip>
#include <random>

struct Constants {
  static model::Rules rules;

  static constexpr int MAX_SIMULATION_DEPTH = 200;
  static constexpr double time_limit = 295.;

  static std::mt19937_64 rd;

  static double rand_double(double a, double b) {
    return a + (double)rd() / rd.max() * (b - a);
  }

  static int rand_int(int a, int b) {
    return (int)(a + rd() % (b - a + 1));
  }

};

#endif //CODEBALL_CONSTANTS_H
