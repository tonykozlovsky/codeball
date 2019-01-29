#ifndef CODEBALL_HELPER_H
#define CODEBALL_HELPER_H

#ifdef LOCAL
#include <model/C.h>
#include <model/Plan.h>
#else
#include "model/C.h"
#include "model/Plan.h"
#endif


struct H {
  static model::Game game;

  static int tick;
  static model::Action actions[7];
  static int global_id;
  static int my_id;

  static Point2d prev_last_action[6];
  static Plan last_best_plan[6];
  static Plan best_plan[6];
  static Plan last_action_plan[6];
  static int player_score[2];
  static int waiting_ticks;
  static double time_limit;
  static double cur_tick_remaining_time;

  static double sum_iterations;
  static double iterations_k;

  static int danger_grid[60][20][100][C::MAX_SIMULATION_DEPTH];
  static DGState used_cells[1000007];
  static int used_cells_size;

  static Point prev_velocity[7];
  static Point prev_position[7];

  enum ROLE {FIGHTER, SEMI, DEFENDER};

  static ROLE role[6];

  static bool tryInit(
      const model::Robot& _me,
      const model::Rules& _rules,
      const model::Game& _game) {
    global_id = _me.id;
    if (tick == _game.current_tick) {
      return false;
    }
    H::global_timer.start();
    game = _game;
    std::sort(game.robots.begin(), game.robots.end(), [](const auto& a, const auto& b) {
      return a.id < b.id;
    });
    C::rules = _rules;
    tick = game.current_tick;
    actions[1] = actions[2] = actions[3] = model::Action();
    if (tick == 0) { // init on tick 0
      for (auto& player : game.players) {
        if (player.me) {
          my_id = player.id;
          break;
        }
      }
      player_score[0] = player_score[1] = 0;
      waiting_ticks = 0;
      C::rd.seed(229);
    }
    for (auto& player : game.players) {
      if (player_score[player.id - 1] != player.score) {
        player_score[player.id - 1] = player.score;
        waiting_ticks = 119;
        std::cout << int(sum_iterations / iterations_k) << " ?_?" << std::endl;
      }
    }
    if (waiting_ticks > 0) {
      waiting_ticks--;
      return false;
    }
    //double time_per_tick = C::time_limit / 18000.;
    //double ticks_remaining = (18000 - tick);
    //double half_ticks_remaining = ticks_remaining / 2.;
    //double tick_end_balance = tick + half_ticks_remaining;
    //double time_end_balance = C::time_limit - time_per_tick * (ticks_remaining - half_ticks_remaining);

    //cur_tick_remaining_time = (time_end_balance - global_timer.getCumulative()) / half_ticks_remaining;
    cur_tick_remaining_time = (C::time_limit - global_timer.getCumulative()) / (18001. - tick);
    return true;
  }

  static model::Action getCurrentAction() {
    return actions[global_id];
  }

  static int getRobotGlobalIdByLocal(int id) {
    if (my_id == 1) {
      return id + 1;
    } else if (id < 3) {
      return id + 4;
    } else {
      return id - 2;
    }
  }

  static int getRobotLocalIdByGlobal(int id) {
    if (my_id == 1) {
      return id - 1;
    } else if (id <= 3) {
      return id + 2;
    } else {
      return id - 4;
    }
  }

  static bool solve(double v0x, double v0z, double v1x, double v1z, double dvx, double dvz, double& ax, double& az) {
    const double eps = 1e-9;
    if (dvx * dvx + dvz * dvz < eps) {
      ax = v1x;
      az = v1z;
      return true;
    }
    double c = v0x * dvz - v0z * dvx;
    double D = 4 * dvz * dvz * c * c - 4 * (dvx * dvx + dvz * dvz) * (c * c - 900 * dvx * dvx);
    if (D >= 0) {
      double x1 = (2 * dvz * c + sqrt(D)) / (2 * (dvx * dvx + dvz * dvz));
      double x2 = (2 * dvz * c - sqrt(D)) / (2 * (dvx * dvx + dvz * dvz));
      double y1 = 900 - x1 * x1;
      double y2 = 900 - x2 * x2;

      if (y1 >= 0) {
        y1 = sqrt(y1);
        if (dvz * x1 - c < 0) {
          y1 = -y1;
        }
        if (fabs((x1 - v0x) * dvz - (y1 - v0z) * dvx) < eps) {
          ax = x1;
          az = y1;
          return true;
        }
      }

      if (y2 >= 0) {
        y2 = sqrt(y2);
        if (dvz * x2 - c > 0) {
          y2 = -y2;
        }
        if (fabs((x2 - v0x) * dvz - (y2 - v0z) * dvx) < eps) {
          ax = x2;
          az = y2;
          return true;
        }
      }

    }
    return false;
  }

  template<typename T>
  static int sgn(T val) {
    return (T(0) < val) - (val < T(0));
  }

  static bool solve2(Point2d v11, Point2d v12, Point2d v21, Point2d v22, Point& crossing) {

    Point2d cut1 = v12 - v11;
    Point2d cut2 = v22 - v21;

    double z1 = cut1.x * (v21 - v11).y - cut1.y * (v21 - v11).x;
    double z2 = cut1.x * (v22 - v11).y - cut1.y * (v22 - v11).x;

    if (sgn(z1) == sgn(z2) || (z1 == 0) || (z2 == 0))
      return false;

    z1 = cut2.x * (v11 - v21).y - cut2.y * (v11 - v21).x;
    z2 = cut2.x * (v12 - v21).y - cut2.y * (v12 - v21).x;

    if (sgn(z1) == sgn(z2) || (z1 == 0) || (z2 == 0))
      return false;

    crossing.x = v11.x + cut1.x * fabs(z1) / fabs(z2 - z1);
    crossing.z = v11.y + cut1.y * fabs(z1) / fabs(z2 - z1);

    return true;

  }

  static MyTimer t[100];
  static MyTimer c[100];
  static MyTimer global_timer;
  static MyTimer cur_tick_timer;
};

#ifndef LOCAL
namespace Frozen {

struct H {
  static model::Game game;

  static int tick;
  static model::Action actions[7];
  static int global_id;
  static int my_id;

  static Point2d prev_last_action[6];
  static Plan last_best_plan[6];
  static Plan best_plan[6];
  static Plan last_action_plan[6];
  static int player_score[2];
  static int waiting_ticks;
  static double time_limit;
  static double half_time;

  static double sum_asserts_failed;
  static double asserts_failed_k;

  static int danger_grid[80][20][60][100];
  static DGState used_cells[1000007];
  static int used_cells_size;

  static Point prev_velocity[7];
  static Point prev_position[7];

  static bool tryInit(
      const model::Robot& _me,
      const model::Rules& _rules,
      const model::Game& _game) {
    global_id = _me.id;
    if (tick == _game.current_tick) {
      return false;
    }
    H::global_timer.start();
    game = _game;
    std::sort(game.robots.begin(), game.robots.end(), [](const auto& a, const auto& b) {
      return a.id < b.id;
    });
    C::rules = _rules;
    tick = game.current_tick;
    actions[1] = actions[2] = actions[3] = actions[4] = model::Action();
    if (tick == 0) { // init on tick 0
      for (auto& player : game.players) {
        if (player.me) {
          my_id = player.id;
          break;
        }
      }
      player_score[0] = player_score[1] = 0;
      waiting_ticks = 0;
      C::rd.seed(229);
    }
    for (auto& player : game.players) {
      if (player_score[player.id - 1] != player.score) {
        player_score[player.id - 1] = player.score;
        waiting_ticks = 119;
        std::cout << int(sum_asserts_failed / asserts_failed_k) << " ?_?" << std::endl;
      }
    }
    if (waiting_ticks > 0) {
      waiting_ticks--;
      return false;
    }
    double time_per_tick = C::time_limit / 18000.;
    double ticks_remaining = (18000 - tick);
    double half_ticks_remaining = ticks_remaining / 50.;
    double tick_end_balance = tick + half_ticks_remaining;
    double time_end_balance = C::time_limit - time_per_tick * (ticks_remaining - half_ticks_remaining);

    half_time = (time_end_balance - global_timer.getCumulative()) / half_ticks_remaining / 2;
    //std::cout << "time: " << std::fixed << std::setprecision(3) << half_time * 2000 << std::endl;
    time_limit = global_timer.getCumulative() + half_time * 2;
    //std::cout << "time_limit: " << std::fixed << std::setprecision(3) << time_limit << std::endl;
    //std::cout << "global_timer: " << std::fixed << std::setprecision(3) <<  global_timer.getCumulative() << std::endl;
    half_time = global_timer.getCumulative() + half_time;
    return true;
  }

  static model::Action getCurrentAction() {
    return actions[global_id];
  }

  static int getRobotGlobalIdByLocal(int id) {
    if (my_id == 1) {
      return id + 1;
    } else if (id < 2) {
      return id + 3;
    } else {
      return id - 1;
    }
  }

  static int getRobotLocalIdByGlobal(int id) {
    if (my_id == 1) {
      return id - 1;
    } else if (id <= 2) {
      return id + 1;
    } else {
      return id - 3;
    }
  }

  static bool solve(double v0x, double v0z, double v1x, double v1z, double dvx, double dvz, double& ax, double& az) {
    const double eps = 1e-9;
    if (dvx * dvx + dvz * dvz < eps) {
      ax = v1x;
      az = v1z;
      return true;
    }
    double c = v0x * dvz - v0z * dvx;
    double D = 4 * dvz * dvz * c * c - 4 * (dvx * dvx + dvz * dvz) * (c * c - 900 * dvx * dvx);
    if (D >= 0) {
      double x1 = (2 * dvz * c + sqrt(D)) / (2 * (dvx * dvx + dvz * dvz));
      double x2 = (2 * dvz * c - sqrt(D)) / (2 * (dvx * dvx + dvz * dvz));
      double y1 = 900 - x1 * x1;
      double y2 = 900 - x2 * x2;

      if (y1 >= 0) {
        y1 = sqrt(y1);
        if (dvz * x1 - c < 0) {
          y1 = -y1;
        }
        if (fabs((x1 - v0x) * dvz - (y1 - v0z) * dvx) < eps) {
          ax = x1;
          az = y1;
          return true;
        }
      }

      if (y2 >= 0) {
        y2 = sqrt(y2);
        if (dvz * x2 - c > 0) {
          y2 = -y2;
        }
        if (fabs((x2 - v0x) * dvz - (y2 - v0z) * dvx) < eps) {
          ax = x2;
          az = y2;
          return true;
        }
      }

    }
    return false;
  }

  template<typename T>
  static int sgn(T val) {
    return (T(0) < val) - (val < T(0));
  }

  static bool solve2(Point2d v11, Point2d v12, Point2d v21, Point2d v22, Point& crossing) {

    Point2d cut1 = v12 - v11;
    Point2d cut2 = v22 - v21;

    double z1 = cut1.x * (v21 - v11).y - cut1.y * (v21 - v11).x;
    double z2 = cut1.x * (v22 - v11).y - cut1.y * (v22 - v11).x;

    if (sgn(z1) == sgn(z2) || (z1 == 0) || (z2 == 0)) // Отсекаем также и пограничные случаи
      return false;

    z1 = cut2.x * (v11 - v21).y - cut2.y * (v11 - v21).x;
    z2 = cut2.x * (v12 - v21).y - cut2.y * (v12 - v21).x;

    if (sgn(z1) == sgn(z2) || (z1 == 0) || (z2 == 0)) // Отсекаем также и пограничные случаи
      return false;

    crossing.x = v11.x + cut1.x * fabs(z1) / fabs(z2 - z1);
    crossing.z = v11.y + cut1.y * fabs(z1) / fabs(z2 - z1);

    return true;

  }

  static MyTimer t[100];
  static MyTimer c[100];
  static MyTimer global_timer;
};

}
#endif
#endif //CODEBALL_HELPER_H
