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
  static model::Action actions[3];
  static int global_id;
  static int id;
  static int my_id;

  static Plan best_plan[2];
  static int player_score[2];
  static int waiting_ticks;
  static double time_limit;
  static double half_time;

  static double sum_asserts_failed;
  static double asserts_failed_k;

  static bool tryInit(
      const model::Robot& _me,
      const model::Rules& _rules,
      const model::Game& _game) {
    global_id = _me.id;
    id = global_id % _rules.team_size;
    if (tick == _game.current_tick) {
      return false;
    }
    H::global_timer.start();
    game = _game;
    C::rules = _rules;
    tick = game.current_tick;
    actions[0] = actions[1] = actions[2] = model::Action();
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
        std::cout << int(sum_asserts_failed / asserts_failed_k) << " asserts failed" << std::endl;
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
    return actions[id];
  }

  static int getMyRobotGlobalIdByLocal(int id) {
    for (auto& robot : game.robots) {
      if (robot.is_teammate && robot.id % 2 == id) {
        return robot.id;
      }
    }
    return -1;
  }

  static MyTimer t[100];
  static MyTimer global_timer;
};

#endif //CODEBALL_HELPER_H
