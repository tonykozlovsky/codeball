#ifdef LOCAL
#include <MyStrategy.h>
#include <model/C.h>
#include <model/P.h>
#include <H.h>
#include <SmartSimulator.h>
#else
#include "MyStrategy.h"
#include "SmartSimulator.h"
#include "model/C.h"
#include "model/P.h"
#include "H.h"
#endif

MyStrategy::MyStrategy() {}

void doStrategy() {
#ifdef FROM_LOG
  for (auto& robot: H::game.robots) {
    Entity e;
    e.fromRobot(robot);
    P::drawEntities({e.state}, 0, e.is_teammate ? 0x00FF00 : 0xFF0000);
  }
  Entity e;
  e.fromBall(H::game.ball);
  P::drawEntities({e.state}, 0, 0x333333);
#endif
  /*for (int i = 0; i < 50; ++i) {
    H::t[i].init_calls();
  }*/

  H::t[0].start();
  for (int id = 0; id < 2; id++) {
    H::best_plan[id].score = -1e18;
    H::best_plan[id].time_jump--;
    if (H::best_plan[id].time_jump < -1) {
      H::best_plan[id].time_jump = -1;
    }
    H::best_plan[id].time_change--;
    if (H::best_plan[id].time_change < -1) {
      H::best_plan[id].time_change = -1;
    }
    H::best_plan[id].was_jumping = false;
    H::best_plan[id].was_in_air_after_jumping = false;
    H::best_plan[id].was_on_ground_after_in_air_after_jumping = false;
    H::best_plan[id].collide_with_ball_before_on_ground_after_jumping = false;
    H::best_plan[id].oncoming_jump = -1;
  }

  for (int id = 1; id >= 0; id--) {
    int iteration = 0;
    SmartSimulator simulator(H::getMyRobotGlobalIdByLocal(id), H::game.robots, H::game.ball, {}, false, H::getMyRobotGlobalIdByLocal(0));
    /*for (; H::global_timer.getCumulative(true) < H::time_limit; iteration++) {
      if (id == 1) {
        if (H::game.ball.z < -0.01) {
          if (H::global_timer.getCumulative(true) > H::half_time) {
            break;
          }
        } else {
          if (H::global_timer.cur() > 0.002) {
            break;
          }
        }
      }*/
    int iterations[2] = {301, 301};
    int additional_iteration[2] = {300, 300};
    if (H::game.ball.z < -0.01) {
      iterations[0] = 551;
      iterations[1] = 51;
      additional_iteration[0] = 550;
      additional_iteration[1] = 50;
    }
    for (; ; iteration++) {
      if (iteration > iterations[id]) {
        break;
      }

      Plan cur_plan;
      if (iteration == 0 || iteration == additional_iteration[id]) {
        cur_plan = H::best_plan[id];
      } else if (iteration % 2 == 1) {
        cur_plan = H::best_plan[id];
        cur_plan.mutate();
      }

      simulator.initIteration(iteration);

      double score = 0;
      double score1 = 0;
      double multiplier = 1.;

      for (int sim_tick = 0; sim_tick < C::MAX_SIMULATION_DEPTH; sim_tick++) {
        for (int i = 0; i < simulator.dynamic_robots_size; ++i) {
          auto& robot = simulator.dynamic_robots[i];
          if (robot != simulator.main_robot && robot->is_teammate) {
            robot->action = H::best_plan[robot->id % 2].toMyAction(i, true);
          }
        }
        simulator.main_robot->action = cur_plan.toMyAction(sim_tick, true);
        if (!cur_plan.was_on_ground_after_in_air_after_jumping && cur_plan.was_in_air_after_jumping && simulator.main_robot->state.touch) {
          cur_plan.was_on_ground_after_in_air_after_jumping = true;
          if (!cur_plan.collide_with_ball_before_on_ground_after_jumping) {
            cur_plan.time_jump = -1;
          }
        }
        if (!cur_plan.was_in_air_after_jumping && cur_plan.was_jumping && !simulator.main_robot->state.touch) {
          cur_plan.was_in_air_after_jumping = true;
        }
        if (simulator.main_robot->action.jump_speed > 0 && simulator.main_robot->state.touch) {
          cur_plan.was_jumping = true;
        }

        int main_robot_additional_jump_type = simulator.tickDynamic(sim_tick, H::getMyRobotGlobalIdByLocal(0), iteration == additional_iteration[id]);

        if (main_robot_additional_jump_type > 0) { // 1 - with ball, 2 - additional
          if (main_robot_additional_jump_type == 1 && cur_plan.was_in_air_after_jumping) {
            cur_plan.collide_with_ball_before_on_ground_after_jumping = true;
          }
          if (cur_plan.oncoming_jump == -1) {
            cur_plan.oncoming_jump = sim_tick;
          }
        }
        if (id == 0) {
          score += simulator.getScoreFighter(sim_tick, false) * multiplier;
          score1 = std::max(simulator.getScoreFighter1(), score1);
        } else {
          score += simulator.getScoreDefender(sim_tick) * multiplier;
        }
        multiplier *= 0.9;
      }
      if (cur_plan.was_in_air_after_jumping && !cur_plan.collide_with_ball_before_on_ground_after_jumping) {
        cur_plan.time_jump = -1;
      }
      if (cur_plan.oncoming_jump == -1) {
        cur_plan.oncoming_jump = cur_plan.time_jump;
      } else if (cur_plan.time_jump != -1) {
        cur_plan.oncoming_jump = std::min(cur_plan.oncoming_jump, cur_plan.time_jump);
      }

      cur_plan.score = score + 100 * score1;

      H::best_plan[id] = std::max(H::best_plan[id], cur_plan);
    }

    H::sum_asserts_failed += iteration;
    /*if (id == 0) {

      P::logn("score: ", H::best_plan[0].score);
      P::logn("time_jump: ", H::best_plan[0].time_jump);
      P::logn("oncoming_jump: ", H::best_plan[0].oncoming_jump);
      P::logn("angle1: ", H::best_plan[0].angle1);
      P::logn("speed1: ", H::best_plan[0].speed1);
      P::logn("time_change: ", H::best_plan[0].time_change);
      P::logn("angle2: ", H::best_plan[0].angle2);
      P::logn("speed2: ", H::best_plan[0].speed2);
      P::logn("was_jumping: ", H::best_plan[0].was_jumping);
      P::logn("was_in_air_after_jumping: ", H::best_plan[0].was_in_air_after_jumping);
      P::logn("collide_with_ball_before_on_ground_after_jumping: ", H::best_plan[0].collide_with_ball_before_on_ground_after_jumping);
      P::logn("was_on_ground_after_in_air_after_jumping: ", H::best_plan[0].was_on_ground_after_in_air_after_jumping);

      SmartSimulator accurate_simulator(H::getMyRobotGlobalIdByLocal(0), H::game.robots, H::game.ball, H::game.nitro_packs, true);
      accurate_simulator.initIteration(400);

      Plan cur_plan = H::best_plan[0];
      for (int sim_tick = 0; sim_tick < C::MAX_SIMULATION_DEPTH; sim_tick++) {
        for (int i = 0; i < accurate_simulator.dynamic_robots_size; ++i) {
          auto& robot = accurate_simulator.dynamic_robots[i];
          if (robot != accurate_simulator.main_robot && robot->is_teammate) {
            robot->action = H::best_plan[robot->id % 2].toMyAction(i, true);
          }
        }
        accurate_simulator.main_robot->action = cur_plan.toMyAction(sim_tick, true);

        bool is_main_robot_collide_with_ball_in_air = accurate_simulator.tickDynamic(sim_tick, true);

      }
    }*/
  }

  H::asserts_failed_k += 1;

  H::actions[0] = H::best_plan[0].toMyAction(0, false).toAction();
  H::actions[1] = H::best_plan[1].toMyAction(0, false).toAction();

  /*for (int i = 0; i < 50; ++i) {
    H::t[i].capture();
    P::logn("t", i, ": ", H::t[i].avg_(), " ", H::t[i].last_());
  }*/
  H::t[0].cur(true, true);
  P::logn(H::t[0].avg());

}

void MyStrategy::act(
    const model::Robot& me,
    const model::Rules& rules,
    const model::Game& game,
    model::Action& action) {
  if (H::tryInit(me, rules, game)) {
    doStrategy();
    action = H::getCurrentAction();
  } else {
    action = H::getCurrentAction();
    H::global_timer.cur(true, true);
  }
}

#ifdef LOCAL
#ifdef DRAWLR

#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

std::string MyStrategy::custom_rendering() {
  rapidjson::Document document;
  document.SetArray();
  rapidjson::Document::AllocatorType& allocator = document.GetAllocator();
  for (auto line : P::lines_to_draw) {
    rapidjson::Value line_object;
    line_object.SetObject();
    rapidjson::Value line_data;
    line_data.SetObject();
    line_data.AddMember("x1", line.a.x, allocator);
    line_data.AddMember("y1", line.a.y, allocator);
    line_data.AddMember("z1", line.a.z, allocator);
    line_data.AddMember("x2", line.b.x, allocator);
    line_data.AddMember("y2", line.b.y, allocator);
    line_data.AddMember("z2", line.b.z, allocator);
    line_data.AddMember("width", 1.0, allocator);
    line_data.AddMember("r", line.getR(), allocator);
    line_data.AddMember("g", line.getG(), allocator);
    line_data.AddMember("b", line.getB(), allocator);
    line_data.AddMember("a", line.getA(), allocator);
    line_object.AddMember("Line", line_data, allocator);
    document.PushBack(line_object, allocator);
  }
  for (auto sphere : P::spheres_to_draw) {
    rapidjson::Value sphere_object;
    sphere_object.SetObject();
    rapidjson::Value sphere_data;
    sphere_data.SetObject();
    sphere_data.AddMember("x", sphere.center.x, allocator);
    sphere_data.AddMember("y", sphere.center.y, allocator);
    sphere_data.AddMember("z", sphere.center.z, allocator);
    sphere_data.AddMember("radius", sphere.radius, allocator);
    sphere_data.AddMember("r", sphere.getR(), allocator);
    sphere_data.AddMember("g", sphere.getG(), allocator);
    sphere_data.AddMember("b", sphere.getB(), allocator);
    sphere_data.AddMember("a", sphere.getA(), allocator);
    sphere_object.AddMember("Sphere", sphere_data, allocator);
    document.PushBack(sphere_object, allocator);
  }

  for (auto& log : P::logs) {
    rapidjson::Value log_object;
    log_object.SetObject();
    rapidjson::Value value;
    value.SetString(log.c_str(), allocator);
    log_object.AddMember("Text", value, allocator);
    document.PushBack(log_object, allocator);
  }

  rapidjson::StringBuffer buf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buf);
  document.Accept(writer);

  P::logs.clear();
  P::lines_to_draw.clear();
  P::spheres_to_draw.clear();
  return buf.GetString();
}
#endif
#endif
