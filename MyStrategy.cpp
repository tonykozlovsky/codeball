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

  if (H::tick % C::TPT == 0) {
    for (int id = 0; id < 2; id++) {
      H::best_plan[id].score.minimal();
      H::best_plan[id].time_jump--;
      if (H::best_plan[id].time_jump < 0) {
        H::best_plan[id].time_jump = 0;
      }
      H::best_plan[id].time_change--;
      if (H::best_plan[id].time_change < 0) {
        H::best_plan[id].time_change = 0;
      }
      H::best_plan[id].was_jumping = false;
      H::best_plan[id].was_in_air_after_jumping = false;
      H::best_plan[id].was_on_ground_after_in_air_after_jumping = false;
      H::best_plan[id].collide_with_ball_before_on_ground_after_jumping = false;
      H::best_plan[id].oncoming_jump = -1;
    }

    for (int id = 2; id < 4; ++id) {
      for (auto& robot : H::game.robots) {
        if (robot.id == H::getRobotGlobalIdByLocal(id)) {
          // todo LAST ACTION
          H::best_plan[id] = Plan(4, C::MAX_SIMULATION_DEPTH, atan2(robot.velocity_z, robot.velocity_x));
        }
      }
    }

    for (int id = 2; id < 4; ++id) {
      for (auto& robot : H::game.robots) {
        if (robot.id == H::getRobotGlobalIdByLocal(id)) {
          Point v0 = H::prev_velocity[id];
          Point v1 = {robot.velocity_x, robot.velocity_y, robot.velocity_z};
          double dvx = v1.x - v0.x;
          double dvz = v1.z - v0.z;
          double ax, az;
          if (H::solve(v0.x, v0.z, v1.x, v1.z, dvx, dvz, ax, az)) {
            H::best_plan[id] = Plan(4, C::MAX_SIMULATION_DEPTH, atan2(az, ax));
            // Point pos = {robot.x, robot.y, robot.z};
            // P::drawLine(pos, {pos.x + ax, pos.y, pos.z + az});
          }
        }
      }
    }


    for (int id = 1; id >= 0; id--) {
      int iteration = 0;
      SmartSimulator simulator(C::MAX_SIMULATION_DEPTH, H::getRobotGlobalIdByLocal(id), H::game.robots, H::game.ball, {}, false);
      int iterations[2] = {251, 251};
      int additional_iteration[2] = {250, 250};
      if (H::game.ball.z < -0.01) {
        iterations[0] = 476;
        iterations[1] = 26;
        additional_iteration[0] = 475;
        additional_iteration[1] = 25;
      }
      for (;; iteration++) {
        if (iteration > iterations[id]) {
          break;
        }

        Plan cur_plan(1, C::MAX_SIMULATION_DEPTH);
        if (iteration == 0 || iteration == additional_iteration[id]) {
          cur_plan = H::best_plan[id];
        } else if (C::rand_double(0, 1) < 1. / 10.) {
          cur_plan = H::best_plan[id];
          cur_plan.mutate(1, C::MAX_SIMULATION_DEPTH);
        }

        if (id == 0) {
          cur_plan.score.start_fighter();
        } else {
          cur_plan.score.start_defender();
        }

        simulator.initIteration(iteration, additional_iteration[id]);
        double multiplier = 1.;
        for (int sim_tick = 0; sim_tick < C::MAX_SIMULATION_DEPTH; sim_tick++) {
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
          int main_robot_additional_jump_type = simulator.tickDynamic(sim_tick, H::getRobotGlobalIdByLocal(0), false);
          if (main_robot_additional_jump_type > 0) { // 1 - with ball, 2 - additional
            if (main_robot_additional_jump_type == 1 && cur_plan.was_in_air_after_jumping) {
              cur_plan.collide_with_ball_before_on_ground_after_jumping = true;
            }
            if (cur_plan.oncoming_jump == -1) {
              cur_plan.oncoming_jump = sim_tick;
              cur_plan.oncoming_jump_speed = main_robot_additional_jump_type == 2 ?
                  std::max(C::MIN_WALL_JUMP, cur_plan.max_jump_speed) : cur_plan.max_jump_speed;
            }
          }

          if (id == 0) {
            cur_plan.score.sum_score += simulator.getSumScoreFighter(sim_tick) * multiplier;
            cur_plan.score.fighter_min_dist_to_ball = std::min(simulator.getMinDistToBallScoreFighter() * multiplier, cur_plan.score.fighter_min_dist_to_ball);
            cur_plan.score.fighter_min_dist_to_goal = std::min(simulator.getMinDistToGoalScoreFighter() * multiplier, cur_plan.score.fighter_min_dist_to_goal);
            if (sim_tick == C::MAX_SIMULATION_DEPTH - 1) {
              cur_plan.score.fighter_last_dist_to_goal = simulator.getMinDistToGoalScoreFighter();
            }
          } else if (id == 1) {
            cur_plan.score.sum_score += simulator.getSumScoreDefender(sim_tick) * multiplier;
            cur_plan.score.defender_min_dist_to_ball = std::min(simulator.getMinDistToBallScoreDefender() * multiplier, cur_plan.score.defender_min_dist_to_ball);
            cur_plan.score.defender_min_dist_from_goal = std::min(simulator.getMinDistFromGoalScoreDefender() * multiplier, cur_plan.score.defender_min_dist_from_goal);
            if (sim_tick == C::MAX_SIMULATION_DEPTH - 1) {
              cur_plan.score.defender_last_dist_from_goal = simulator.getMinDistFromGoalScoreDefender();
            }
          }
          multiplier *= 0.999;
        }

        if (cur_plan.was_in_air_after_jumping && !cur_plan.collide_with_ball_before_on_ground_after_jumping) {
          cur_plan.time_jump = -1;
        }
        if (cur_plan.oncoming_jump == -1) {
          cur_plan.oncoming_jump = cur_plan.time_jump;
          cur_plan.oncoming_jump_speed = cur_plan.max_jump_speed;
        } else if (cur_plan.time_jump != -1) {
          if (cur_plan.oncoming_jump > cur_plan.time_jump) {
            cur_plan.oncoming_jump = cur_plan.time_jump;
            cur_plan.oncoming_jump_speed = cur_plan.max_jump_speed;
          }
        }

        H::best_plan[id] = std::max(H::best_plan[id], cur_plan);
      }

      H::sum_asserts_failed += iteration;
#ifdef DEBUG
      if (id == 0) {

        P::logn("fighter score: ", H::best_plan[0].score.score());
        P::logn("sum_score: ", H::best_plan[0].score.sum_score);
        P::logn("fighter_min_dist_to_ball: ", -H::best_plan[0].score.fighter_min_dist_to_ball);
        P::logn("fighter_min_dist_to_goal: ", -H::best_plan[0].score.fighter_min_dist_to_goal);
        P::logn("fighter_last_dist_to_goal: ", -H::best_plan[0].score.fighter_last_dist_to_goal);
        P::logn("time_jump: ", H::best_plan[0].time_jump);
        P::logn("oncoming_jump: ", H::best_plan[0].oncoming_jump);
        P::logn("oncoming_jump_speed: ", H::best_plan[0].oncoming_jump_speed);
        P::logn("max_jump_speed: ", H::best_plan[0].max_jump_speed);
        P::logn("angle1: ", H::best_plan[0].angle1);
        P::logn("speed1: ", H::best_plan[0].speed1);
        P::logn("time_change: ", H::best_plan[0].time_change);
        P::logn("angle2: ", H::best_plan[0].angle2);
        P::logn("speed2: ", H::best_plan[0].speed2);
        P::logn("was_jumping: ", H::best_plan[0].was_jumping);
        P::logn("was_in_air_after_jumping: ", H::best_plan[0].was_in_air_after_jumping);
        P::logn("collide_with_ball_before_on_ground_after_jumping: ", H::best_plan[0].collide_with_ball_before_on_ground_after_jumping);
        P::logn("was_on_ground_after_in_air_after_jumping: ", H::best_plan[0].was_on_ground_after_in_air_after_jumping);
        P::logn("ajs: ", H::best_plan[0].toMyAction(0, false).toAction().jump_speed);


        Plan cur_plan = H::best_plan[0];

        SmartSimulator simulator(C::MAX_SIMULATION_DEPTH, H::getRobotGlobalIdByLocal(0), H::game.robots, H::game.ball, {}, false, H::getRobotGlobalIdByLocal(0));
        simulator.initIteration(iteration, additional_iteration[id]);

        SmartSimulator accurate_simulator(C::MAX_SIMULATION_DEPTH, H::getRobotGlobalIdByLocal(0), H::game.robots, H::game.ball, {}, true, H::getRobotGlobalIdByLocal(0));
        accurate_simulator.initIteration(iteration, additional_iteration[id]);

        for (int sim_tick = 0; sim_tick < C::MAX_SIMULATION_DEPTH; sim_tick++) {

          simulator.main_robot->action = cur_plan.toMyAction(sim_tick, true);

          simulator.tickDynamic(sim_tick, H::getRobotGlobalIdByLocal(0), true);

          accurate_simulator.main_robot->action = cur_plan.toMyAction(sim_tick, true);

          accurate_simulator.tickDynamic(sim_tick, H::getRobotGlobalIdByLocal(0), true);


          //P::logn("vel: ",
          //    simulator.main_robot->state.velocity.x - accurate_simulator.main_robot->state.velocity.x,
          //    " ", simulator.main_robot->state.velocity.y - accurate_simulator.main_robot->state.velocity.y,
          //    " ", simulator.main_robot->state.velocity.z - accurate_simulator.main_robot->state.velocity.z);
        }
      } else {
        /*P::logn("defender score: ", H::best_plan[1].score.score());
        P::logn("sum_score: ", H::best_plan[1].score.sum_score);
        P::logn("defender_min_dist_from_goal: ", H::best_plan[1].score.defender_min_dist_from_goal);
        P::logn("defender_last_dist_from_goal: ", H::best_plan[1].score.defender_last_dist_from_goal);
        P::logn("defender_min_dist_to_ball: ", -H::best_plan[1].score.defender_min_dist_to_ball);
        */
      }
#endif
    }
    H::asserts_failed_k += 1;
  }
#ifndef FROM_LOG
  H::actions[H::getRobotGlobalIdByLocal(0)] = H::best_plan[0].toMyAction(0, false).toAction();
  H::actions[H::getRobotGlobalIdByLocal(1)] = H::best_plan[1].toMyAction(0, false).toAction();
#endif

  for (auto& robot : H::game.robots) {
    int id = H::getRobotLocalIdByGlobal(robot.id);
    H::prev_velocity[id] = {robot.velocity_x, robot.velocity_y, robot.velocity_z};
  }

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

  if (H::tick % C::TPT == C::TPT - 1) {
    P::logs.clear();
    P::lines_to_draw.clear();
    P::spheres_to_draw.clear();
  }
  return buf.GetString();
}
#endif
#endif
