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

void clearBestPlans() {
  for (int id = 0; id < 4; id++) {
    H::best_plan[id].score.minimal();
    H::best_plan[id].time_jump--;
    if (H::best_plan[id].time_jump < 0) { // todo -1 ?
      H::best_plan[id].time_jump = 0;
    }
    H::best_plan[id].time_change--;
    if (H::best_plan[id].time_change < 0) { // todo -1 ?
      H::best_plan[id].time_change = 0;
    }
    H::best_plan[id].time_nitro_on--;
    if (H::best_plan[id].time_nitro_on < 0) { // todo -1 ?
      H::best_plan[id].time_nitro_on = 0;
    }
    H::best_plan[id].time_nitro_off--;
    if (H::best_plan[id].time_nitro_off < 0) { // todo -1 ?
      H::best_plan[id].time_nitro_off = 0;
    }
    H::best_plan[id].was_jumping = false;
    H::best_plan[id].was_on_ground_after_jumping = false;
    H::best_plan[id].collide_with_entity_before_on_ground_after_jumping = false;
    H::best_plan[id].oncoming_jump = -1;
  }
}

void enemiesPrediction() {

  for (int id = 0; id < 4; ++id) {
    for (auto& robot : H::game.robots) {
      if (robot.id == H::getRobotGlobalIdByLocal(id)) {
        H::last_action_plan[id] = Plan(4, C::MAX_SIMULATION_DEPTH, robot.velocity_x, robot.velocity_z);
      }
    }
  }

  for (int id = 0; id < 4; ++id) {
    for (auto& robot : H::game.robots) {
      if (robot.id == H::getRobotGlobalIdByLocal(id)) {
        Point v0 = H::prev_velocity[id];
        Point v1 = {robot.velocity_x, robot.velocity_y, robot.velocity_z};
        double dvx = v1.x - v0.x;
        double dvz = v1.z - v0.z;
        double ax, az;
        if (H::solve(v0.x, v0.z, v1.x, v1.z, dvx, dvz, ax, az)) {
          H::last_action_plan[id] = Plan(4, C::MAX_SIMULATION_DEPTH, ax, az);
          // Point pos = {robot.x, robot.y, robot.z};
          // P::drawLine(pos, {pos.x + ax, pos.y, pos.z + az});
        }
      }
    }
  }

  const int enemy_depth = 50;

  for (int enemy_id : {2, 3}) {
    continue;
    SmartSimulator simulator(enemy_depth, H::getRobotGlobalIdByLocal(enemy_id), 3, H::game.robots, H::game.ball, {});

    for (int iteration = 0; iteration < 100; iteration++) {
      Plan cur_plan(2, enemy_depth);
      if (iteration == 0) {
        cur_plan = H::best_plan[enemy_id];
      } else if (C::rand_double(0, 1) < 1. / 10.) {
        cur_plan = H::best_plan[enemy_id];
        cur_plan.mutate(2, enemy_depth);
      }
      cur_plan.score.start_fighter();
      simulator.initIteration(iteration, cur_plan);

      cur_plan.plans_config = 3;
      double multiplier = 1.;
      for (int sim_tick = 0; sim_tick < enemy_depth; sim_tick++) {
        /*int cell_x = std::clamp((int) ((simulator.main_robot->state.position.x + 40.) / 2.), 1, 78);
        int cell_y = std::clamp((int) ((simulator.main_robot->state.position.y + 1.) / 2.), 1, 18);
        int cell_z = std::clamp((int) ((simulator.main_robot->state.position.z + 30.) / 2.), 1, 58);
        H::danger_grid[cell_x + 1][cell_y][cell_z][sim_tick]++;
        H::used_cells[H::used_cells_size++] = {cell_x + 1, cell_y, cell_z, sim_tick};
        H::danger_grid[cell_x][cell_y + 1][cell_z][sim_tick]++;
        H::used_cells[H::used_cells_size++] = {cell_x, cell_y + 1, cell_z, sim_tick};
        H::danger_grid[cell_x][cell_y][cell_z + 1][sim_tick]++;
        H::used_cells[H::used_cells_size++] = {cell_x, cell_y, cell_z + 1, sim_tick};

        H::danger_grid[cell_x - 1][cell_y][cell_z][sim_tick]++;
        H::used_cells[H::used_cells_size++] = {cell_x - 1, cell_y, cell_z, sim_tick};
        H::danger_grid[cell_x][cell_y - 1][cell_z][sim_tick]++;
        H::used_cells[H::used_cells_size++] = {cell_x, cell_y - 1, cell_z, sim_tick};
        H::danger_grid[cell_x][cell_y][cell_z - 1][sim_tick]++;
        H::used_cells[H::used_cells_size++] = {cell_x, cell_y, cell_z - 1, sim_tick};
        */
        simulator.tickDynamic(sim_tick);

        cur_plan.score.sum_score += simulator.getSumScoreEnemy(sim_tick) * multiplier;
        cur_plan.score.fighter_min_dist_to_ball = std::min(simulator.getMinDistToBallScoreEnemy() * multiplier, cur_plan.score.fighter_min_dist_to_ball);
        cur_plan.score.fighter_min_dist_to_goal = std::min(simulator.getMinDistToGoalScoreEnemy() * multiplier, cur_plan.score.fighter_min_dist_to_goal);
        if (sim_tick == enemy_depth - 1) {
          cur_plan.score.fighter_last_dist_to_goal = simulator.getMinDistToGoalScoreEnemy();
        }

        multiplier *= 0.999;
      }
      H::best_plan[enemy_id] = std::max(H::best_plan[enemy_id], cur_plan);
    }
    /*if (enemy_id == 3) {
      Plan cur_plan = H::best_plan[enemy_id];

      SmartSimulator simulator_(
          C::MAX_SIMULATION_DEPTH,
          H::getRobotGlobalIdByLocal(enemy_id),
          cur_plan.plans_config,
          H::game.robots,
          H::game.ball,
          {},
          false,
          H::getRobotGlobalIdByLocal(enemy_id));
      simulator_.initIteration(250, cur_plan);

      for (int sim_tick = 0; sim_tick < C::MAX_SIMULATION_DEPTH; sim_tick++) {
        simulator_.tickDynamic(sim_tick, H::getRobotGlobalIdByLocal(enemy_id), true);
      }
    }*/
  }
}

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


  //todo saving packs collisions

  //for (int i = 0; i < 100; ++i) {
  //  auto& t = H::t[i];
  //  t.clearCur();
  //}
  //for (int i = 0; i < 100; ++i) {
  //  auto& c = H::c[i];
  //  c.init_calls();
  //}
  //H::t[0].start();

  if (H::tick % C::TPT == 0) {

    clearBestPlans();

    enemiesPrediction();

    int iterations[2] = {250 + 1, 250 + 1};
    for (int id = 1; id >= 0; id--) {
      int iteration = 0;
      SmartSimulator simulator_smart(C::MAX_SIMULATION_DEPTH, H::getRobotGlobalIdByLocal(id), 1, H::game.robots, H::game.ball, H::game.nitro_packs);
      SmartSimulator simulator_stupid(C::MAX_SIMULATION_DEPTH, H::getRobotGlobalIdByLocal(id), 2, H::game.robots, H::game.ball, H::game.nitro_packs);

      if (id == 1) {
        bool ball_on_my_side = false;
        for (int i = 0; i < C::MAX_SIMULATION_DEPTH; ++i) {
          if (simulator_smart.ball->states[i].position.z < -0.01 || simulator_stupid.ball->states[i].position.z < -0.01) {
            ball_on_my_side = true;
          }
        }
        if (!ball_on_my_side) {
          iterations[0] = 475 + 1;
          iterations[1] = 25 + 1;
        }
      }
      for (;; iteration++) {
        if (iteration > iterations[id]) {
          break;
        }

        Plan cur_plan_smart(1, C::MAX_SIMULATION_DEPTH);
        if (iteration == 0) {
          cur_plan_smart = H::best_plan[id];
        } else if (C::rand_double(0, 1) < 1. / 2.) { // todo check coefficient
          cur_plan_smart = H::best_plan[id];
          cur_plan_smart.mutate(1, C::MAX_SIMULATION_DEPTH);
        }

        if (id == 0) {
          cur_plan_smart.score.start_fighter();
        } else {
          cur_plan_smart.score.start_defender();
        }

        simulator_smart.initIteration(iteration, cur_plan_smart);

        Plan cur_plan_stupid = cur_plan_smart;

        simulator_stupid.initIteration(iteration, cur_plan_stupid);

        cur_plan_smart.plans_config = 1;
        cur_plan_stupid.plans_config = 2;

        cur_plan_smart.score.sum_score += 1e18;
        for (int minimax = 1; minimax < 2; ++minimax) {
          auto& simulator = minimax == 0 ? simulator_smart : simulator_stupid;
          auto& cur_plan = minimax == 0 ? cur_plan_smart : cur_plan_stupid;
          double multiplier = 1.;
          for (int sim_tick = 0; sim_tick < C::MAX_SIMULATION_DEPTH; sim_tick++) {

            bool main_touch = simulator.main_robot->state.touch;

            int main_robot_additional_jump_type = simulator.tickDynamic(sim_tick, H::getRobotGlobalIdByLocal(0), false);

            if (main_robot_additional_jump_type == 0 && simulator.main_robot->action.jump_speed > 0 && main_touch) {
              cur_plan.was_jumping = true;
            }

            if (main_robot_additional_jump_type > 0) { // 1 - with ball, 2 - with entity, 3 - additional
              if ((main_robot_additional_jump_type == 1 || main_robot_additional_jump_type == 2) && cur_plan.was_jumping && !cur_plan.was_on_ground_after_jumping) {
                cur_plan.collide_with_entity_before_on_ground_after_jumping = true;
              }
              if (cur_plan.oncoming_jump == -1) {
                cur_plan.oncoming_jump = sim_tick;
                cur_plan.oncoming_jump_speed = main_robot_additional_jump_type == 3 ?
                    std::max(C::MIN_WALL_JUMP, cur_plan.max_jump_speed) : cur_plan.max_jump_speed;
              }
            }

            if (cur_plan.was_jumping && !cur_plan.was_on_ground_after_jumping && simulator.main_robot->state.touch) {
              cur_plan.was_on_ground_after_jumping = true;
              if (!cur_plan.collide_with_entity_before_on_ground_after_jumping) {
                cur_plan.score.minimal();
                break;
              }
            }

            if (id == 0) {
              cur_plan.score.sum_score += simulator.getSumScoreFighter(sim_tick) * multiplier;
              cur_plan.score.fighter_min_dist_to_ball = std::min(simulator.getMinDistToBallScoreFighter() * multiplier, cur_plan.score.fighter_min_dist_to_ball);
              cur_plan.score.fighter_min_dist_to_goal = std::min(simulator.getMinDistToGoalScoreFighter() * multiplier, cur_plan.score.fighter_min_dist_to_goal);
              cur_plan.score.min_dist_to_enemy = std::min(simulator.getMinDistToEnemyScore() * multiplier, cur_plan.score.min_dist_to_enemy);
              if (sim_tick == C::MAX_SIMULATION_DEPTH - 1) {
                cur_plan.score.fighter_last_dist_to_goal = simulator.getMinDistToGoalScoreFighter();
              }
            } else if (id == 1) {
              cur_plan.score.sum_score += simulator.getSumScoreDefender(sim_tick) * multiplier;
              cur_plan.score.defender_min_dist_to_ball = std::min(simulator.getMinDistToBallScoreDefender() * multiplier, cur_plan.score.defender_min_dist_to_ball);
              cur_plan.score.defender_min_dist_from_goal = std::min(simulator.getMinDistFromGoalScoreDefender() * multiplier, cur_plan.score.defender_min_dist_from_goal);
              cur_plan.score.min_dist_to_enemy = std::min(simulator.getMinDistToEnemyScore() * multiplier, cur_plan.score.min_dist_to_enemy);
              if (sim_tick == C::MAX_SIMULATION_DEPTH - 1) {
                cur_plan.score.defender_last_dist_from_goal = simulator.getMinDistFromGoalScoreDefender();
              }
            }
            multiplier *= 0.999;
          }

          if (cur_plan.was_jumping && !cur_plan.collide_with_entity_before_on_ground_after_jumping) {
            cur_plan.score.minimal();
          } else {


            if (cur_plan.oncoming_jump == -1) {
              cur_plan.oncoming_jump = cur_plan.time_jump;
              cur_plan.oncoming_jump_speed = cur_plan.max_jump_speed;
            } else if (cur_plan.time_jump != -1) {
              if (cur_plan.oncoming_jump > cur_plan.time_jump) {
                cur_plan.oncoming_jump = cur_plan.time_jump;
                cur_plan.oncoming_jump_speed = cur_plan.max_jump_speed;
              }
            }
          }
        }
        H::best_plan[id] = std::max(H::best_plan[id], std::min(cur_plan_smart, cur_plan_stupid));
      }

      H::sum_asserts_failed += iteration;
#ifdef DEBUG
      if (id == 0) {
        P::logn("best plan id: ", H::best_plan[id].unique_id);
        P::logn("fighter score: ", H::best_plan[id].score.score());
        P::logn("sum_score: ", H::best_plan[id].score.sum_score);
        P::logn("fighter_min_dist_to_ball: ", -H::best_plan[id].score.fighter_min_dist_to_ball);
        P::logn("fighter_min_dist_to_goal: ", -H::best_plan[id].score.fighter_min_dist_to_goal);
        P::logn("fighter_last_dist_to_goal: ", -H::best_plan[id].score.fighter_last_dist_to_goal);
        P::logn("min_dist_to_enemy: ", H::best_plan[id].score.min_dist_to_enemy);
        P::logn("time_jump: ", H::best_plan[id].time_jump);
        P::logn("oncoming_jump: ", H::best_plan[id].oncoming_jump);
        P::logn("oncoming_jump_speed: ", H::best_plan[id].oncoming_jump_speed);
        P::logn("max_jump_speed: ", H::best_plan[id].max_jump_speed);
        P::logn("angle1: ", H::best_plan[id].angle1);
        P::logn("speed1: ", H::best_plan[id].speed1);
        P::logn("time_change: ", H::best_plan[id].time_change);
        P::logn("angle2: ", H::best_plan[id].angle2);
        P::logn("speed2: ", H::best_plan[id].speed2);
        P::logn("was_jumping: ", H::best_plan[id].was_jumping);
        P::logn("was_jumping: ", H::best_plan[id].was_jumping);
        P::logn("collide_with_entity_before_on_ground_after_jumping: ", H::best_plan[id].collide_with_entity_before_on_ground_after_jumping);
        P::logn("was_on_ground_after_jumping: ", H::best_plan[id].was_on_ground_after_jumping);


        Plan cur_plan = H::best_plan[id];

        SmartSimulator simulator(C::MAX_SIMULATION_DEPTH, H::getRobotGlobalIdByLocal(id), cur_plan.plans_config, H::game.robots, H::game.ball, H::game.nitro_packs, false, H::getRobotGlobalIdByLocal(id));
        simulator.initIteration(iteration, cur_plan);

        SmartSimulator accurate_simulator(C::MAX_SIMULATION_DEPTH, H::getRobotGlobalIdByLocal(id), cur_plan.plans_config, H::game.robots, H::game.ball, H::game.nitro_packs, true, H::getRobotGlobalIdByLocal(id));
        accurate_simulator.initIteration(iteration, cur_plan);

        for (int sim_tick = 0; sim_tick < C::MAX_SIMULATION_DEPTH; sim_tick++) {
          simulator.tickDynamic(sim_tick, H::getRobotGlobalIdByLocal(id), true);
          accurate_simulator.tickDynamic(sim_tick, H::getRobotGlobalIdByLocal(id), true);
        }
      }
#endif
    }
    H::asserts_failed_k += 1;
  }


#ifndef FROM_LOG
  for (auto& robot : H::game.robots) {
    if (robot.is_teammate) {
      Entity e;
      e.fromRobot(robot);
      e.action = H::best_plan[H::getRobotLocalIdByGlobal(robot.id)].toMyAction(0, false, true);
      e.nitroCheck();
      if (!e.action.use_nitro) {
        e.action = H::best_plan[H::getRobotLocalIdByGlobal(robot.id)].toMyAction(0, false, false);
      }
      H::actions[robot.id] = e.action.toAction();
    }
  }
#endif

  for (auto& robot : H::game.robots) {
    int id = H::getRobotLocalIdByGlobal(robot.id);
    H::prev_velocity[id] = {robot.velocity_x, robot.velocity_y, robot.velocity_z};
  }

  //H::t[0].cur(true);
  //for (int i = 0; i < 7; ++i) {
  //  auto& t = H::t[i];
  //  t.cur(false, true);
  //  P::logn("t", i, " avg: ", t.avg() * 1000, " cur: ", t.getCur() * 1000, " x", (int)(std::floor(t.getCur() / t.avg() * 100)), "%");
  //}

  //for (int i = 0; i < 5; ++i) {
  //  auto& c = H::c[i];
  //  c.capture();
  //  P::logn("c", i, " avg: ", c.avg_(), " cur: ", c.last_(), " x", (int)(std::floor((double)c.last_() / c.avg_() * 100)), "%");
  //}

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
