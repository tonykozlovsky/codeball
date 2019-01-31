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

void clearBestPlans() {
  for (int id = 0; id < 3; id++) {
    H::best_plan[id].clearAndShift(C::MAX_SIMULATION_DEPTH);
  }
  for (int id = 3; id < 6; id++) {
    H::best_plan[id].clearAndShift(C::ENEMY_SIMULATION_DEPTH);
  }
}

void addCell(int x, int y, int z, int t) {
  if (x < 0 || y < 0 || z < 0 || t < 0) {
    return;
  }
  H::danger_grid[x][y][z][t]++;
  if (H::danger_grid[x][y][z][t] == 1) {
    H::used_cells[H::used_cells_size++] = {x, y, z, t};
  }
}

void addCell(int x, int y, int z, int t, int tpt) {
  for (int i = 0; i < tpt; ++i) {
    addCell(x, y, z, t * tpt + i);
  }
}

int enemiesPrediction() {

  for (int id = 0; id < 6; ++id) {
    for (auto& robot : H::game.robots) {
      if (robot.id == H::getRobotGlobalIdByLocal(id)) {
        static constexpr double jr = 0.0033333333333333333333333333333;
        double jump_speed = 0;
        if (!robot.touch) {
          jump_speed = (robot.radius - 1) / jr;
        }
        Entity e;
        e.fromRobot(robot);
        Entity ball;
        ball.fromBall(H::game.ball);
        bool is_dribler = (e.state.position - ball.state.position).length() < 3.05;
        H::last_action_plan[id] = Plan(71, C::MAX_SIMULATION_DEPTH, robot.velocity_x, robot.velocity_z, 0, 0, {0, 0, 0}, jump_speed, is_dribler);
      }
    }
  }

  //H::t[2].start();
  for (int id = 0; id < 6; ++id) {
    for (auto& robot : H::game.robots) {
      if (robot.id == H::getRobotGlobalIdByLocal(id)) {
        Point v0 = H::prev_velocity[id];
        Point v1 = {robot.velocity_x, robot.velocity_y, robot.velocity_z};
        double dvx = (v1.x - v0.x);
        double dvy = (v1.y - v0.y);
        double dvz = (v1.z - v0.z);
        double ax, az;
        static constexpr double jr = 0.0033333333333333333333333333333;
        double jump_speed = 0;
        if (!robot.touch) {
          jump_speed = (robot.radius - 1) / jr;
        }
        Entity e;
        e.fromRobot(robot);
        Entity ball;
        ball.fromBall(H::game.ball);
        bool is_dribler = (e.state.position - ball.state.position).length() < 3.05;
        if (!robot.touch && !robot.is_teammate) {
          bool using_nitro = true;
          double eps = 1e-9;
          if (fabs(dvx) < eps && fabs(dvz) < eps && fabs(dvy - (-1./2)) < eps) {
            using_nitro = false;
          }
          if (using_nitro) {
            //P::logn(dvy);
            double min_error = 1e9;
            Point best;
            for (double x = -1; x < 1; x += 0.01) {
              for (double y = -1; y < 1; y += 0.01) {
                if (x * x + y * y > 1) {
                  continue;
                }
                for (double z : {sqrt(1 - (x * x + y * y)), -sqrt(1 - (x * x + y * y))}) {
                  //P::logn(x * x + y * y + z * z);
                  Point target_velocity{x, y, z};
                  target_velocity = target_velocity.normalize() * 100;
                  const auto& target_velocity_change = target_velocity - v0;
                  const auto& tvc_length_sq = target_velocity_change.length_sq();
                  if (tvc_length_sq > 0) {
                    const auto& ac_per_dt = C::rules.ROBOT_NITRO_ACCELERATION / 60.;
                    const auto& robot_acceleration = target_velocity_change * (ac_per_dt / sqrt(tvc_length_sq));
                    Point velocity = v0 + robot_acceleration;
                    velocity.y -= 1. / 2.;
                    double cur_error = (velocity - v1).length_sq();
                    if (cur_error < min_error) {
                      min_error = cur_error;
                      best = target_velocity;
                    }
                  }
                }
              }
            }
            if (min_error < 1e9) {
              //P::logn("me: ", min_error);
              //P::logn("acc: ", (v1 - v0).length());
              Point p0 = H::prev_position[id];
              Point p1 = {robot.x, robot.y, robot.z};
              //P::drawLine(p1, p1 + best, 0xFFF000);
              //P::logn(best.x - robot.velocity_x, " ", best.y - robot.velocity_y, " ",  best.z - robot.velocity_z);
              //P::logn(best.x, " ",  best.y, " ",  best.z);
              H::last_action_plan[id] = Plan(72, C::MAX_SIMULATION_DEPTH, 0, 0, 0, 0, best, jump_speed, is_dribler);
              /*Point target_velocity = best;
              target_velocity = target_velocity.normalize() * 100;
              const auto& target_velocity_change = target_velocity - v0;
              const auto& tvc_length_sq = target_velocity_change.length_sq();
              if (tvc_length_sq > 0) {
                const auto& ac_per_dt = C::rules.ROBOT_NITRO_ACCELERATION / 60.;
                const auto& robot_acceleration = target_velocity_change * (ac_per_dt / sqrt(tvc_length_sq));
                P::logn(robot_acceleration.x, " ", robot_acceleration.y," ",  robot_acceleration.z);
                Point velocity = v0 + robot_acceleration;
                velocity.y -= 1. / 2.;
                double cur_error = (velocity - v1).length_sq();
                if (cur_error < min_error) {
                  min_error = cur_error;
                  best = target_velocity;
                }
              }*/

            }
          }
        } else if (H::solve(v0.x, v0.z, v1.x, v1.z, dvx, dvz, ax, az)) {

          Point2d cur{ax, az};
          Point2d prev = H::prev_last_action[H::getRobotLocalIdByGlobal(robot.id)];

          Point p0 = H::prev_position[id];
          Point p1 = {robot.x, robot.y, robot.z};
          //P::drawLine(p1, {p1.x + dvx * 30, 1, p1.z + dvz * 30}, 0x0000FF);
          Point crossing;
          //P::drawLine({p0.x, 1, p0.z}, {p0.x + prev.x, 1, p0.z + prev.y}, 0xFF0000);
          //P::drawLine({p1.x, 1, p1.z}, {p1.x + cur.x, 1, p1.z + cur.y}, 0xFF0000);
          if (0 && H::solve2(
              {p0.x, p0.z},
              {p0.x + prev.x, p0.z + prev.y},
              {p1.x, p1.z},
              {p1.x + cur.x, p1.z + cur.y},
              crossing)) {
            //P::logn("kek");
            //P::drawLine(p0, crossing, 0xFF0000);
            //P::drawLine(p1, crossing, 0xFF0000);
            H::last_action_plan[id] = Plan(5, C::MAX_SIMULATION_DEPTH, ax, az, crossing.x, crossing.z);
          } else {
            H::last_action_plan[id] = Plan(71, C::MAX_SIMULATION_DEPTH, ax, az, 0, 0, {0, 0, 0}, jump_speed, is_dribler);
          }
          //if (!robot.is_teammate) {
          //  P::logn(robot.id, " ", sqrt(ax * ax + az * az));
          //  P::drawLine(p1, {p1.x + ax, p1.y, p1.z + az}, 0xFFFFFF);
          //}

          H::prev_last_action[H::getRobotLocalIdByGlobal(robot.id)] = {ax, az};

        }
      }
    }
  }
  //H::t[2].cur(true);
  for (int i = 0; i < H::used_cells_size; ++i) {
    const auto& cell = H::used_cells[i];
    H::danger_grid[cell.x][cell.y][cell.z][cell.t] = 0;

    //P::drawSphere({cell.x * 2 + 1 - 30, cell.y * 2 + 1, cell.z * 2 + 1 - 50}, 1, 0x00AA00);

  }
  H::used_cells_size = 0;

  int min_time_for_enemy_to_hit_the_ball = C::NEVER;

  H::t[1].start();
  for (int enemy_id : {3, 4, 5}) {
    SmartSimulator simulator(true, C::TPT, C::ENEMY_SIMULATION_DEPTH, H::getRobotGlobalIdByLocal(enemy_id), 3, H::game.robots, H::game.ball, {});
    for (int iteration = 0; iteration < 200; iteration++) {
      Plan cur_plan(61, C::ENEMY_SIMULATION_DEPTH);
      if (iteration == 0) {
        cur_plan = H::best_plan[enemy_id];
      }
      cur_plan.score.start_fighter();
      simulator.initIteration(iteration, cur_plan);

      cur_plan.plans_config = 3;
      //double multiplier = 1.;
      bool main_fly_on_prefix = !(simulator.main_robot->state.touch && simulator.main_robot->state.touch_surface_id == 1);
      for (int sim_tick = 0; sim_tick < C::ENEMY_SIMULATION_DEPTH; sim_tick++) {
        simulator.tickDynamic(sim_tick);
        main_fly_on_prefix &= !(simulator.main_robot->state.touch && simulator.main_robot->state.touch_surface_id == 1);

        double x = simulator.main_robot->state.position.x + 30.;
        double y = simulator.main_robot->state.position.y;
        double z = simulator.main_robot->state.position.z + 50.;
        int cell_x = (int) (x / 2.);
        int cell_y = (int) (y / 2.);

        int cell_z = (int) (z / 2.);

        addCell(cell_x + 1, cell_y, cell_z, sim_tick, 1);
        addCell(cell_x, cell_y + 1, cell_z, sim_tick, 1);
        addCell(cell_x, cell_y, cell_z + 1, sim_tick, 1);
        addCell(cell_x - 1, cell_y, cell_z, sim_tick, 1);
        addCell(cell_x, cell_y - 1, cell_z, sim_tick, 1);
        addCell(cell_x, cell_y, cell_z - 1, sim_tick, 1);

        if (!main_fly_on_prefix && simulator.main_robot->collide_with_ball) {
          min_time_for_enemy_to_hit_the_ball = std::min(min_time_for_enemy_to_hit_the_ball, sim_tick);
        }

        /*
        cur_plan.score.sum_score += simulator.getSumScoreEnemy(sim_tick) * multiplier;
        cur_plan.score.fighter_min_dist_to_ball = std::min(simulator.getMinDistToBallScoreEnemy() * multiplier, cur_plan.score.fighter_min_dist_to_ball);
        cur_plan.score.fighter_min_dist_to_goal = std::min(simulator.getMinDistToGoalScoreEnemy() * multiplier, cur_plan.score.fighter_min_dist_to_goal);
        if (sim_tick == enemy_depth - 1) {

          cur_plan.score.fighter_last_dist_to_goal = simulator.getMinDistToGoalScoreEnemy();
        }*/

        //multiplier *= 0.999;
      }
      //H::best_plan[enemy_id] = std::max(H::best_plan[enemy_id], cur_plan);
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
  H::t[1].cur(true, true);
  P::logn(H::t[1].avg());
  P::logn("mtfethtb: ", min_time_for_enemy_to_hit_the_ball);
  return min_time_for_enemy_to_hit_the_ball;
}

void updateRoles() {

  int closest_to_goal;
  double closest_distance_to_goal = 1e9;
  for (auto& robot : H::game.robots) {
    if (robot.is_teammate) {
      Entity e;
      e.fromRobot(robot);
      double dist = (Point{0, 1, -42} - e.state.position).length();
      if (dist < closest_distance_to_goal) {
        closest_distance_to_goal = dist;
        closest_to_goal = robot.id;
      }
    }
  }

  int closest_to_ball;
  double closest_distance_to_ball = 1e9;
  Entity ball;
  ball.fromBall(H::game.ball);
  for (auto& robot : H::game.robots) {
    if (robot.is_teammate) {
      Entity e;
      e.fromRobot(robot);
      double dist = (Point{0,
          1,
          42} - e.state.position).length();
      if (robot.id != closest_to_goal && dist < closest_distance_to_ball) {
        closest_distance_to_ball = dist;
        closest_to_ball = robot.id;
      }
    }
  }

  int other;
  for (auto& robot : H::game.robots) {
    if (robot.is_teammate) {
      if (robot.id != closest_to_ball && robot.id != closest_to_goal) {
        other = robot.id;
      }
    }
  }
  H::role[H::getRobotLocalIdByGlobal(closest_to_goal)] = H::DEFENDER;
  //P::logn("def: ", closest_to_goal);
  H::role[H::getRobotLocalIdByGlobal(closest_to_ball)] = H::FIGHTER;
  //P::logn("fi: ", closest_to_ball);
  H::role[H::getRobotLocalIdByGlobal(other)] = H::SEMI;
  //P::logn("semi: ", other);
}

void doStrategy() {
#ifdef FROM_LOG
  for (auto& robot: H::game.robots) {
    Entity e;
    e.fromRobot(robot);
    P::drawEntities({e.state}, 0, e.is_teammate ? 0x00FF00 : 0xFF0000);
  }
  for (auto& nitro_pack : H::game.nitro_packs) {
    Entity e;
    e.fromPack(nitro_pack);
    if (e.state.alive) {
      P::drawEntities({e.state}, 0, 0x0000FF);
    }
  }
  Entity e;
  e.fromBall(H::game.ball);
  P::drawEntities({e.state}, 0, 0x333333);
#endif


  //todo saving packs collisions


#ifdef LOCAL
  H::cur_tick_timer.start();
#endif

  if (H::tick % C::TPT == 0) {

    for (int i = 0; i < 100; ++i) {
      auto& t = H::t[i];
      t.clearCur();
    }
    for (int i = 0; i < 100; ++i) {
      auto& c = H::c[i];
      c.init_calls();
    }

    H::t[0].start();

    updateRoles();

    clearBestPlans();

    int min_time_for_enemy_to_hit_the_ball = enemiesPrediction();

    int iterations[3] = {200 * 2, 200 * 2, 200 * 2};
    //P::logn(H::cur_tick_remaining_time);
    //double available_time[3] = {0, 0, 0};
    //double available_time_prefix[3] = {H::global_timer.getCumulative() + H::cur_tick_remaining_time / 3, H::global_timer.getCumulative() + 2 * H::cur_tick_remaining_time / 3, H::global_timer.getCumulative() + H::cur_tick_remaining_time};
    for (int id = 0; id < 3; id++) {
      int iteration = 0;
      SmartSimulator simulator(false, C::TPT, C::MAX_SIMULATION_DEPTH, H::getRobotGlobalIdByLocal(id), 2, H::game.robots, H::game.ball, H::game.nitro_packs);

      bool ball_on_my_side = false;
      if (id == 0) {
        for (int i = 0; i < C::MAX_SIMULATION_DEPTH; ++i) {
          if (simulator.ball->states[i].position.z < -0.01) {
            ball_on_my_side = true;
          }
        }
        if (!ball_on_my_side) {
          for (int i = 0; i < 3; ++i) {
            if (H::role[i] == H::DEFENDER) {
              //available_time[i] = 0.1 * H::cur_tick_remaining_time;
              iterations[i] = 50 * 2;
            } else {
              iterations[i] = 275 * 2;
              //available_time[i] = 0.45 * H::cur_tick_remaining_time;
            }
          }
          //for (int i = 0; i < 3; ++i) {
          //  available_time_prefix[i] = i == 0 ? H::global_timer.getCumulative() + available_time[i] : available_time[i] + available_time_prefix[i - 1];
          //}
        }
      }
      for (;; iteration++) {
        if (iteration > iterations[id]) {
          break;
        }
      //for (; H::global_timer.getCumulative(true) < available_time_prefix[id]; iteration++) {
        int plan_type;
        double rd = C::rand_double(0, 1);

        if (simulator.main_robot->state.touch) {
          if (rd < 1./ 7) {
            plan_type = 20;
          } else if (rd < 2. / 7) {
            plan_type = 21;
          } else if (rd < 3. / 7) {
            plan_type = 22;
          } else if (rd < 4. / 7) {
            plan_type = 23;
          } else if (rd < 5. / 7) {
            plan_type = 11;
          } else if (rd < 6. / 7) {
            plan_type = 12;
          } else {
            plan_type = 11;
          }
        } else {
          if (rd < 0.5) {
            plan_type = 31;
          } else {
            plan_type = 32;
          }
        }
        //if (rd < 0.25) {
        //  plan_type = 21;
        //} else if (rd < 0.25 + 0.25) {
        //  plan_type = 11;
        //} else if (rd < 0.25 + 0.25 + 0.25) {
        //  plan_type = 12;
        //} else {
        //  plan_type = 13;
        //}

        Plan cur_plan(plan_type, C::MAX_SIMULATION_DEPTH);
        if (iteration == 0) {
          cur_plan = H::best_plan[id];
        } else if (C::rand_double(0, 1) < 1. / 10.) { // todo check coefficient
          cur_plan = H::best_plan[id];
          cur_plan.mutate(cur_plan.configuration, C::MAX_SIMULATION_DEPTH);
        }

        if (H::role[id] == H::FIGHTER) {
          cur_plan.score.start_fighter();
        } else if (H::role[id] == H::SEMI) {
          cur_plan.score.start_fighter();
        } else if (H::role[id] == H::DEFENDER) {
          cur_plan.score.start_defender();
        }

        simulator.initIteration(iteration, cur_plan);

        cur_plan.plans_config = 2;

        double multiplier = 1.;

        bool collide_with_smth = false;
        bool fly_on_prefix = (!simulator.main_robot->state.touch || simulator.main_robot->state.touch_surface_id != 1);

        for (int sim_tick = 0; sim_tick < C::MAX_SIMULATION_DEPTH; sim_tick++) {

          bool main_touch = simulator.main_robot->state.touch || simulator.main_robot->state.position.y < C::NITRO_TOUCH_EPSILON;

          int main_robot_additional_jump_type = simulator.tickDynamic(sim_tick, H::getRobotGlobalIdByLocal(0), false);

          fly_on_prefix &= (!simulator.main_robot->state.touch || simulator.main_robot->state.touch_surface_id != 1);

          if (main_robot_additional_jump_type == 0 && simulator.main_robot->action.jump_speed > 0 && main_touch) {
            cur_plan.was_jumping = true;
          }

          if (main_robot_additional_jump_type > 0) { // 1 - with ball, 2 - with entity, 3 - additional
            if (fly_on_prefix &&
              (main_robot_additional_jump_type == 1 || main_robot_additional_jump_type == 2)) {
              collide_with_smth = true;
            }
            if ((main_robot_additional_jump_type == 1 || main_robot_additional_jump_type == 2)
                && cur_plan.was_jumping
                && !cur_plan.was_on_ground_after_jumping) {
              cur_plan.collide_with_entity_before_on_ground_after_jumping = true;
              if (H::role[id] == H::DEFENDER && main_robot_additional_jump_type == 1
                  && min_time_for_enemy_to_hit_the_ball < sim_tick
                  && cur_plan.time_jump <= min_time_for_enemy_to_hit_the_ball) {
                cur_plan.score.minimal();
                break;
              }
            }
            if (cur_plan.oncoming_jump == C::NEVER) {
              cur_plan.oncoming_jump = sim_tick;
              cur_plan.oncoming_jump_speed = main_robot_additional_jump_type == 3 ?
                  std::max(C::MIN_WALL_JUMP, cur_plan.max_jump_speed) : cur_plan.max_jump_speed;
            }
          }

          if (cur_plan.was_jumping && !cur_plan.was_on_ground_after_jumping && simulator.main_robot->state.touch) {
            cur_plan.was_on_ground_after_jumping = true;
            if (!cur_plan.collide_with_entity_before_on_ground_after_jumping) {
              cur_plan.time_jump = C::NEVER;
            }
          }

          if (H::role[id] == H::FIGHTER) {
            cur_plan.score.sum_score += simulator.getSumScoreFighter(sim_tick) * multiplier;
            cur_plan.score.fighter_min_dist_to_ball = std::min(simulator.getMinDistToBallScoreFighter() * multiplier, cur_plan.score.fighter_min_dist_to_ball);
            cur_plan.score.fighter_min_dist_to_goal = std::min(simulator.getMinDistToGoalScoreFighter() * multiplier, cur_plan.score.fighter_min_dist_to_goal);
            if (sim_tick == C::MAX_SIMULATION_DEPTH - 1) {
              cur_plan.score.fighter_last_dist_to_goal = simulator.getMinDistToGoalScoreFighter();
            }
          } else if (H::role[id] == H::DEFENDER) {
            cur_plan.score.sum_score += simulator.getSumScoreDefender(sim_tick) * multiplier;
            cur_plan.score.defender_min_dist_to_ball = std::min(simulator.getMinDistToBallScoreDefender() * multiplier, cur_plan.score.defender_min_dist_to_ball);
            cur_plan.score.defender_min_dist_from_goal = std::min(simulator.getMinDistFromGoalScoreDefender() * multiplier, cur_plan.score.defender_min_dist_from_goal);
            if (sim_tick == C::MAX_SIMULATION_DEPTH - 1) {
              cur_plan.score.defender_last_dist_from_goal = simulator.getMinDistFromGoalScoreDefender();
            }
          } else if (H::role[id] == H::SEMI) {
            cur_plan.score.sum_score += simulator.getSumScoreFighter(sim_tick) * multiplier;
            cur_plan.score.fighter_min_dist_to_ball = std::min(simulator.getMinDistToBallScoreFighter() * multiplier, cur_plan.score.fighter_min_dist_to_ball);
            cur_plan.score.fighter_min_dist_to_goal = std::min(simulator.getMinDistToGoalScoreFighter() * multiplier, cur_plan.score.fighter_min_dist_to_goal);
            if (sim_tick == C::MAX_SIMULATION_DEPTH - 1) {
              cur_plan.score.fighter_last_dist_to_goal = simulator.getMinDistToGoalScoreFighter();
            }
          }

          multiplier *= 0.999;
        }

        if (!collide_with_smth) {
          cur_plan.time_nitro_on = C::NEVER;
          cur_plan.time_nitro_off = C::NEVER;
        }

        if (!cur_plan.was_jumping || (cur_plan.was_jumping && !cur_plan.collide_with_entity_before_on_ground_after_jumping)) {
          cur_plan.time_jump = C::NEVER;
        } else { 

          if (cur_plan.oncoming_jump == C::NEVER) {
            cur_plan.oncoming_jump = cur_plan.time_jump;
            cur_plan.oncoming_jump_speed = cur_plan.max_jump_speed;
          } else if (cur_plan.time_jump != C::NEVER) {
            if (cur_plan.oncoming_jump > cur_plan.time_jump) {
              cur_plan.oncoming_jump = cur_plan.time_jump;
              cur_plan.oncoming_jump_speed = cur_plan.max_jump_speed;
            }
          }
        }

        H::best_plan[id] = std::max(H::best_plan[id], cur_plan);
      }

      H::sum_iterations += iteration;
#ifdef DEBUG
      if (id == 0) {
        P::logn("best plan id: ", H::best_plan[id].unique_id);
        P::logn("fighter score: ", H::best_plan[id].score.score());
        P::logn("sum_score: ", H::best_plan[id].score.sum_score);
        P::logn("fighter_min_dist_to_ball: ", -H::best_plan[id].score.fighter_min_dist_to_ball);
        P::logn("fighter_min_dist_to_goal: ", -H::best_plan[id].score.fighter_min_dist_to_goal);
        P::logn("fighter_last_dist_to_goal: ", -H::best_plan[id].score.fighter_last_dist_to_goal);
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

        SmartSimulator simulator(false, C::TPT, C::MAX_SIMULATION_DEPTH, H::getRobotGlobalIdByLocal(id), cur_plan.plans_config, H::game.robots, H::game.ball, H::game.nitro_packs, false, H::getRobotGlobalIdByLocal(id));
        simulator.initIteration(iteration, cur_plan);

        SmartSimulator accurate_simulator(false, C::TPT, C::MAX_SIMULATION_DEPTH, H::getRobotGlobalIdByLocal(id), cur_plan.plans_config, H::game.robots, H::game.ball, H::game.nitro_packs, true, H::getRobotGlobalIdByLocal(id));
        accurate_simulator.initIteration(iteration, cur_plan);

        for (int sim_tick = 0; sim_tick < C::MAX_SIMULATION_DEPTH; sim_tick++) {
          simulator.tickDynamic(sim_tick, H::getRobotGlobalIdByLocal(id), true);
          /*if (sim_tick < C::ENEMY_SIMULATION_DEPTH) {
            const int cell_x = std::clamp((int) ((simulator.ball->getState().position.x + 30. - 1.) / 2.), 0, 58);
            const int cell_y = std::clamp((int) ((simulator.ball->getState().position.y - 1.) / 2.), 0, 18);
            const int cell_z = std::clamp((int) ((simulator.ball->getState().position.z + 50. - 1.) / 2.), 0, 98);
            const int sum = H::danger_grid[cell_x][cell_y][cell_z][sim_tick]
                + H::danger_grid[cell_x + 1][cell_y][cell_z][sim_tick]
                + H::danger_grid[cell_x][cell_y + 1][cell_z][sim_tick]
                + H::danger_grid[cell_x][cell_y][cell_z + 1][sim_tick]
                + H::danger_grid[cell_x + 1][cell_y + 1][cell_z][sim_tick]
                + H::danger_grid[cell_x + 1][cell_y][cell_z + 1][sim_tick]
                + H::danger_grid[cell_x][cell_y + 1][cell_z + 1][sim_tick]
                + H::danger_grid[cell_x + 1][cell_y + 1][cell_z + 1][sim_tick];
            if (sum > 0) {
              P::drawEntities(simulator.ball->getState());
            }
          }*/
          accurate_simulator.tickDynamic(sim_tick, H::getRobotGlobalIdByLocal(id), true);
        }
      }
#endif
    }
    H::iterations_k += 1;

#ifndef FROM_LOG
    for (auto& robot : H::game.robots) {
      if (robot.is_teammate) {
        H::best_plan_type[H::best_plan[H::getRobotLocalIdByGlobal(robot.id)].configuration]++;
        Entity e;
        e.fromRobot(robot);
        Entity ball;
        ball.fromBall(H::game.ball);
        //P::logn(robot.id, ": ", (e.state.position - ball.state.position).length() - (e.state.radius + ball.state.radius));

        if (!robot.touch) {
          e.action = H::best_plan[H::getRobotLocalIdByGlobal(robot.id)].toMyAction(0, false, true, e.state.position, e.state.velocity);
          e.nitroCheck();
          if (!e.action.use_nitro) {
            e.action = H::best_plan[H::getRobotLocalIdByGlobal(robot.id)].toMyAction(0, false, false, e.state.position, e.state.velocity);
          }
        } else {
          e.action = H::best_plan[H::getRobotLocalIdByGlobal(robot.id)].toMyAction(0, false, false, e.state.position, e.state.velocity);
        }
        H::actions[robot.id] = e.action.toAction();
      }
    }
    for (auto& it : H::best_plan_type) {
      P::logn(it.first, " ", it.second);
    }
#endif

    H::t[0].cur(true);
    for (int i = 0; i < 5; ++i) {
      auto& t = H::t[i];
      t.cur(false, true);
      P::logn("t", i, " avg: ", t.avg() * 1000, " cur: ", t.getCur() * 1000, " x", (int)(std::floor(t.getCur() / t.avg() * 100)), "%");
    }

    for (int i = 0; i < 5; ++i) {
      auto& c = H::c[i];
      c.capture();
      P::logn("c", i, " avg: ", c.avg_(), " cur: ", c.last_(), " x", (int)(std::floor((double)c.last_() / c.avg_() * 100)), "%");
    }
  }

  for (auto& robot : H::game.robots) {
    int id = H::getRobotLocalIdByGlobal(robot.id);
    H::prev_velocity[id] = {robot.velocity_x, robot.velocity_y, robot.velocity_z};
    H::prev_position[id] = {robot.x, robot.y, robot.z};
  }

#ifdef LOCAL
  H::cur_tick_timer.cur(true, true);
  std::cerr << H::cur_tick_timer.avg() << std::endl;
#endif
}

MyStrategy::MyStrategy() {}

#ifndef LOCAL
namespace Frozen {
void clearBestPlans() {
  for (int id = 0; id < 2; id++) {
    H::best_plan[id].clearAndShift(C::MAX_SIMULATION_DEPTH);
  }
  for (int id = 2; id < 4; id++) {
    H::best_plan[id].clearAndShift(C::ENEMY_SIMULATION_DEPTH);
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

          Point2d cur{ax, az};
          Point2d prev = H::prev_last_action[H::getRobotLocalIdByGlobal(robot.id)];

          Point p0 = H::prev_position[id];
          Point p1 = {robot.x, robot.y, robot.z};
          //P::drawLine(p1, {p1.x + dvx * 30, 1, p1.z + dvz * 30}, 0x0000FF);
          Point crossing;
          //P::drawLine({p0.x, 1, p0.z}, {p0.x + prev.x, 1, p0.z + prev.y}, 0xFF0000);
          //P::drawLine({p1.x, 1, p1.z}, {p1.x + cur.x, 1, p1.z + cur.y}, 0xFF0000);
          if (0 && H::solve2(
              {p0.x, p0.z},
              {p0.x + prev.x, p0.z + prev.y},
              {p1.x, p1.z},
              {p1.x + cur.x, p1.z + cur.y},
              crossing)) {
            //P::logn("kek");
            //P::drawLine(p0, crossing, 0xFF0000);
            //P::drawLine(p1, crossing, 0xFF0000);
            H::last_action_plan[id] = Plan(5, C::MAX_SIMULATION_DEPTH, ax, az, crossing.x, crossing.z);
          } else {

            H::last_action_plan[id] = Plan(4, C::MAX_SIMULATION_DEPTH, ax, az);

          }
          //if (!robot.is_teammate) {
          //  P::logn(robot.id, " ", sqrt(ax * ax + az * az));
          //  P::drawLine(p1, {p1.x + ax, p1.y, p1.z + az}, 0xFFFFFF);
          //}

          H::prev_last_action[H::getRobotLocalIdByGlobal(robot.id)] = {ax, az};

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
  for (auto& nitro_pack : H::game.nitro_packs) {
    Entity e;
    e.fromPack(nitro_pack);
    P::drawEntities({e.state}, 0, 0x0000FF);
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

      bool ball_on_my_side = false;
      if (id == 1) {
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

      for (; H::global_timer.getCumulative(true) < H::time_limit; iteration++) {
        if (id == 1) {
          if (ball_on_my_side) {
            if (H::global_timer.getCumulative(true) > H::half_time) {
              break;
            }
          } else {
            if (H::global_timer.cur() > 0.002) {
              break;
            }
          }
        }
        /*for (;; iteration++) {
          if (iteration > iterations[id]) {
            break;
          }*/
        Plan cur_plan_smart(1, C::MAX_SIMULATION_DEPTH);
        if (iteration == 0) {
          cur_plan_smart = H::best_plan[id];
        } else if (C::rand_double(0, 1) < 1. / 10.) { // todo check coefficient
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
              if (cur_plan.oncoming_jump == C::NEVER) {
                cur_plan.oncoming_jump = sim_tick;
                cur_plan.oncoming_jump_speed = main_robot_additional_jump_type == 3 ?
                    std::max(C::MIN_WALL_JUMP, cur_plan.max_jump_speed) : cur_plan.max_jump_speed;
              }
            }

            if (cur_plan.was_jumping && !cur_plan.was_on_ground_after_jumping && simulator.main_robot->state.touch) {
              cur_plan.was_on_ground_after_jumping = true;
              if (!cur_plan.collide_with_entity_before_on_ground_after_jumping) {
                cur_plan.time_jump = C::NEVER;
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

          if (cur_plan.was_jumping && !cur_plan.collide_with_entity_before_on_ground_after_jumping) {
            cur_plan.time_jump = C::NEVER;
          } else {

            if (cur_plan.oncoming_jump == C::NEVER) {
              cur_plan.oncoming_jump = cur_plan.time_jump;
              cur_plan.oncoming_jump_speed = cur_plan.max_jump_speed;
            } else if (cur_plan.time_jump != C::NEVER) {
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
      e.action = H::best_plan[H::getRobotLocalIdByGlobal(robot.id)].toMyAction(0, false, true, e.state.position);
      e.nitroCheck();
      if (!e.action.use_nitro) {
        e.action = H::best_plan[H::getRobotLocalIdByGlobal(robot.id)].toMyAction(0, false, false, e.state.position);
      }
      H::actions[robot.id] = e.action.toAction();
    }
  }
#endif

  for (auto& robot : H::game.robots) {
    int id = H::getRobotLocalIdByGlobal(robot.id);
    H::prev_velocity[id] = {robot.velocity_x, robot.velocity_y, robot.velocity_z};
    H::prev_position[id] = {robot.x, robot.y, robot.z};
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
}
#endif
void MyStrategy::act(
    const model::Robot& me,
    const model::Rules& rules,
    const model::Game& game,
    model::Action& action) {
#ifndef LOCAL
  if (rules.team_size == 3) {
    if (H::tryInit(me, rules, game)) {
      doStrategy();
      action = H::getCurrentAction();
    } else {
      action = H::getCurrentAction();
      H::global_timer.cur(true, true);
    }
  } else {
      if (Frozen::H::tryInit(me, rules, game)) {
      Frozen::doStrategy();
      action = Frozen::H::getCurrentAction();
    } else {
      action = Frozen::H::getCurrentAction();
      Frozen::H::global_timer.cur(true, true);
    }
  }
#else
  if (H::tryInit(me, rules, game)) {
    doStrategy();
    action = H::getCurrentAction();
  } else {
    action = H::getCurrentAction();
    H::global_timer.cur(true, true);
  }
#endif
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
  //if (H::tick == 3900) {
    return buf.GetString();
  //} else {
  //  return {};
  //}
}

#endif
#endif
