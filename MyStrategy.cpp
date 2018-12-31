#ifdef LOCAL
#include <MyStrategy.h>
#include <Simulator.h>
#include <model/C.h>
#include <model/P.h>
#include <H.h>
#else
#include "MyStrategy.h"
#include "Simulator.h"
#include "model/C.h"
#include "model/P.h"
#include "H.h"
#endif

MyStrategy::MyStrategy() {}

void doStrategy() {
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
    H::best_plan[id].additional_jump = -1;
  }

  for (int id = 1; id >= 0; id--) {
    int iteration = 0;
/*    for (; H::global_timer.getCumulative(true) < H::time_limit; iteration++) {
      if (id == 1) {
        if (H::game.ball.z < -0.01) {
          if (H::global_timer.getCumulative(true) > H::half_time) {
            break;
          }
        } else {
          if (H::global_timer.cur() > 0.0015) {
            break;
          }
        }
      }*/
    for (;; iteration++) {
      if (H::game.ball.z < -0.01) {
        if (iteration >= 100) {
          break;
        }
      } else {
        if (id == 1 && iteration >= 10) {
          break;
        }
        if (id == 0 && iteration >= 190) {
          break;
        }
      }
      Plan cur_plan;
      if (iteration == 0) {
        cur_plan = H::best_plan[id];
      } else if (C::rand_double(0, 1) < 0.5) {
        cur_plan = H::best_plan[id];
        cur_plan.mutate();
      }
      Simulator simulator(H::game.robots, H::game.ball);

      double score = 0;
      double multiplier = 1.;
      for (int sim_tick = 0; sim_tick < C::MAX_SIMULATION_DEPTH; sim_tick++) {
        bool sbd_jump = false;
        for (auto& robot : simulator.robots) {
          if (robot.is_teammate) {
            if (robot.global_id % 2 == id) {
              robot.action = cur_plan.toMyAction(sim_tick);
              if (!cur_plan.was_on_ground_after_in_air_after_jumping && cur_plan.was_in_air_after_jumping && robot.touch) {
                cur_plan.was_on_ground_after_in_air_after_jumping = true;
#ifdef DEBUG
                if (id == 0 && iteration == 0) {
                  P::logn("cur_plan.was_on_ground_after_in_air_after_jumping = true; ", sim_tick);
                }
#endif
                if (!cur_plan.collide_with_ball_before_on_ground_after_jumping) {
                  cur_plan.time_jump = -1;
#ifdef DEBUG
                  if (id == 0 && iteration == 0) {
                    P::logn("cur_plan.time_jump = -1; ", sim_tick);
                  }
#endif
                }
              }
              if (!cur_plan.was_in_air_after_jumping && cur_plan.was_jumping && !robot.touch) {
                cur_plan.was_in_air_after_jumping = true;
#ifdef DEBUG
                if (id == 0 && iteration == 0) {
                  P::logn("cur_plan.was_in_air_after_jumping = true; ", sim_tick);
                }
#endif
              }
              if (robot.action.jump_speed > 0) {
                if (robot.touch) {
                  sbd_jump = true;
                  cur_plan.was_jumping = true;
#ifdef DEBUG
                  if (id == 0 && iteration == 0) {
                    P::logn("cur_plan.was_jumping = true; ", sim_tick);
                  }
#endif
                } else {
                  robot.action.jump_speed = 0;
                  cur_plan.time_jump = -1;
                }
              }
            } else {
              robot.action = H::best_plan[robot.global_id % 2].toMyAction(sim_tick);
              if (robot.action.jump_speed > 0) {
                if (robot.touch) {
                  sbd_jump = true;
                } else {
                  robot.action.jump_speed = 0;
                }
              }
            }
          }
        }
        simulator.tick(sbd_jump);
        bool needs_rollback = false;
        for (auto& robot : simulator.robots){
          if (robot.collide_with_ball_in_air) {
            needs_rollback = true;
            robot.action.jump_speed = C::rules.ROBOT_MAX_JUMP_SPEED;
          }
        }
        if (needs_rollback) {
          if (simulator.getMyRobotById(id).collide_with_ball_in_air && cur_plan.additional_jump == -1) {
            if (cur_plan.was_in_air_after_jumping) {
              cur_plan.collide_with_ball_before_on_ground_after_jumping = true;
#ifdef DEBUG
              if (id == 0 && iteration == 0) {
                P::logn("cur_plan.collide_with_ball_before_on_ground_after_jumping = true; ", sim_tick);
              }
#endif
            }
#ifdef DEBUG
            if (id == 0 && iteration == 0) {
              P::logn("BALL_COLLIDE: ", sim_tick);
            }
#endif
            cur_plan.additional_jump = sim_tick;
          }
          simulator.rollback();
          simulator.tick(true);
        }
        if (id == 0) {
          score += simulator.getScoreFighter() * multiplier;
        } else {
          score += simulator.getScoreDefender() * multiplier;
        }
        multiplier *= 0.9;
      }

      cur_plan.score = score;

#ifdef DEBUG
      for (auto& robot : simulator.robots) {
        if (robot.is_teammate && robot.global_id % 2 == id) {
          cur_plan.robot_trace = robot.trace;
        }
      }
      cur_plan.ball_trace = simulator.ball.trace;
#endif

      H::best_plan[id] = std::max(H::best_plan[id], cur_plan);
    }

    H::sum_bushes_near_the_road += iteration;
#ifdef DEBUG
    Plan accurate_plan;
    if (id == 0) {
      Simulator simulator(H::game.robots, H::game.ball);
      for (int sim_tick = 0; sim_tick < C::MAX_SIMULATION_DEPTH; sim_tick++) {
        for (auto& robot : simulator.robots) {
          if (robot.is_teammate) {
            if (robot.global_id % 2 == id) {
              robot.action = H::best_plan[id].toMyAction(sim_tick);
            } else {
              robot.action = H::best_plan[robot.global_id % 2].toMyAction(sim_tick);
            }
          }
        }
        simulator.tick(false, true);
      }
      for (auto& robot : simulator.robots) {
        if (robot.is_teammate && robot.global_id % 2 == id) {
          accurate_plan.robot_trace = robot.trace;
        }
      }
      accurate_plan.ball_trace = simulator.ball.trace;
    }
    if (id == 1) {
      /*for (int i = 1; i < H::best_plan[id].robot_trace.size(); i++) {
        P::drawLine(H::best_plan[id].robot_trace[i - 1], H::best_plan[id].robot_trace[i], 0xFF0000);
      }
      for (int i = 1; i < H::best_plan[id].ball_trace.size(); i++) {
        P::drawLine(H::best_plan[id].ball_trace[i - 1], H::best_plan[id].ball_trace[i], 0x00FF00);
      }
      for (int i = 1; i < accurate_plan.robot_trace.size(); i++) {
        P::drawLine(accurate_plan.robot_trace[i - 1], accurate_plan.robot_trace[i], 0xFFFF00);
      }
      for (int i = 1; i < accurate_plan.ball_trace.size(); i++) {
        P::drawLine(accurate_plan.ball_trace[i - 1], accurate_plan.ball_trace[i], 0x000000);
      }*/
    } else {
      for (int i = 1; i < H::best_plan[id].robot_trace.size(); i++) {
        P::drawLine(H::best_plan[id].robot_trace[i - 1], H::best_plan[id].robot_trace[i], 0xFF0000);
      }
      for (int i = 1; i < H::best_plan[id].ball_trace.size(); i++) {
        P::drawLine(H::best_plan[id].ball_trace[i - 1], H::best_plan[id].ball_trace[i], 0x00FF00);
      }
      for (int i = 1; i < accurate_plan.robot_trace.size(); i++) {
        P::drawLine(accurate_plan.robot_trace[i - 1], accurate_plan.robot_trace[i], 0x00FFFF);
      }
      for (int i = 1; i < accurate_plan.ball_trace.size(); i++) {
        P::drawLine(accurate_plan.ball_trace[i - 1], accurate_plan.ball_trace[i], 0xFFFFFF);
      }
    }
#endif
  }
  H::bushes_near_the_road_k += 1.;
  if ((H::tick + 1) % 2000 == 0) {
    std::cout << "Bushes near the road: " << H::sum_bushes_near_the_road / H::bushes_near_the_road_k << std::endl;
  }
  H::actions[0] = H::best_plan[0].toMyAction(0).toAction();
  H::actions[1] = H::best_plan[1].toMyAction(0).toAction();
#ifdef DEBUG
  /*P::logn(H::best_plan[0].additional_jump);
  P::logn(H::best_plan[1].additional_jump);

  P::logn(H::best_plan[0].toMyAction(0).target_velocity.x);
  P::logn(H::best_plan[0].toMyAction(0).target_velocity.y);
  P::logn(H::best_plan[0].toMyAction(0).target_velocity.z);
  P::logn(H::best_plan[0].toMyAction(0).jump_speed);
  P::logn(H::best_plan[0].toMyAction(0).target_velocity.length());
  P::logn(" ");

  Simulator simulator(H::game.robots, H::game.ball, true);
  for (auto& robot : simulator.robots) {
    if (robot.is_teammate) {
      robot.action = H::best_plan[robot.global_id % 2].toMyAction(0);
      robot.action = H::best_plan[robot.global_id % 2].toMyAction(0);
    }
  }
  simulator.tick(false);
  Simulator simulator2(H::game.robots, H::game.ball, true);
  for (auto& robot : simulator2.robots) {
    if (robot.is_teammate) {
      robot.action = H::best_plan[robot.global_id % 2].toMyAction(0);
      robot.action = H::best_plan[robot.global_id % 2].toMyAction(0);
    }
  }
  simulator2.tick(false, true);
  for (int i = 1; i < 2; ++i) {
    P::logn(simulator.robots[i].global_id, "pos: ");
    P::logn(simulator.robots[i].position.x - simulator2.robots[i].position.x);
    P::logn(simulator.robots[i].position.y - simulator2.robots[i].position.y);
    P::logn(simulator.robots[i].position.z - simulator2.robots[i].position.z);
    P::logn((simulator.robots[i].position - simulator2.robots[i].position).length());
    P::logn("vel:");
    P::logn(simulator.robots[i].velocity.x - simulator2.robots[i].velocity.x);
    P::logn(simulator.robots[i].velocity.y - simulator2.robots[i].velocity.y);
    P::logn(simulator.robots[i].velocity.z - simulator2.robots[i].velocity.z);
    P::logn((simulator.robots[i].velocity - simulator2.robots[i].velocity).length());
  }*/
#endif
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
