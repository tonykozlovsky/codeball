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
    H::best_plan[id].collide_with_ball = false;
  }

  for (int id = 1; id >= 0; id--) {
    int iteration = 0;
    for (; H::global_timer.getCumulative(true) < H::time_limit; iteration++) {
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
      }
      Plan cur_plan;
      if (iteration == 0) {
        cur_plan = H::best_plan[id];
      }
      //else if (C::rand_int(0, 1) == 0) {
      //  cur_plan = H::best_plan[id];
      //  cur_plan.mutate();
      //}
      Simulator simulator(H::game.robots, H::game.ball);

      double score = 0;
      double multiplier = 1.;
      for (int sim_tick = 0; sim_tick < C::MAX_SIMULATION_DEPTH; sim_tick++) {
        for (auto& robot : simulator.robots) {
          if (robot.is_teammate) {
            if (robot.global_id % 2 == id) {
              robot.action = cur_plan.toMyAction(sim_tick);
            } else {
              robot.action = H::best_plan[robot.global_id % 2].toMyAction(sim_tick);
            }
          }
        }
        //H::t[0].start();
        simulator.tick();
        //H::t[0].cur(true);
        if (id == 0) {
          score += simulator.getScoreFighter() * multiplier;
        } else {
          score += simulator.getScoreDefender() * multiplier;
        }
        multiplier *= 0.9;
      }

      cur_plan.score = score;
      for (auto& robot : simulator.robots) {
        if (robot.is_teammate && robot.global_id % 2 == id) {
          cur_plan.collide_with_ball = simulator.collide_with_ball[robot.global_id];
          //cur_plan.robot_trace = robot.trace;
        }
      }
      //cur_plan.ball_trace = simulator.ball.trace;

      /*if (iteration == 0) {
        if (id == 0) {
          P::logn("Prev_Fight: ", cur_plan.score);
        } else {
          P::logn("Prev_Def: ", cur_plan.score);
        }
        P::logn("Prev_angle1: ", cur_plan.angle1);
        P::logn("Prev_angle2: ", cur_plan.angle2);
        P::logn("Prev_time_change: ", cur_plan.time_change);
        P::logn("Prev_time_jump: ", cur_plan.time_jump);
        P::logn("Prev_speed1: ", cur_plan.speed1);
        P::logn("Prev_speed2: ", cur_plan.speed2);
        P::logn("____________________________");

      }*/
      H::best_plan[id] = std::max(H::best_plan[id], cur_plan);

    }
    /*if (id == 0) {
      P::logn("Fight: ", H::best_plan[id].score);
      P::logn("it: ", iteration);

      P::logn("angle1: ", H::best_plan[id].angle1);
      P::logn("angle2: ", H::best_plan[id].angle2);
      P::logn("time_change: ", H::best_plan[id].time_change);
      P::logn("time_jump: ", H::best_plan[id].time_jump);
      P::logn("speed1: ", H::best_plan[id].speed1);
      P::logn("speed2: ", H::best_plan[id].speed2);
      P::logn("____________________________");

      for (int i = 1; i < H::best_plan[id].robot_trace.size(); i++) {
        P::drawLine(H::best_plan[id].robot_trace[i - 1], H::best_plan[id].robot_trace[i], 0xFF0000);
      }
      for (int i = 1; i < H::best_plan[id].ball_trace.size(); i++) {
        P::drawLine(H::best_plan[id].ball_trace[i - 1], H::best_plan[id].ball_trace[i], 0x00FF00);
      }

    } else {
      P::logn("Def: ", H::best_plan[id].score);
      P::logn("it: ", iteration);

      P::logn("angle1: ", H::best_plan[id].angle1);
      P::logn("angle2: ", H::best_plan[id].angle2);
      P::logn("time_change: ", H::best_plan[id].time_change);
      P::logn("time_jump: ", H::best_plan[id].time_jump);
      P::logn("speed1: ", H::best_plan[id].speed1);
      P::logn("speed2: ", H::best_plan[id].speed2);
      P::logn("____________________________");

      for (int i = 1; i < H::best_plan[id].robot_trace.size(); i++) {
        P::drawLine(H::best_plan[id].robot_trace[i - 1], H::best_plan[id].robot_trace[i], 0xFF0000);
      }
      for (int i = 1; i < H::best_plan[id].ball_trace.size(); i++) {
        P::drawLine(H::best_plan[id].ball_trace[i - 1], H::best_plan[id].ball_trace[i], 0x00FF00);
      }

    }*/

  }
  /*std::vector<std::pair<double, int> > timers;
  for (int i = 12; i <= 35; i++) {
    timers.push_back({(double)H::t[i].k / H::t[36].k, i});
  }
  std::sort(timers.begin(), timers.end());
  for (auto& t : timers) {
    P::logn("t" + std::to_string(t.second) + ": ", t.first * 100);
  }
  /*for (int i = 12; i <= 31; i++) {
    H::t[i].cur(false, true);
    P::logn("t" + std::to_string(i) + ": ", H::t[i].avg() * 1e3);
  }*/
  /*std::vector<std::pair<double, int> > timers;
  for (int i = 0; i <= 50; i++) {
    H::t[i].cur(false, true);
    timers.push_back({H::t[i].avg(), i});
  }
  std::sort(timers.rbegin(), timers.rend());
  int sum = 0;
  for (auto& t : timers) {
    sum += (int) (t.first * 1e3 / 21 * 100);
    P::logn("t" + std::to_string(t.second) + ": ", t.first * 1e3, "    ", (int) (t.first * 1e3 / 21 * 100), "%");
  }
  P::logn("sum: ", sum);*/


  H::actions[0] = H::best_plan[0].toMyAction(0, true).toAction();
  H::actions[1] = H::best_plan[1].toMyAction(0, true).toAction();
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
