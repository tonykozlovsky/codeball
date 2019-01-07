#ifndef CODEBALL_SIMULATOR_H
#define CODEBALL_SIMULATOR_H

#ifdef LOCAL
#include <model/Entity.h>
#include <model/P.h>
#include <model/Dan.h>
#include <H.h>
#else
#include "model/Entity.h"
#include "model/P.h"
#include "model/Dan.h"
#include "H.h"
#endif

/*struct Simulator {

  Entity robots[4];
  Entity ball;
  bool my_goal = false;
  bool enemy_goal = false;
  bool collide_with_ball[10];

  double getScoreFighter() {
    double score = 0;
    double closest_enemy_dist = 1e9;
    for (auto& robot : robots) {
      if (robot.is_teammate && robot.id % 2 == 0) {
        score += -0.01 * (robot.position - ball.position).length();
        if (!robot.touch) {
          score -= 20;
        }
      }
      if (!robot.is_teammate) {
        closest_enemy_dist = std::min(closest_enemy_dist, (ball.position - robot.position).length());
      }
    }
    score += closest_enemy_dist;
    double dist = (ball.position - Point{0, 0, C::rules.arena.depth / 2 + C::rules.arena.goal_depth}).length();
    score += -dist * dist;
    return score;
  }

  double getScoreFighter1() {
    double d1 = (Point{-C::rules.arena.goal_width / 2 + 2, C::rules.arena.goal_height - 2, C::rules.arena.depth / 2 + 2} - ball.position).length();
    double d2 = (Point{0, C::rules.arena.goal_height - 2, C::rules.arena.depth / 2 + 2} - ball.position).length();
    double d3 = (Point{C::rules.arena.goal_width / 2 - 2, C::rules.arena.goal_height - 2, C::rules.arena.depth / 2 + 2} - ball.position).length();
    return C::rules.arena.depth + C::rules.arena.width + C::rules.arena.height - std::min(d1, std::min(d2, d3));
  }

  Entity& getMyRobotById(const int id) {
    if (H::my_id == 1) {
      return robots[1 - id];
    } else {
      return robots[3 - id];
    }
  }

  double getScoreDefender() {
    double score = 0;
    if (enemy_goal) {
      score -= 1e9;
    }
    double closest_enemy_dist = 1e9;
    for (auto& robot : robots) {
      if (robot.is_teammate && robot.id % 2 == 1) {
        double x = 0;
        score += -0.1 * (robot.position - Point{
            x, 0, -C::rules.arena.depth / 2 - 2}).length();
        if (!robot.touch) {
          score -= 0.1;
        }
      }
      if (!robot.is_teammate) {
        closest_enemy_dist = std::min(closest_enemy_dist, (ball.position - robot.position).length());
      }
    }
    double dist = (ball.position - Point{0, 0, -C::rules.arena.depth / 2 - C::rules.arena.goal_depth}).length();
    dist = std::min(dist, C::rules.arena.depth / 2 + C::rules.arena.goal_depth);
    score += dist;
    return score;
  }

  Simulator() {}

  Simulator(const std::vector<model::Robot>& _robots, const model::Ball& _ball, bool debug = false) {
    for (int i = 0; i < 4; i++) {
      robots[i] = Entity(_robots[i]);
      collide_with_ball[_robots[i].id] = false;
    }
    std::sort(robots, robots + 4);
    this->debug = debug;
    ball = Entity(_ball);
#ifdef DEBUG
    update_trace();
#endif
  }

  double length(const Point& p) {
    return p.length();
  }

  double length(const Point2d& p) {
    return p.length();
  }

  Point normalize(const Point& p) {
    return p.normalize();
  }

  Point2d normalize(const Point2d& p) {
    return p.normalize();
  }

  double dot(const Point& a, const Point& b) {
    return a.dot(b);
  }

  Point clamp(const Point& v, const double ub) {
    return v.clamp(ub);
  }

  int cur_microtick;

  bool collide_entities(Entity& a, Entity& b, bool check_with_ball = false) {
    const Point& delta_position = b.position - a.position;
    const double distance_sq = delta_position.length_sq();
    const double sum_r = a.radius + b.radius;
    if (check_with_ball && (3.05) * (3.05) > distance_sq) { // TODO FIX RADIUS
      a.collide_with_ball_in_air = true;
    }
    if (sum_r * sum_r > distance_sq) {
      any_triggers_fired = true;
      const double penetration = sum_r - sqrt(distance_sq);
      const double k_a = 1. / (a.mass * ((1 / a.mass) + (1 / b.mass)));
      const double k_b = 1. / (b.mass * ((1 / a.mass) + (1 / b.mass)));
      const Point& normal = normalize(delta_position);
      a.position -= normal * (penetration * k_a);
      b.position += normal * (penetration * k_b);
      const double delta_velocity = dot(b.velocity - a.velocity, normal)
          - (b.radius_change_speed + a.radius_change_speed);
      if (delta_velocity < 0) {
        const Point& impulse = normal * ((1. + (C::rules.MAX_HIT_E + C::rules.MIN_HIT_E) / 2.) * delta_velocity);
        a.velocity += impulse * k_a;
        b.velocity -= impulse * k_b;
        return true;
      }
    }
    return false;
  }

  bool collide_with_arena(Entity& e, Point& result) {
    const Dan& dan = Dan::dan_to_arena(e.position, e.radius);
    const double distance = dan.distance;
    const Point& normal = dan.normal;
    if (e.radius > distance) { // TODO distance sq
      const double penetration = e.radius - distance;
      e.position += normal * penetration;
      const double velocity = dot(e.velocity, normal) - e.radius_change_speed;
      if (velocity < 0) {
        e.velocity -= normal * ((1. + e.arena_e) * velocity);
        result = normal;
        return true;
      }
    }
    return false;
  }

  void move(Entity& e, const double delta_time, int number_of_microticks) {
    e.velocity = clamp(e.velocity, C::rules.MAX_ENTITY_SPEED);
    e.position += e.velocity * delta_time;

    if (number_of_microticks > 1) {
      const double coef = (1 - (number_of_microticks + 1) / 2. / number_of_microticks); // TODO check
      e.position -= e.acceleration * (coef * delta_time);
    }

    e.position.y -= C::rules.GRAVITY * delta_time * delta_time / 2;
    e.velocity.y -= C::rules.GRAVITY * delta_time;
  }

  bool debug = false;

  void update(const double delta_time, const int number_of_microticks) {
    for (auto& robot : robots) {
      robot.acceleration = {0, 0, 0};
      if (robot.touch) {
        const Point& target_velocity = robot.action.target_velocity - robot.touch_normal * robot.touch_normal.dot(robot.action.target_velocity);

        const Point& target_velocity_change = target_velocity - robot.velocity;

        double length = target_velocity_change.length_sq();
        if (length > 0) {
          const double acceleration = C::rules.ROBOT_ACCELERATION * fmax(0., robot.touch_normal.y);
          length = sqrt(length);
          if (acceleration * delta_time < length) {
            robot.acceleration = target_velocity_change * (acceleration * delta_time / length);
            robot.velocity += robot.acceleration;
          } else {
            robot.velocity += target_velocity_change;
          }
        }
      }

      move(robot, delta_time, number_of_microticks);

      robot.radius = C::rules.ROBOT_MIN_RADIUS + (C::rules.ROBOT_MAX_RADIUS - C::rules.ROBOT_MIN_RADIUS) * robot.action.jump_speed / C::rules.ROBOT_MAX_JUMP_SPEED;
      robot.radius_change_speed = robot.action.jump_speed;
    }

    move(ball, delta_time, number_of_microticks);

    Point collision_normal;
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < i; j++) {
        collide_entities(robots[i], robots[j]);
      }
    }

    for (auto& robot : robots) {
      collide_entities(robot, ball, !robot.touch);
      if (!collide_with_arena(robot, collision_normal)) {
        if (robot.touch) {
          any_triggers_fired = true;
        }
        robot.touch = false;
      } else {
        if (!robot.touch) {
          any_triggers_fired = true;
        }
        robot.touch = true;
        robot.touch_normal = collision_normal;
      }
    }

    if (!collide_with_arena(ball, collision_normal)) {
      if (ball.touch) {
        any_triggers_fired = true;
      }
      ball.touch = false;
    } else {
      if (!ball.touch) {
        any_triggers_fired = true;
      }
      ball.touch = true;
    }

    if (!my_goal && !enemy_goal) {
      if (ball.position.z > C::rules.arena.depth / 2 + ball.radius) {
        my_goal = true;
      } else if (-ball.position.z > C::rules.arena.depth / 2 + ball.radius) {
        enemy_goal = true;
      }
    }
  }

  bool any_triggers_fired = false;

  void clearTriggers() {
    any_triggers_fired = false;
  }

  void clearCollideWithBallInAir() {
    for (auto& robot : robots) {
      robot.collide_with_ball_in_air = false;
    }
  }

  void tick_microticks(int number_of_microticks) {
    clearTriggers();
    if (number_of_microticks == 0) {
      return;
    }
    update((double) number_of_microticks / C::rules.TICKS_PER_SECOND / C::rules.MICROTICKS_PER_TICK, number_of_microticks);
  }

  bool somebodyJumpThisTick() {
    for (auto& robot : robots) {
      if (robot.action.jump_speed > 0) {
        return true;
      }
    }
    return false;
  }

  void tick_tick() {
    clearCollideWithBallInAir();
    int remaining_microticks = 100;
    if (somebodyJumpThisTick()) {
      tick_microticks(1);
      tick_microticks(1);
      remaining_microticks = 98;
    }
    save();
    tick_microticks(remaining_microticks);
    if (any_triggers_fired) {
      int l = 0;
      int r = remaining_microticks;
      while (r - l > 1) {
        int mid = (r + l) / 2;
        rollback();
        tick_microticks(mid);
        if (any_triggers_fired) {
          r = mid;
        } else {
          l = mid;
        }
      }
      rollback();
      tick_microticks(l);
      tick_microticks(1);
      tick_microticks(remaining_microticks - l - 1);
    }
#ifdef DEBUG
    update_trace();
#endif
  }

#ifdef DEBUG
  void update_trace() {
    for (auto& robot : robots) {
      robot.trace.push_back(robot.position);
    }
    ball.trace.push_back(ball.position);
  }
#endif

#ifdef DEBUG
  void rollback_trace() {
    for (auto& robot : robots) {
      robot.trace.pop_back();
    }
    ball.trace.pop_back();
  }
#endif

  void rollback() {
    for (auto& robot : robots) {
      robot.roll_back();
    }
    ball.roll_back();
  }

  void save() {
    for (auto& robot : robots) {
      robot.save();
    }
    ball.save();
  }

  void tick_micro() {
    save();
    for (auto& robot : robots) {
      robot.collide_with_ball_in_air = false;
    }
    double delta_time = 1. / C::rules.TICKS_PER_SECOND;
    for (cur_microtick = 0; cur_microtick < C::rules.MICROTICKS_PER_TICK; cur_microtick++) {
      update(delta_time / C::rules.MICROTICKS_PER_TICK, 1);
    }
#ifdef DEBUG
    update_trace();
#endif
  }





  void test_random_points() {
    Point min_p{1e9, 1e9, 1e9}, max_p{-1e9, -1e9, -1e9};
    long long it = 0;
    for (;; it++) {
      Point p;
      p.x = C::rand_double(0, C::rules.arena.width / 2);
      p.y = C::rand_double(0, C::rules.arena.height);
      p.z = C::rand_double(0, C::rules.arena.depth / 2 + C::rules.arena.goal_depth);
      Dan d = dan_to_arena_quarter(p, C::rules.BALL_RADIUS);
      if (d.distance < C::rules.BALL_RADIUS) {
        min_p.x = std::min(min_p.x, p.x);
        min_p.y = std::min(min_p.y, p.y);
        min_p.z = std::min(min_p.z, p.z);
        max_p.x = std::max(max_p.x, p.x);
        max_p.y = std::max(max_p.y, p.y);
        max_p.z = std::max(max_p.z, p.z);
      }
      if (it % 10000000 == 0) {
        std::cout << std::endl;
        std::cout << min_p.x << " " << max_p.x << std::endl;
        std::cout << min_p.y << " " << max_p.y << std::endl;
        std::cout << min_p.z << " " << max_p.z << std::endl;
      }
    }
  }

};*/

#endif //CODEBALL_SIMULATOR_H
