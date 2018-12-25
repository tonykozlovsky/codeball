#ifndef CODEBALL_SIMULATOR_H
#define CODEBALL_SIMULATOR_H

#ifdef LOCAL
#include <model/Entity.h>
#include <model/P.h>
#include <H.h>
#else
#include "model/Entity.h"
#include "model/Painter.h"
#include "H.h"
#endif

struct Simulator {

  struct Dan {
    double distance;
    Point normal;
    bool operator<(const Dan& other) const {
      return distance < other.distance;
    }
  };

  Entity robots[4];
  Entity ball;
  bool my_goal = false;
  bool enemy_goal = false;
  bool collide_with_ball[10];

  double getScoreFighter() {
    if (my_goal) {
      return 1e9;
    }
    if (enemy_goal) {
      return -1e9;
    }
    return -(ball.position - Point{0, 0,
        C::rules.arena.depth / 2
            + C::rules.arena.goal_depth}).length();
  }

  double getScoreDefender() {
    if (enemy_goal) {
      return -1e9;
    }
    double score = 0;
    for (auto& robot : robots) {
      if (robot.is_teammate && robot.global_id % 2 == 1) {
        score += -(robot.position - Point{
            0, 0, -C::rules.arena.depth / 2}).length();
      }
    }
    score += -2 * std::min(ball.position.z, 0.) * std::min(ball.position.z, 0.);
    return score;
  }

  Simulator() {}

  Simulator(const std::vector<model::Robot>& _robots, const model::Ball& _ball) {
    for (int i = 0; i < 4; i++) {
      robots[i] = Entity(_robots[i]);
      collide_with_ball[_robots[i].id] = false;
    }
    ball = Entity(_ball);
    update_trace();
  }

  void print_velocity(const Point& p) {
    std::cerr << "velocity: " << std::fixed << std::setprecision(30) << p.length() << std::endl;
    std::cerr << "x: " << std::fixed << std::setprecision(30) << p.x << std::endl;
    std::cerr << "y: " << std::fixed << std::setprecision(30) << p.y << std::endl;
    std::cerr << "z: " << std::fixed << std::setprecision(30) << p.z << std::endl;
  }

  void print_error(const Point& p1, const Point& p2) {
    std::cerr << "d: " << std::fixed << std::setprecision(30) << (p1 - p2).length() << std::endl;
    std::cerr << "x: " << std::fixed << std::setprecision(30) << fabs(p1.x - p2.x) << std::endl;
    std::cerr << "y: " << std::fixed << std::setprecision(30) << fabs(p1.y - p2.y) << std::endl;
    std::cerr << "z: " << std::fixed << std::setprecision(30) << fabs(p1.z - p2.z) << std::endl;
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

  bool collide_entities(Entity& a, Entity& b) {
    const Point& delta_position = b.position - a.position;
    const double distance_sq = delta_position.length_sq();
    if ( (a.radius + b.radius) * (a.radius + b.radius) > distance_sq) {
      const double penetration = a.radius + b.radius - sqrt(distance_sq);
      const double k_a = 1. / (a.mass * ((1 / a.mass) + (1 / b.mass)));
      const double k_b = 1. / (b.mass * ((1 / a.mass) + (1 / b.mass)));
      const Point& normal = normalize(delta_position);
      a.position -= normal * (penetration * k_a);
      b.position += normal * (penetration * k_b);
      const double delta_velocity = dot(b.velocity - a.velocity, normal)
          + (b.radius_change_speed - a.radius_change_speed);
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
    //H::t[10].start();
    const Dan& dan = dan_to_arena(e.position, e.radius);
    //H::t[10].cur(true);
    //H::t[11].start();
    const double distance = dan.distance;
    const Point& normal = dan.normal;
    if (e.radius > distance ) { // TODO distance sq
      const double penetration = e.radius - distance;
      e.position += normal * penetration;
      const double velocity = dot(e.velocity, normal) - e.radius_change_speed;
      if (velocity < 0) {
        e.velocity -= normal * ((1. + e.arena_e) * velocity);
        result = normal;
        //H::t[11].cur(true);
        return true;
      }
    }
    //H::t[11].cur(true);
    return false;
  }

  void move(Entity& e, const double delta_time) {
    e.velocity = clamp(e.velocity, C::rules.MAX_ENTITY_SPEED);
    e.position += e.velocity * delta_time;
    e.position.y -= C::rules.GRAVITY * delta_time * delta_time / 2;
    e.velocity.y -= C::rules.GRAVITY * delta_time;
  }

  void update(const double delta_time) {
    //H::t[2].start();
    for (auto& robot : robots) {
      if (robot.touch) {
        //H::t[10].start();
        const Point& target_velocity = robot.action.target_velocity - robot.touch_normal * robot.touch_normal.dot(robot.action.target_velocity);
        //H::t[10].cur(true);
        //H::t[14].start();
        const Point& target_velocity_change = target_velocity - robot.velocity;
        //H::t[14].cur(true);
        //H::t[15].start();
        double length = target_velocity_change.length_sq();
        const double& acceleration = C::rules.ROBOT_ACCELERATION * fmax(0., robot.touch_normal.y);
        length = sqrt(length);
        if (acceleration * delta_time <= length) {
          robot.velocity += target_velocity_change * (acceleration * delta_time / length);
        } else {
          robot.velocity += target_velocity_change;
        }
        //H::t[15].cur(true);
      }
      //H::t[11].start();
      move(robot, delta_time);
      //H::t[11].cur(true);
      //H::t[12].start();
      robot.radius = C::rules.ROBOT_MIN_RADIUS + (C::rules.ROBOT_MAX_RADIUS - C::rules.ROBOT_MIN_RADIUS) * robot.action.jump_speed / C::rules.ROBOT_MAX_JUMP_SPEED;
      robot.radius_change_speed = robot.action.jump_speed;
      //H::t[12].cur(true);
    }
    //H::t[2].cur(true);
    //H::t[5].start();
    move(ball, delta_time);
    //H::t[5].cur(true);
    //H::t[6].start();
    Point collision_normal;
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < i; j++) {
        collide_entities(robots[i], robots[j]);
      }
    }
    //H::t[6].cur(true);
    //H::t[7].start();
    for (auto& robot : robots) {
      //H::t[8].start();
      if (collide_entities(robot, ball) && !robot.touch) {
        collide_with_ball[robot.global_id] = true;
      }
      //H::t[8].cur(true);
      //H::t[9].start();
      if (!collide_with_arena(robot, collision_normal)) {
        robot.touch = false;
      } else {
        robot.touch = true;
        robot.touch_normal = collision_normal;
      }
      //H::t[9].cur(true);
    }
    //H::t[7].cur(true);

    //H::t[3].start();
    collide_with_arena(ball, collision_normal);
    //H::t[3].cur(true);

    //H::t[4].start();
    if (!my_goal && !enemy_goal) {
      if (ball.position.z > C::rules.arena.depth / 2 + ball.radius) {
        my_goal = true;
      } else if (-ball.position.z > C::rules.arena.depth / 2 + ball.radius) {
        enemy_goal = true;
      }
    }
    //H::t[4].cur(true);
  }

  void update_trace() {
    for (auto& robot : robots) {
      robot.trace.push_back(robot.position);
    }
    ball.trace.push_back(ball.position);
  }

  void tick() {
    //H::t[1].start();
    double delta_time = 1. / C::rules.TICKS_PER_SECOND;
    //for (int i = 0; i < Constants::rules.MICROTICKS_PER_TICK; i++) {
    //update(delta_time / Constants::rules.MICROTICKS_PER_TICK);
    //}
    update(delta_time);
    //H::t[1].cur(true);
    update_trace();
  }

  Dan dan_to_plane(
      const Point& point,
      const Point& point_on_plane,
      const Point& plane_normal) {
    return {dot(point - point_on_plane, plane_normal), plane_normal};
  }

  Dan dan_to_sphere_inner(
      const Point& point,
      const Point& sphere_center,
      double sphere_radius) {
    return {sphere_radius - length(point - sphere_center),
        normalize(sphere_center - point)};
  }

  Dan dan_to_sphere_outer(
      const Point& point,
      const Point& sphere_center,
      double sphere_radius) {
    return {length(point - sphere_center) - sphere_radius,
        normalize(point - sphere_center)};
  }

  Dan dan_to_arena_quarter(const Point& point, const double radius) {

    // Ground
    Dan dan = dan_to_plane(point, Point(0, 0, 0), Point(0, 1, 0));

    // Ceiling
    dan = std::min(dan, dan_to_plane(point, Point(0, C::rules.arena.height, 0), Point(0, -1, 0)));

    // Side x
    dan = std::min(dan, dan_to_plane(point, Point(C::rules.arena.width / 2, 0, 0), Point(-1, 0, 0)));

    // Side z (goal)
    dan = std::min(dan, dan_to_plane(
        point,
        Point(0, 0, (C::rules.arena.depth / 2) + C::rules.arena.goal_depth),
        Point(0, 0, -1)));

    // Side z
    Point2d v = Point2d(point.x, point.y) - Point2d(
        (C::rules.arena.goal_width / 2) - C::rules.arena.goal_top_radius,
        C::rules.arena.goal_height - C::rules.arena.goal_top_radius);
    if (point.x >= (C::rules.arena.goal_width / 2) + C::rules.arena.goal_side_radius
        || point.y >= C::rules.arena.goal_height + C::rules.arena.goal_side_radius
        || (
            v.x > 0
                && v.y > 0
                && length(v) >= C::rules.arena.goal_top_radius + C::rules.arena.goal_side_radius)) {
      dan = std::min(dan, dan_to_plane(point, Point(0, 0, C::rules.arena.depth / 2), Point(0, 0, -1)));
    }
    // Side x & ceiling (goal)
    if (point.z >= (C::rules.arena.depth / 2) + C::rules.arena.goal_side_radius) {
      // x
      dan = std::min(dan, dan_to_plane(
          point,
          Point(C::rules.arena.goal_width / 2, 0, 0),
          Point(-1, 0, 0)));
      // y
      dan = std::min(dan, dan_to_plane(point, Point(0, C::rules.arena.goal_height, 0), Point(0, -1, 0)));
    }

    // Goal back corners
    if (point.z > (C::rules.arena.depth / 2) + C::rules.arena.goal_depth - C::rules.arena.bottom_radius) {
      dan = std::min(dan, dan_to_sphere_inner(
          point,
          Point(
              std::clamp(
                  point.x,
                  C::rules.arena.bottom_radius - (C::rules.arena.goal_width / 2),
                  (C::rules.arena.goal_width / 2) - C::rules.arena.bottom_radius
              ),
              std::clamp(
                  point.y,
                  C::rules.arena.bottom_radius,
                  C::rules.arena.goal_height - C::rules.arena.goal_top_radius
              ),
              (C::rules.arena.depth / 2) + C::rules.arena.goal_depth - C::rules.arena.bottom_radius),
          C::rules.arena.bottom_radius));
    }
    // Corner
    if (point.x > (C::rules.arena.width / 2) - C::rules.arena.corner_radius
        && point.z > (C::rules.arena.depth / 2) - C::rules.arena.corner_radius) {
      dan = std::min(dan, dan_to_sphere_inner(
          point,
          Point(
              (C::rules.arena.width / 2) - C::rules.arena.corner_radius,
              point.y,
              (C::rules.arena.depth / 2) - C::rules.arena.corner_radius
          ),
          C::rules.arena.corner_radius));
    }
    // Goal outer corner
    if (point.z < (C::rules.arena.depth / 2) + C::rules.arena.goal_side_radius) {
      // Side x
      if (point.x < (C::rules.arena.goal_width / 2) + C::rules.arena.goal_side_radius) {
        dan = std::min(dan, dan_to_sphere_outer(
            point,
            Point(
                (C::rules.arena.goal_width / 2) + C::rules.arena.goal_side_radius,
                point.y,
                (C::rules.arena.depth / 2) + C::rules.arena.goal_side_radius
            ),
            C::rules.arena.goal_side_radius));
      }
      // Ceiling
      if (point.y < C::rules.arena.goal_height + C::rules.arena.goal_side_radius) {
        dan = std::min(dan, dan_to_sphere_outer(
            point,
            Point(
                point.x,
                C::rules.arena.goal_height + C::rules.arena.goal_side_radius,
                (C::rules.arena.depth / 2) + C::rules.arena.goal_side_radius
            ),
            C::rules.arena.goal_side_radius));
      }
      // Top corner
      Point2d o = Point2d(
          (C::rules.arena.goal_width / 2) - C::rules.arena.goal_top_radius,
          C::rules.arena.goal_height - C::rules.arena.goal_top_radius
      );
      Point2d v = Point2d(point.x, point.y) - o;
      if (v.x > 0 && v.y > 0) {
        o = o + normalize(v) * (C::rules.arena.goal_top_radius + C::rules.arena.goal_side_radius);
        dan = std::min(dan, dan_to_sphere_outer(
            point,
            Point(o.x, o.y, (C::rules.arena.depth / 2) + C::rules.arena.goal_side_radius),
            C::rules.arena.goal_side_radius));
      }
    }
    // Goal inside top corners
    if (point.z > (C::rules.arena.depth / 2) + C::rules.arena.goal_side_radius
        && point.y > C::rules.arena.goal_height - C::rules.arena.goal_top_radius) {
      // Side x
      if (point.x > (C::rules.arena.goal_width / 2) - C::rules.arena.goal_top_radius) {
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            Point(
                (C::rules.arena.goal_width / 2) - C::rules.arena.goal_top_radius,
                C::rules.arena.goal_height - C::rules.arena.goal_top_radius,
                point.z
            ),
            C::rules.arena.goal_top_radius));
      }
      // Side z
      if (point.z > (C::rules.arena.depth / 2) + C::rules.arena.goal_depth - C::rules.arena.goal_top_radius) {
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            Point(
                point.x,
                C::rules.arena.goal_height - C::rules.arena.goal_top_radius,
                (C::rules.arena.depth / 2) + C::rules.arena.goal_depth - C::rules.arena.goal_top_radius
            ),
            C::rules.arena.goal_top_radius));
      }
    }

    // Bottom corners
    if (point.y < C::rules.arena.bottom_radius) {
      // Side x
      if (point.x > (C::rules.arena.width / 2) - C::rules.arena.bottom_radius) {
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            Point(
                (C::rules.arena.width / 2) - C::rules.arena.bottom_radius,
                C::rules.arena.bottom_radius,
                point.z
            ),
            C::rules.arena.bottom_radius));
      }
      // Side z
      if (point.z > (C::rules.arena.depth / 2) - C::rules.arena.bottom_radius
          && point.x >= (C::rules.arena.goal_width / 2) + C::rules.arena.goal_side_radius) {
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            Point(
                point.x,
                C::rules.arena.bottom_radius,
                (C::rules.arena.depth / 2) - C::rules.arena.bottom_radius
            ),
            C::rules.arena.bottom_radius));
      }
      // Side z (goal)
      if (point.z > (C::rules.arena.depth / 2) + C::rules.arena.goal_depth - C::rules.arena.bottom_radius) {
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            Point(
                point.x,
                C::rules.arena.bottom_radius,
                (C::rules.arena.depth / 2) + C::rules.arena.goal_depth - C::rules.arena.bottom_radius
            ),
            C::rules.arena.bottom_radius));
      }
      // Goal outer corner
      Point2d o = Point2d(
          (C::rules.arena.goal_width / 2) + C::rules.arena.goal_side_radius,
          (C::rules.arena.depth / 2) + C::rules.arena.goal_side_radius
      );
      Point2d v = Point2d(point.x, point.z) - o;
      if (v.x < 0 && v.y < 0
          && length(v) < C::rules.arena.goal_side_radius + C::rules.arena.bottom_radius) {
        o = o + normalize(v) * (C::rules.arena.goal_side_radius + C::rules.arena.bottom_radius);
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            Point(o.x, C::rules.arena.bottom_radius, o.y),
            C::rules.arena.bottom_radius));
      }
      // Side x (goal)
      if (point.z >= (C::rules.arena.depth / 2) + C::rules.arena.goal_side_radius
          && point.x > (C::rules.arena.goal_width / 2) - C::rules.arena.bottom_radius) {
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            Point(
                (C::rules.arena.goal_width / 2) - C::rules.arena.bottom_radius,
                C::rules.arena.bottom_radius,
                point.z
            ),
            C::rules.arena.bottom_radius));
      }
      // Corner
      if (point.x > (C::rules.arena.width / 2) - C::rules.arena.corner_radius
          && point.z > (C::rules.arena.depth / 2) - C::rules.arena.corner_radius) {
        Point2d corner_o = Point2d(
            (C::rules.arena.width / 2) - C::rules.arena.corner_radius,
            (C::rules.arena.depth / 2) - C::rules.arena.corner_radius
        );
        Point2d n = Point2d(point.x, point.z) - corner_o;
        double dist = n.length();
        if (dist > C::rules.arena.corner_radius - C::rules.arena.bottom_radius) {
          n = n / dist;
          Point2d o2 = corner_o + n * (C::rules.arena.corner_radius - C::rules.arena.bottom_radius);
          dan = std::min(dan, dan_to_sphere_inner(
              point,
              Point(o2.x, C::rules.arena.bottom_radius, o2.y),
              C::rules.arena.bottom_radius));
        }
      }
    }

    // Ceiling corners
    if (point.y > C::rules.arena.height - C::rules.arena.top_radius) {
      // Side x
      if (point.x > (C::rules.arena.width / 2) - C::rules.arena.top_radius) {
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            Point(
                (C::rules.arena.width / 2) - C::rules.arena.top_radius,
                C::rules.arena.height - C::rules.arena.top_radius,
                point.z
            ),
            C::rules.arena.top_radius));
      }
      // Side z
      if (point.z > (C::rules.arena.depth / 2) - C::rules.arena.top_radius) {
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            Point(
                point.x,
                C::rules.arena.height - C::rules.arena.top_radius,
                (C::rules.arena.depth / 2) - C::rules.arena.top_radius
            ),
            C::rules.arena.top_radius));
      }

      // Corner
      if (point.x > (C::rules.arena.width / 2) - C::rules.arena.corner_radius
          && point.z > (C::rules.arena.depth / 2) - C::rules.arena.corner_radius) {
        Point2d corner_o = Point2d(
            (C::rules.arena.width / 2) - C::rules.arena.corner_radius,
            (C::rules.arena.depth / 2) - C::rules.arena.corner_radius
        );
        Point2d dv = Point2d(point.x, point.z) - corner_o;
        if (length(dv) > C::rules.arena.corner_radius - C::rules.arena.top_radius) {
          Point2d n = normalize(dv);
          Point2d o2 = corner_o + n * (C::rules.arena.corner_radius - C::rules.arena.top_radius);
          dan = std::min(dan, dan_to_sphere_inner(
              point,
              Point(o2.x, C::rules.arena.height - C::rules.arena.top_radius, o2.y),
              C::rules.arena.top_radius));
        }
      }
    }

    return dan;
  }

  Dan dan_to_arena(Point& point, const double radius) {
    const bool negate_x = point.x < 0;
    const bool negate_z = point.z < 0;
    if (negate_x) {
      point.x = -point.x;
    }
    if (negate_z) {
      point.z = -point.z;
    }
    Dan result = dan_to_arena_quarter(point, radius);
    if (negate_x) {
      result.normal.x = -result.normal.x;
      point.x = -point.x;
    }
    if (negate_z) {
      result.normal.z = -result.normal.z;
      point.z = -point.z;
    }
    return result;
  }

  void test() {
    { // test 1
      ball = Entity("ball");
      ball.position = {-3.094469, 14.393752, -37.612641};
      ball.velocity = {1.741709, 5.221515, -24.657713};
      tick();
      std::cerr << "########## Test 1:" << std::endl;
      std::cerr << "Position:" << std::endl;
      print_error(ball.position, Point{-3.065441, 14.362125, -37.640170});
      std::cerr << "Velocity:" << std::endl;
      print_error(ball.velocity, Point{1.741709, -7.465922, 16.160170});
    }
    { // test 2
      ball = Entity("ball");
      ball.position = {-23.019338622046326748, 17.999962430128810809, -19.14657426491466552};
      ball.velocity = {14.682921642666871165, 0.058461879134667917024, -1.4818202142470027205};
      tick();
      std::cerr << "########## Test 2:" << std::endl;
      std::cerr << "Position:" << std::endl;
      print_error(ball.position,
                  Point{-22.774622648706465355, 17.996454426285627193, -19.171271268485416073});
      std::cerr << "Velocity:" << std::endl;
      print_error(ball.velocity,
                  Point{14.682959226403681896, -0.46121522321528829469, -1.4818202142470027205});
    }
    { // test 3
      ball = Entity("ball");
      ball.position = Point{16.329517921537998859, 16.195591242457055614, -36.845542433926816273};
      ball.velocity = Point{-25.283203469330487678, 7.6680203103518476127, 6.3722070924858815744};

      tick();
      std::cerr << "########## Test 3:" << std::endl;
      std::cerr << "Position:" << std::endl;
      print_error(ball.position,
                  Point{15.908131197049071304, 16.319208932268558954, -36.739319891201859036});
      std::cerr << "Velocity:" << std::endl;
      print_error(ball.velocity,
                  Point{-25.283203469330487678, 7.1669386313658574039, 6.3734987090127770415});

    }
    { // test 4
      ball = Entity("ball");
      ball.position = Point{-27.995519339371629286, 2.9054418436248079516, 6.1702947673222912073};
      ball.velocity = Point{-0.57403220611490801684, 6.0725454135943017775, -12.125100730674212457};
      tick();
      std::cerr << "########## Test 4:" << std::endl;
      std::cerr << "Position:" << std::endl;
      print_error(ball.position,
                  Point{-27.999998915097581431, 3.0028025199069756646, 5.9682097551443806793});
      std::cerr << "Velocity:" << std::endl;
      print_error(ball.velocity,
                  Point{0.0021698048401537694749, 5.600275987957815893, -12.125100730674212457});

    }

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

};

#endif //CODEBALL_SIMULATOR_H
