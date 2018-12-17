#include "MyStrategy.h"

#include <RewindClient/RewindClient.h>

#include <iostream>
#include <iomanip>

using namespace model;

MyStrategy::MyStrategy() { }

void drawArena(const Rules& rules) {
  auto& draw = RewindClient::instance();
  //walls
  draw.line3d(
      -rules.arena.width / 2, 0, -rules.arena.depth / 2,
      -rules.arena.width / 2, rules.arena.height, -rules.arena.depth / 2);
  draw.line3d(
      -rules.arena.width / 2, 0, rules.arena.depth / 2,
      -rules.arena.width / 2, rules.arena.height, rules.arena.depth / 2);
  draw.line3d(
      rules.arena.width / 2, 0, -rules.arena.depth / 2,
      rules.arena.width / 2, rules.arena.height, -rules.arena.depth / 2);
  draw.line3d(
      rules.arena.width / 2, 0, rules.arena.depth / 2,
      rules.arena.width / 2, rules.arena.height, rules.arena.depth / 2);
  //floor
  draw.line3d(
      -rules.arena.width / 2, 0, -rules.arena.depth / 2,
      -rules.arena.width / 2, 0, rules.arena.depth / 2);
  draw.line3d(
      -rules.arena.width / 2, 0, rules.arena.depth / 2,
      rules.arena.width / 2, 0, rules.arena.depth / 2);
  draw.line3d(
      rules.arena.width / 2, 0, rules.arena.depth / 2,
      rules.arena.width / 2, 0, -rules.arena.depth / 2);
  draw.line3d(
      rules.arena.width / 2, 0, -rules.arena.depth / 2,
      -rules.arena.width / 2, 0, -rules.arena.depth / 2);
  //ceiling
  draw.line3d(
      -rules.arena.width / 2, rules.arena.height, -rules.arena.depth / 2,
      -rules.arena.width / 2, rules.arena.height, rules.arena.depth / 2);
  draw.line3d(
      -rules.arena.width / 2, rules.arena.height, rules.arena.depth / 2,
      rules.arena.width / 2, rules.arena.height, rules.arena.depth / 2);
  draw.line3d(
      rules.arena.width / 2, rules.arena.height, rules.arena.depth / 2,
      rules.arena.width / 2, rules.arena.height, -rules.arena.depth / 2);
  draw.line3d(
      rules.arena.width / 2, rules.arena.height, -rules.arena.depth / 2,
      -rules.arena.width / 2, rules.arena.height, -rules.arena.depth / 2);
  //goal1
  draw.line3d(
      -rules.arena.goal_width / 2, 0, -rules.arena.depth / 2,
      -rules.arena.goal_width / 2, 0, -rules.arena.depth / 2 - rules.arena.goal_depth);
  draw.line3d(
      rules.arena.goal_width / 2, 0, -rules.arena.depth / 2,
      rules.arena.goal_width / 2, 0, -rules.arena.depth / 2 - rules.arena.goal_depth);
  draw.line3d(
      -rules.arena.goal_width / 2, rules.arena.goal_height, -rules.arena.depth / 2,
      -rules.arena.goal_width / 2, rules.arena.goal_height, -rules.arena.depth / 2 - rules.arena.goal_depth);
  draw.line3d(
      rules.arena.goal_width / 2, rules.arena.goal_height, -rules.arena.depth / 2,
      rules.arena.goal_width / 2, rules.arena.goal_height, -rules.arena.depth / 2 - rules.arena.goal_depth);
  draw.line3d(
      -rules.arena.goal_width / 2, 0, -rules.arena.depth / 2 - rules.arena.goal_depth,
      rules.arena.goal_width / 2, 0, -rules.arena.depth / 2 - rules.arena.goal_depth);
  draw.line3d(
      -rules.arena.goal_width / 2, rules.arena.goal_height, -rules.arena.depth / 2 - rules.arena.goal_depth,
      rules.arena.goal_width / 2, rules.arena.goal_height, -rules.arena.depth / 2 - rules.arena.goal_depth);
  draw.line3d(
      -rules.arena.goal_width / 2, 0, -rules.arena.depth / 2 - rules.arena.goal_depth,
      -rules.arena.goal_width / 2, rules.arena.goal_height, -rules.arena.depth / 2 - rules.arena.goal_depth);
  draw.line3d(
      rules.arena.goal_width / 2, 0, -rules.arena.depth / 2 - rules.arena.goal_depth,
      rules.arena.goal_width / 2, rules.arena.goal_height, -rules.arena.depth / 2 - rules.arena.goal_depth);
  //goal2
  draw.line3d(
      -rules.arena.goal_width / 2, 0, rules.arena.depth / 2,
      -rules.arena.goal_width / 2, 0, rules.arena.depth / 2 + rules.arena.goal_depth);
  draw.line3d(
      rules.arena.goal_width / 2, 0, rules.arena.depth / 2,
      rules.arena.goal_width / 2, 0, rules.arena.depth / 2 + rules.arena.goal_depth);
  draw.line3d(
      -rules.arena.goal_width / 2, rules.arena.goal_height, rules.arena.depth / 2,
      -rules.arena.goal_width / 2, rules.arena.goal_height, rules.arena.depth / 2 + rules.arena.goal_depth);
  draw.line3d(
      rules.arena.goal_width / 2, rules.arena.goal_height, rules.arena.depth / 2,
      rules.arena.goal_width / 2, rules.arena.goal_height, rules.arena.depth / 2 + rules.arena.goal_depth);
  draw.line3d(
      -rules.arena.goal_width / 2, 0, rules.arena.depth / 2 + rules.arena.goal_depth,
      rules.arena.goal_width / 2, 0, rules.arena.depth / 2 + rules.arena.goal_depth);
  draw.line3d(
      -rules.arena.goal_width / 2, rules.arena.goal_height, rules.arena.depth / 2 + rules.arena.goal_depth,
      rules.arena.goal_width / 2, rules.arena.goal_height, rules.arena.depth / 2 + rules.arena.goal_depth);
  draw.line3d(
      -rules.arena.goal_width / 2, 0, rules.arena.depth / 2 + rules.arena.goal_depth,
      -rules.arena.goal_width / 2, rules.arena.goal_height, rules.arena.depth / 2 + rules.arena.goal_depth);
  draw.line3d(
      rules.arena.goal_width / 2, 0, rules.arena.depth / 2 + rules.arena.goal_depth,
      rules.arena.goal_width / 2, rules.arena.goal_height, rules.arena.depth / 2 + rules.arena.goal_depth);

}

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action) {
  auto& draw = RewindClient::instance();
  if (me.id == 1) {
    std::cout << std::fixed << std::setprecision(30) << me.y << std::endl;
    //if (game.current_tick == 20) {
    action.jump_speed = 100;
    drawArena(rules);
    for (auto& rob : game.robots) {
      draw.circle3d(rob.x, rob.y, rob.z, rob.radius);
    }
    draw.circle3d(game.ball.x, game.ball.y, game.ball.z, game.ball.radius);
    draw.end_frame();
    //}
  }
}
