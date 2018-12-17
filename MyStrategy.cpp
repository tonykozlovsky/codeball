#include "MyStrategy.h"

#include <RewindClient/RewindClient.h>
#include <Simulator.h>
#include <MyTimer.h>

#include <iostream>
#include <iomanip>

MyStrategy::MyStrategy() {}

void drawArena() {
  auto& draw = RewindClient::instance();
  //walls
  draw.line3d(
      -Helper::arena.width / 2, 0, -Helper::arena.depth / 2,
      -Helper::arena.width / 2, Helper::arena.height, -Helper::arena.depth / 2);
  draw.line3d(
      -Helper::arena.width / 2, 0, Helper::arena.depth / 2,
      -Helper::arena.width / 2, Helper::arena.height, Helper::arena.depth / 2);
  draw.line3d(
      Helper::arena.width / 2, 0, -Helper::arena.depth / 2,
      Helper::arena.width / 2, Helper::arena.height, -Helper::arena.depth / 2);
  draw.line3d(
      Helper::arena.width / 2, 0, Helper::arena.depth / 2,
      Helper::arena.width / 2, Helper::arena.height, Helper::arena.depth / 2);
  //floor
  draw.line3d(
      -Helper::arena.width / 2, 0, -Helper::arena.depth / 2,
      -Helper::arena.width / 2, 0, Helper::arena.depth / 2);
  draw.line3d(
      -Helper::arena.width / 2, 0, Helper::arena.depth / 2,
      Helper::arena.width / 2, 0, Helper::arena.depth / 2);
  draw.line3d(
      Helper::arena.width / 2, 0, Helper::arena.depth / 2,
      Helper::arena.width / 2, 0, -Helper::arena.depth / 2);
  draw.line3d(
      Helper::arena.width / 2, 0, -Helper::arena.depth / 2,
      -Helper::arena.width / 2, 0, -Helper::arena.depth / 2);
  //ceiling
  draw.line3d(
      -Helper::arena.width / 2, Helper::arena.height, -Helper::arena.depth / 2,
      -Helper::arena.width / 2, Helper::arena.height, Helper::arena.depth / 2);
  draw.line3d(
      -Helper::arena.width / 2, Helper::arena.height, Helper::arena.depth / 2,
      Helper::arena.width / 2, Helper::arena.height, Helper::arena.depth / 2);
  draw.line3d(
      Helper::arena.width / 2, Helper::arena.height, Helper::arena.depth / 2,
      Helper::arena.width / 2, Helper::arena.height, -Helper::arena.depth / 2);
  draw.line3d(
      Helper::arena.width / 2, Helper::arena.height, -Helper::arena.depth / 2,
      -Helper::arena.width / 2, Helper::arena.height, -Helper::arena.depth / 2);
  //goal1
  draw.line3d(
      -Helper::arena.goal_width / 2, 0, -Helper::arena.depth / 2,
      -Helper::arena.goal_width / 2, 0, -Helper::arena.depth / 2 - Helper::arena.goal_depth);
  draw.line3d(
      Helper::arena.goal_width / 2, 0, -Helper::arena.depth / 2,
      Helper::arena.goal_width / 2, 0, -Helper::arena.depth / 2 - Helper::arena.goal_depth);
  draw.line3d(
      -Helper::arena.goal_width / 2,
      Helper::arena.goal_height,
      -Helper::arena.depth / 2,
      -Helper::arena.goal_width / 2,
      Helper::arena.goal_height,
      -Helper::arena.depth / 2 - Helper::arena.goal_depth);
  draw.line3d(
      Helper::arena.goal_width / 2,
      Helper::arena.goal_height,
      -Helper::arena.depth / 2,
      Helper::arena.goal_width / 2,
      Helper::arena.goal_height,
      -Helper::arena.depth / 2 - Helper::arena.goal_depth);
  draw.line3d(
      -Helper::arena.goal_width / 2, 0, -Helper::arena.depth / 2 - Helper::arena.goal_depth,
      Helper::arena.goal_width / 2, 0, -Helper::arena.depth / 2 - Helper::arena.goal_depth);
  draw.line3d(
      -Helper::arena.goal_width / 2,
      Helper::arena.goal_height,
      -Helper::arena.depth / 2 - Helper::arena.goal_depth,
      Helper::arena.goal_width / 2,
      Helper::arena.goal_height,
      -Helper::arena.depth / 2 - Helper::arena.goal_depth);
  draw.line3d(
      -Helper::arena.goal_width / 2,
      0,
      -Helper::arena.depth / 2 - Helper::arena.goal_depth,
      -Helper::arena.goal_width / 2,
      Helper::arena.goal_height,
      -Helper::arena.depth / 2 - Helper::arena.goal_depth);
  draw.line3d(
      Helper::arena.goal_width / 2,
      0,
      -Helper::arena.depth / 2 - Helper::arena.goal_depth,
      Helper::arena.goal_width / 2,
      Helper::arena.goal_height,
      -Helper::arena.depth / 2 - Helper::arena.goal_depth);
  //goal2
  draw.line3d(
      -Helper::arena.goal_width / 2, 0, Helper::arena.depth / 2,
      -Helper::arena.goal_width / 2, 0, Helper::arena.depth / 2 + Helper::arena.goal_depth);
  draw.line3d(
      Helper::arena.goal_width / 2, 0, Helper::arena.depth / 2,
      Helper::arena.goal_width / 2, 0, Helper::arena.depth / 2 + Helper::arena.goal_depth);
  draw.line3d(
      -Helper::arena.goal_width / 2,
      Helper::arena.goal_height,
      Helper::arena.depth / 2,
      -Helper::arena.goal_width / 2,
      Helper::arena.goal_height,
      Helper::arena.depth / 2 + Helper::arena.goal_depth);
  draw.line3d(
      Helper::arena.goal_width / 2,
      Helper::arena.goal_height,
      Helper::arena.depth / 2,
      Helper::arena.goal_width / 2,
      Helper::arena.goal_height,
      Helper::arena.depth / 2 + Helper::arena.goal_depth);
  draw.line3d(
      -Helper::arena.goal_width / 2, 0, Helper::arena.depth / 2 + Helper::arena.goal_depth,
      Helper::arena.goal_width / 2, 0, Helper::arena.depth / 2 + Helper::arena.goal_depth);
  draw.line3d(
      -Helper::arena.goal_width / 2,
      Helper::arena.goal_height,
      Helper::arena.depth / 2 + Helper::arena.goal_depth,
      Helper::arena.goal_width / 2,
      Helper::arena.goal_height,
      Helper::arena.depth / 2 + Helper::arena.goal_depth);
  draw.line3d(
      -Helper::arena.goal_width / 2,
      0,
      Helper::arena.depth / 2 + Helper::arena.goal_depth,
      -Helper::arena.goal_width / 2,
      Helper::arena.goal_height,
      Helper::arena.depth / 2 + Helper::arena.goal_depth);
  draw.line3d(
      Helper::arena.goal_width / 2,
      0,
      Helper::arena.depth / 2 + Helper::arena.goal_depth,
      Helper::arena.goal_width / 2,
      Helper::arena.goal_height,
      Helper::arena.depth / 2 + Helper::arena.goal_depth);

}

void doStrategy() {
  auto& draw = RewindClient::instance();
  drawArena();

  Simulator simulator(Helper::game.robots, Helper::game.ball);

  MyTimer t;
  t.start();
  for (int i = 0; i < 1000; i++) {
    /*for (auto& robot : simulator.robots) {
      draw.circle3d(
          robot.position.x,
          robot.position.y,
          robot.position.z,
          robot.radius);
    }
    draw.circle3d(
        simulator.ball.position.x,
        simulator.ball.position.y,
        simulator.ball.position.z,
        simulator.ball.radius);*/
    simulator.tick();
  }
  draw.message("Time: ", t.cur() * 1000);

  draw.end_frame();
}

void MyStrategy::act(
    const model::Robot& me,
    const model::Rules& rules,
    const model::Game& game,
    model::Action& action) {
  if (Helper::tryInit(me, rules, game)) {
    doStrategy();
  } else {
    action = Helper::getCurrentAction();
  }
}

