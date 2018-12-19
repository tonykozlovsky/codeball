#ifdef LOCAL

#include <RewindClient/RewindClient.h>
#include <Simulator.h>
#include <MyTimer.h>
#include <MyStrategy.h>

#else

#include "Simulator.h"
#include "MyTimer.h"
#include "MyStrategy.h"

#endif

#include <iostream>
#include <iomanip>

MyStrategy::MyStrategy() {}

#ifdef LOCAL
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
#endif

void doStrategy() {

#ifdef LOCAL
  auto& draw = RewindClient::instance();
  drawArena();
#endif

  Simulator simulator(Helper::game.robots, Helper::game.ball);

#ifdef LOCAL
  for (auto& robot : simulator.robots) {
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
      simulator.ball.radius);

#endif
  Helper::t[10].start();
  simulator.tick();
  Helper::t[10].cur(true, true);

  /*Helper::t[0].cur(false, true);
  Helper::t[1].cur(false, true);
  Helper::t[2].cur(false, true);
  Helper::t[3].cur(false, true);
  Helper::t[4].cur(false, true);
  Helper::t[5].cur(false, true);
  Helper::t[6].cur(false, true);
  Helper::t[7].cur(false, true);
  Helper::t[8].cur(false, true);
  Helper::t[9].cur(false, true);
  Helper::t[10].cur(false, true);*/

  /*std::cout << std::fixed << std::setprecision(6) << "t0: " << Helper::t[0].avg() * 1000000 <<
  std::endl;
  std::cout << std::fixed << std::setprecision(6) << "t1: " << Helper::t[1].avg() * 1000000 << std::endl;
  std::cout << std::fixed << std::setprecision(6) << "t2: " << Helper::t[2].avg() * 1000000 << std::endl;
  std::cout << std::fixed << std::setprecision(6) << "t3: " << Helper::t[3].avg() * 1000000 << std::endl;
  std::cout << std::fixed << std::setprecision(6) << "t4: " << Helper::t[4].avg() * 1000000 << std::endl;
  std::cout << std::fixed << std::setprecision(6) << "t5: " << Helper::t[5].avg() * 1000000 << std::endl;
  std::cout << std::fixed << std::setprecision(6) << "t6: " << Helper::t[6].avg() * 1000000 << std::endl;
  std::cout << std::fixed << std::setprecision(6) << "t7: " << Helper::t[7].avg() * 1000000 << std::endl;
  std::cout << std::fixed << std::setprecision(6) << "t8: " << Helper::t[8].avg() * 1000000 << std::endl;
  std::cout << std::fixed << std::setprecision(6) << "t9: " << Helper::t[9].avg() * 1000000 << std::endl;
  */
  if (Helper::tick == 17500) {
  std::cout << std::fixed << std::setprecision(6) << "t10: " << Helper::t[10].avg() * 1000
            << std::endl;
  std::cout << std::endl;
}

#ifdef LOCAL
  draw.message("Time0: ", Helper::t[0].avg() * 1000);
  draw.message("Time1: ", Helper::t[1].avg() * 1000);
  draw.message("Time2: ", Helper::t[2].avg() * 1000);
  draw.message("Time3: ", Helper::t[3].avg() * 1000);
  draw.message("Time4: ", Helper::t[4].avg() * 1000);
  draw.message("Time5: ", Helper::t[5].avg() * 1000);
  draw.message("Time6: ", Helper::t[6].avg() * 1000);
  draw.message("Time7: ", Helper::t[7].avg() * 1000);
  draw.message("Time8: ", Helper::t[8].avg() * 1000);
  draw.message("Time9: ", Helper::t[9].avg() * 1000);
  draw.message("Time10: ", Helper::t[10].avg());

  for (auto& robot : simulator.robots) {
    for (int i = 1; i < robot.trace.size(); i += 1) {
      draw.line3d(
          robot.trace[i - 1].x,
          robot.trace[i - 1].y,
          robot.trace[i - 1].z,
          robot.trace[i].x,
          robot.trace[i].y,
          robot.trace[i].z);
    }
  }
  for (int i = 1; i < simulator.ball.trace.size(); i += 1) {
    draw.line3d(
        simulator.ball.trace[i - 1].x,
        simulator.ball.trace[i - 1].y,
        simulator.ball.trace[i - 1].z,
        simulator.ball.trace[i].x,
        simulator.ball.trace[i].y,
        simulator.ball.trace[i].z);
  }

  draw.end_frame();
#endif

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

