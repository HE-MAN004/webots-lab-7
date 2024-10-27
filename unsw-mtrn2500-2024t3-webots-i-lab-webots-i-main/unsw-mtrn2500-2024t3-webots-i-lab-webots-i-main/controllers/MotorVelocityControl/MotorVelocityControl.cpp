#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

constexpr double maxEpuckMotorSpeed{6.28};

// TODO: Complete these functions.
void move(webots::Motor& left, webots::Motor& right, double speed);
void turn(webots::Motor& left, webots::Motor& right, double speed);

void move(webots::Motor& left, webots::Motor& right, double speed) {
  left.setVelocity(speed);
  right.setVelocity(speed);
}

void turn(webots::Motor& left, webots::Motor& right, double speed) {
  left.setVelocity(-speed);
  right.setVelocity(speed);
}
// Waits for a duration (in milliseconds).
void wait(webots::Robot& robot, int duration) {
  const double start{robot.getTime()};
  const double timeStep{robot.getBasicTimeStep()};
  while (robot.getTime() - start < duration * 0.001) {
    robot.step(timeStep);
  }
}

int main(int argc, char **argv) {
  webots::Robot robot;
  webots::Motor& leftMotor{*robot.getMotor("left wheel motor")};
  webots::Motor& rightMotor{*robot.getMotor("right wheel motor")};
  
  // Do not set a limit on how much the wheel can rotate.
  leftMotor.setPosition(INFINITY);
  rightMotor.setPosition(INFINITY);
  
  // Get the time step of the current world.
  double timeStep{robot.getBasicTimeStep()};
  
  // TODO: Complete the robot behaviour as specified.
  while (robot.step(timeStep) != -1) {
    // Move forward at max speed for 1 second.
    move(leftMotor, rightMotor, maxEpuckMotorSpeed);
    wait(robot, 1000);

    // Stop for 0.5 seconds.
    move(leftMotor, rightMotor, 0.0);
    wait(robot, 500);

    // Move backward at half speed for 2 seconds.
    move(leftMotor, rightMotor, -maxEpuckMotorSpeed / 2);
    wait(robot, 2000);

    // Stop for 0.5 seconds.
    move(leftMotor, rightMotor, 0.0);
    wait(robot, 500);

    // Turn left on the spot at quarter speed for 0.5 seconds.
    turn(leftMotor, rightMotor, maxEpuckMotorSpeed / 4);
    wait(robot, 500);

    // Stop for 0.5 seconds.
    move(leftMotor, rightMotor, 0.0);
    wait(robot, 500);
  }

  return 0;
}
