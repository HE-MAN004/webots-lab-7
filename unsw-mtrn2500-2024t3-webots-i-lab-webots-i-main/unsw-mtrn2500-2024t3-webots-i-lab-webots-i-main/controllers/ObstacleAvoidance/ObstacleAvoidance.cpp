#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <limits>

#define TIME_STEP 64

constexpr double epuckWheelRadius{0.0206};
constexpr double epuckAxleLength{0.052};
constexpr double cellLength{0.2};
constexpr double pi{3.14159265358979323846};


constexpr double obstacleThreshold = 80.0; // Adjust based on sensor lookup table
constexpr double MAX_SPEED{6.28};


int main(int argc, char **argv) {
  webots::Robot *robot = new webots::Robot();
  
  webots::Motor *leftMotor = robot->getMotor("left wheel motor");
  webots::Motor *rightMotor = robot->getMotor("right wheel motor");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  
  
  webots::DistanceSensor* frontSensorLeft = robot->getDistanceSensor("ps7");
  webots::DistanceSensor* frontSensorRight = robot->getDistanceSensor("ps0");
  webots::PositionSensor& leftSensor{*robot->getPositionSensor("left wheel sensor")};
  webots::PositionSensor& rightSensor{*robot->getPositionSensor("right wheel sensor")};

  frontSensorLeft->enable(TIME_STEP);
  frontSensorRight->enable(TIME_STEP);

  leftSensor.enable(TIME_STEP);
  rightSensor.enable(TIME_STEP);

  leftMotor->setPosition(std::numeric_limits<double>::infinity());
  rightMotor->setPosition(std::numeric_limits<double>::infinity());
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);

  while (robot->step(TIME_STEP) != -1) {    
    double leftSpeed  = 0.5 * MAX_SPEED;
    double rightSpeed = 0.5 * MAX_SPEED;
    double frontDistance = frontSensorLeft->getValue();


    if (frontSensorLeft->getValue() > obstacleThreshold || frontSensorRight->getValue() > obstacleThreshold) {      
      double angleRadians = 60 * (pi / 180.0);
      double wheelRotation = ((epuckAxleLength / 2) * angleRadians) / epuckWheelRadius;
    
      double leftTarget = leftSensor.getValue() + wheelRotation;
      double rightTarget = rightSensor.getValue() - wheelRotation;
    
      leftMotor->setPosition(leftTarget);
      rightMotor->setPosition(rightTarget);
      robot->step(2000);
  
     }

    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
    
        leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    
  }
  delete robot;

  return 0;
}