#pragma once

#include "Vec3f.hpp"//our standard vector type

/*!
 * This defines some types we use for the main loop.
 */
struct MainLoopInput {
  float currentTime;  //[s]

  struct {
    float value;  //[V]
    bool updated;
  } batteryVoltage;

  struct {
    Vec3f accelerometer;  //[m/s**2]
    Vec3f rateGyro;  //[rad/s]
    bool updated;
  } imuMeasurement;

  struct {
    bool buttonRed;
    bool buttonGreen;
    bool buttonBlue;
    bool buttonYellow;
    bool buttonArm;
    bool updated;
  } userInput;

  struct {
    float value_x;  //[rad/s]
    float value_y;  //[rad/s]
    bool updated;
  } opticalFlowSensor;

  struct {
    float value;  //[m]
    bool updated;
  } heightSensor;

};

struct MainLoopOutput {
  int motorCommand1;  // located at body +x +y
  int motorCommand2;  // located at body +x -y
  int motorCommand3;  // located at body -x -y
  int motorCommand4;  // located at body -x +y

  //variables that are only used for telemetry:
  float telemetryOutputs_plusMinus100[12];  // NOTE! These are bounded to be in +/- 100
};
