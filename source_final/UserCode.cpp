#include "UserCode.hpp"
#include "UtilityFunctions.hpp"
#include "Vec3f.hpp"

#include <stdio.h> //for printf

//An example of a variable that persists beyond the function call.
float exampleVariable_float = 0.0f;  //Note the trailing 'f' in the number. This is to force single precision floating point.
Vec3f exampleVariable_Vec3f = Vec3f(0, 0, 0);
int exampleVariable_int = 0;

//We keep the last inputs and outputs around for debugging:
MainLoopInput lastMainLoopInputs;
MainLoopOutput lastMainLoopOutputs;

//Some constants that we may use:
const float mass = 32e-3f;  // mass of the quadcopter [kg]
const float gravity = 9.81f;  // acceleration of gravity [m/s^2]
const float inertia_xx = 16e-6f;  //MMOI about x axis [kg.m^2]
const float inertia_yy = inertia_xx;  //MMOI about y axis [kg.m^2]
const float inertia_zz = 29e-6f;  //MMOI about z axis [kg.m^2]
const float l = 33e-3f;
const float k = 0.01f;

const float dt = 1.0f / 500.0f;  //[s] period between successive calls to MainLoop

//New variables
Vec3f estGyroBias = Vec3f(0, 0, 0);
Vec3f rateGyro_corr = Vec3f(0, 0, 0);

float estRoll = 0;
float estPitch = 0;
float estYaw = 0;
float rho = 0.00001;

float n1 = 0.0f;
float n2 = 0.0f;
float n3 = 0.0f;
float csum = mass * gravity;
float thrust = 8.0f;
float pdot = 0.0f;
float qdot = 0.0f;
float rdot = 5.0f;
float p_des = 0.0f;
float q_des = 0.0f;
float r_des = 0.0f;
Vec3f cmdAngAcc = Vec3f(0,0,0);
Vec3f cmdAngVel = Vec3f(0,0,0);
float roll_des = 0.0f;
float pitch_des = 0.0f;
float yaw_des = 0.0f;

float estHeight = 0;
float estVelocity_1 = 0;
float estVelocity_2 = 0;
float estVelocity_3 = 0;
float lastHeightMeas_meas = 0;
float lastHeightMeas_time = 0;
float natFreq_height = 2.0f;
float dampingRatio_height = 0.7f;
const float desiredHeight = 1.6f; //this is the max height
float current_time = 0;
float runway_time = 0;



MainLoopOutput MainLoop(MainLoopInput const &in) {
  //Your code goes here!
  // The function input (named "in") is a struct of type
  // "MainLoopInput". You can understand what values it
  // contains by going to its definition (click on "MainLoopInput",
  // and then hit <F3> -- this should take you to the definition).
  // For example, "in.userInput.buttonBlue" is true if the
  // blue button is pushed, false otherwise.

  //Define the output numbers (in the struct outVals):
  MainLoopOutput outVals;
//  motorCommand1 -> located at body +x +y
//  motorCommand2 -> located at body +x -y
//  motorCommand3 -> located at body -x -y
//  motorCommand4 -> located at body -x +y
  outVals.motorCommand1 = 0;
  outVals.motorCommand2 = 0;
  outVals.motorCommand3 = 0;
  outVals.motorCommand4 = 0;

  bool runway = false; // taking off or landing
  float h_now = desiredHeight;

  if (in.userInput.buttonGreen) { //LANDING
    runway = true;
    h_now = 0.005f;
    runway_time = runway_time + dt;
    natFreq_height = 2.4f;
    dampingRatio_height = 0.7f;
  }

  if (in.userInput.buttonBlue) { //TAKEOFF, doesn't do anything crazy, just makes it smooth takeoff
    runway = false; //
    float a = desiredHeight * 4.0f; //calculates acceleration
    float t = runway_time; //internal time
    if (runway_time < 0.5f) {
      h_now = 0.5f*a*t*t; //concave up
    }
    else if (runway_time < 1.0f) {
      float c = desiredHeight - 0.5f*a;
      h_now = -0.5f*a*t*t + a*t + c; //concave down
    }
    else {
      h_now = desiredHeight;
    }
    runway_time = runway_time + dt;

  }

  if (runway == false) {
    runway_time = 0; //resets it to 0 when not landing
    natFreq_height = 2.0f; //resets to original flying constants
    dampingRatio_height = 0.7f;
  }

  if (in.currentTime < 1.0f) {
    estGyroBias = estGyroBias + (in.imuMeasurement.rateGyro / 500.0f);
  }
  rateGyro_corr = in.imuMeasurement.rateGyro - estGyroBias;

  //Executing the integrator
  estRoll = (1.0f - rho) * (estRoll + dt*rateGyro_corr.x) + (rho * (1.0f * in.imuMeasurement.accelerometer.y / gravity));
  estPitch = (1.0f - rho) * (estPitch + dt*rateGyro_corr.y) + (rho * (-1.0f * in.imuMeasurement.accelerometer.x / gravity));
//  estYaw = (1 - rho) * (estYaw + dt*rateGyro_corr.z) + (rho * (-1 * lastMainLoopInputs.imuMeasurement.accelerometer.z / abs(gravity)));
  estYaw = estYaw + dt*rateGyro_corr.z;

  estHeight = estHeight + estVelocity_3 * dt;
  estVelocity_3 = estVelocity_3 + 0 * dt; //assume constant

  float const mixHeight = 0.3f;
  if (in.heightSensor.updated) {
    //check that the measurement is reasonable
    if (in.heightSensor.value < 5.0f) {
      float hMeas = in.heightSensor.value * cosf(estRoll) * cosf(estPitch);
      estHeight = (1 - mixHeight) * estHeight + mixHeight * hMeas;
      float v3Meas = (hMeas - lastHeightMeas_meas) / (in.currentTime - lastHeightMeas_time);
      estVelocity_3 = (1 - mixHeight) * estVelocity_3 + mixHeight * v3Meas;
      //store this measurement for the next velocity update
      lastHeightMeas_meas = hMeas;
      lastHeightMeas_time = in.currentTime;
    }
  }

  estVelocity_1 = estVelocity_1 + 0*dt;
  estVelocity_2 = estVelocity_2 + 0*dt;

  float const mixHorizVel = 0.1f;
  if (in.opticalFlowSensor.updated) {
    float sigma_1 = in.opticalFlowSensor.value_x;
    float sigma_2 = in.opticalFlowSensor.value_y;
    float div = (cosf(estRoll) * cosf(estPitch));
    if (div > 0.5f) {
      float deltaPredict = estHeight / div; //this is delta in the equation
      float v1Meas = (-sigma_1 + in.imuMeasurement.rateGyro.y) * deltaPredict;
      float v2Meas = (-sigma_2 - in.imuMeasurement.rateGyro.x) * deltaPredict;
      estVelocity_1 = (1 - mixHorizVel) * estVelocity_1 + mixHorizVel * v1Meas;
      estVelocity_2 = (1 - mixHorizVel) * estVelocity_2 + mixHorizVel * v2Meas;
    }
  }

  float desHeight = desiredHeight;

  if (runway == true) {
    desHeight = h_now; //set the landing heigh
  }

  float desAcc3 = (-2 * dampingRatio_height * natFreq_height * estVelocity_3) - (natFreq_height * natFreq_height) * (estHeight - desHeight);
  float desNormalizedAcceleration = (gravity + desAcc3) / (cosf(estRoll) * cosf(estPitch));
  thrust = desNormalizedAcceleration;

  float const timeConstant_rollRate = 0.03f;
  float const timeConstant_pitchRate = timeConstant_rollRate;
  float const timeConstant_yawRate = 0.1f;
  float const timeConstant_rollAngle = 0.08f;
  float const timeConstant_pitchAngle = timeConstant_rollAngle;
  float const timeConstant_yawAngle = 0.2f;
  float const timeConst_horizVel = 3.0f;

  float desAcc1 = -(1 / timeConst_horizVel) * estVelocity_1;
  float desAcc2 = -(1 / timeConst_horizVel) * estVelocity_2;

  roll_des = -desAcc2 / gravity;
  pitch_des = desAcc1 / gravity;
  yaw_des = 0;


  cmdAngVel.x = (-1.0f/timeConstant_rollAngle) * (estRoll - roll_des);
  cmdAngVel.y = (-1.0f/timeConstant_pitchAngle) * (estPitch - pitch_des);
  cmdAngVel.z = (-1.0f/timeConstant_yawAngle) * (estYaw - yaw_des);
  cmdAngAcc.x = (-1.0f/timeConstant_rollRate) * (rateGyro_corr.x - cmdAngVel.x);
  cmdAngAcc.y = (-1.0f/timeConstant_pitchRate) * (rateGyro_corr.y - cmdAngVel.y);
  cmdAngAcc.z = (-1.0f/timeConstant_yawRate) * (rateGyro_corr.z - cmdAngVel.z);


  csum = mass * thrust;
  n1 = cmdAngAcc.x * inertia_xx;
  n2 = cmdAngAcc.y * inertia_yy;
  n3 = cmdAngAcc.z * inertia_zz;

  const float c1 = 0.25f * (csum + n1/l - n2/l + n3/k);
  const float c2 = 0.25f * (csum - n1/l - n2/l - n3/k);
  const float c3 = 0.25f * (csum - n1/l + n2/l + n3/k);
  const float c4 = 0.25f * (csum + n1/l + n2/l - n3/k);


  outVals.motorCommand1 = pwmCommandFromSpeed(speedFromForce(c1));  //apply cmd to all motors equally
  outVals.motorCommand2 = pwmCommandFromSpeed(speedFromForce(c2));
  outVals.motorCommand3 = pwmCommandFromSpeed(speedFromForce(c3));
  outVals.motorCommand4 = pwmCommandFromSpeed(speedFromForce(c4));

  outVals.telemetryOutputs_plusMinus100[0] = estRoll;
  outVals.telemetryOutputs_plusMinus100[1] = estPitch;
  outVals.telemetryOutputs_plusMinus100[2] = estYaw;
  outVals.telemetryOutputs_plusMinus100[3] = estVelocity_1;
  outVals.telemetryOutputs_plusMinus100[4] = estVelocity_2;
  outVals.telemetryOutputs_plusMinus100[5] = estVelocity_3;
  outVals.telemetryOutputs_plusMinus100[6] = estHeight;
  outVals.telemetryOutputs_plusMinus100[7] = roll_des;
  outVals.telemetryOutputs_plusMinus100[8] = pitch_des;
  outVals.telemetryOutputs_plusMinus100[9] = 0;

  lastMainLoopInputs = in;
  lastMainLoopOutputs = outVals;

  current_time = current_time + dt;


  return outVals;
}

void PrintStatus() {
  //For a quick reference on the printf function, see: http://www.cplusplus.com/reference/cstdio/printf/
  // Note that \n is a "new line" character.
  // Also, note that to print a `float` variable, you have to explicitly cast it to
  //  `double` in the printf function, and explicitly specify precision using something
  //  like %6.3f (six significant digits, three after the period). Example:
  //   printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));

  //Accelerometer measurement
  printf("Acc:");
  printf("x=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.x));
  printf("y=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.y));
  printf("z=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.z));
  //Rate gyro measurement
  printf("Gyro:");
  printf("x=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.x));
  printf("y=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.y));
  printf("z=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.z));
  printf("\n"); //New line

  printf("Gyro Correction = (%6.3f, %6.3f, %6.3f)\n",
         double(rateGyro_corr.x), double(rateGyro_corr.y),
         double(rateGyro_corr.z));
  printf("Gyro Bias = (%6.3f, %6.3f, %6.3f)\n",
         double(estGyroBias.x), double(estGyroBias.y),
         double(estGyroBias.z));

  printf("Altitude Estimations: \n");
  printf("Altitude:");
  printf("estRoll=%6.3f, ", double(estRoll));
  printf("estPitch=%6.3f, ", double(estPitch));
  printf("estYaw=%6.3f, ", double(estYaw));
  printf("\n"); //New line
  printf("Last range = %6.3fm, " , \
         double(lastMainLoopInputs.heightSensor.value));
  printf("Last flow: x=%6.3f, y=%6.3f\n" , \
         double(lastMainLoopInputs.opticalFlowSensor.value_x), \
         double(lastMainLoopInputs.opticalFlowSensor.value_y));


//  printf("Example variable values:\n");
//  printf("  exampleVariable_int = %d\n", exampleVariable_int);
//  //Note that it is somewhat annoying to print float variables.
//  //  We need to cast the variable as double, and we need to specify
//  //  the number of digits we want (if you used simply "%f", it would
//  //  truncate to an integer.
//  //  Here, we print 6 digits, with three digits after the period.
//  printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));
//
//  //We print the Vec3f by printing it's three components independently:
//  printf("  exampleVariable_Vec3f = (%6.3f, %6.3f, %6.3f)\n",
//         double(exampleVariable_Vec3f.x), double(exampleVariable_Vec3f.y),
//         double(exampleVariable_Vec3f.z));

  //just an example of how we would inspect the last main loop inputs and outputs:
  printf("Last main loop inputs:\n");
  printf("  batt voltage = %6.3f\n",
         double(lastMainLoopInputs.batteryVoltage.value));
  printf("  JS buttons: ");
  if (lastMainLoopInputs.userInput.buttonRed)
    printf("buttonRed ");
  if (lastMainLoopInputs.userInput.buttonGreen)
    printf("buttonGreen ");
  if (lastMainLoopInputs.userInput.buttonBlue)
    printf("buttonBlue ");
  if (lastMainLoopInputs.userInput.buttonYellow)
    printf("buttonYellow ");
  if (lastMainLoopInputs.userInput.buttonArm)
    printf("buttonArm ");
  printf("\n");
  printf("Last main loop outputs:\n");
  printf("  motor commands: = %6.3f\t%6.3f\t%6.3f\t%6.3f\t\n",
         double(lastMainLoopOutputs.motorCommand1),
         double(lastMainLoopOutputs.motorCommand2),
         double(lastMainLoopOutputs.motorCommand3),
         double(lastMainLoopOutputs.motorCommand4));
}
