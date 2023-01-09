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

const float dt = 1.0f / 500.0f;  //[s] period between successive calls to MainLoop

//New variables (10/13)
Vec3f estGyroBias = Vec3f(0, 0, 0);
Vec3f rateGyro_corr = Vec3f(0, 0, 0);

float estRoll = 0.0f;
float estPitch = 0.0f;
float estYaw = 0.0f;
const float rho = 0.00001f; //0.01

//New variables (10/27)
float c1 = 0.0f;
float c2 = 0.0f;
float c3 = 0.0f;
float c4 = 0.0f;

const float l = 0.033f; // 0.033[m]
const float k = 0.01f; // 0.01[m]

//New variables (11/10)
float estHeight = 0;
float estVelocity_1 = 0;
float estVelocity_2 = 0;
float estVelocity_3 = 0;

float lastHeightMeas_meas = 0;
float lastHeightMeas_time = 0;

const float natFreq_height = 2.0f; // 2.0f
const float dampingRatio_height = 0.7f; // 0.7f

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

  // CODE ADDED (10/13)
  if (in.currentTime < 1.0f) {
    estGyroBias = estGyroBias + (in.imuMeasurement.rateGyro / 500.0f);
  }
  rateGyro_corr = in.imuMeasurement.rateGyro - estGyroBias;

  // Executing the integrator
  estRoll = (1.0f - rho) * (estRoll + dt*rateGyro_corr.x) + (rho * (1.0f * in.imuMeasurement.accelerometer.y / abs(gravity)));
  estPitch = (1.0f - rho) * (estPitch + dt*rateGyro_corr.y) + (rho * (-1.0f * in.imuMeasurement.accelerometer.x / abs(gravity)));
  estYaw = estYaw + dt*rateGyro_corr.z;

  outVals.telemetryOutputs_plusMinus100[0] = estRoll;
  outVals.telemetryOutputs_plusMinus100[1] = estPitch;
  outVals.telemetryOutputs_plusMinus100[2] = estYaw;

  // CODE ADDED (11/10)
   //height estimator:
   //prediction step:
   estHeight = estHeight + estVelocity_3 * dt;
   estVelocity_3 = estVelocity_3 + 0 * dt; //assume constant

   //correction step:
   float const mixHeight = 0.3f;  // CONST 0.3f
   if (in.heightSensor.updated) {
     //check that the measurement is reasonable
     if (in.heightSensor.value < 5.0f) {
       float hMeas = in.heightSensor.value * cosf(estRoll) * cosf(estPitch);
       estHeight = (1.0f - mixHeight) * estHeight + mixHeight * hMeas;

       float v3Meas = (hMeas - lastHeightMeas_meas) / (in.currentTime - lastHeightMeas_time);

       estVelocity_3 = (1.0f - mixHeight) * estVelocity_3 + mixHeight * v3Meas;

       //store measurement for next velocity update
       lastHeightMeas_meas = hMeas;
       lastHeightMeas_time = in.currentTime;
     }
   }

   //prediction
   //assuming velocity is constant
   estVelocity_1 = estVelocity_1 + 0 * dt;
   estVelocity_2 = estVelocity_2 + 0 * dt;

   //correction step:
   const float mixHorizVel = 0.1f;   // CONST 0.1f
   if (in.opticalFlowSensor.updated) {
     float sigma_1 = in.opticalFlowSensor.value_x;
     float sigma_2 = in.opticalFlowSensor.value_y;

     float div = (cosf(estRoll) * cosf(estPitch));
     if (div > 0.5f) {
       float deltaPredict = estHeight / div; //delta in eqn

       float v1Meas = (-sigma_1 + in.imuMeasurement.rateGyro.y) * deltaPredict;
       float v2Meas = (-sigma_2 + in.imuMeasurement.rateGyro.x) * deltaPredict;

       estVelocity_1 = (1.0f - mixHorizVel) * estVelocity_1 + mixHorizVel * v1Meas;
       estVelocity_2 = (1.0f - mixHorizVel) * estVelocity_2 + mixHorizVel * v2Meas;
     }
   }

   //time constant
   const float timeConst_horizVel = 1.6f;  // CONST 2.0f

   //control code
   float desAcc1 = -(1.0f / timeConst_horizVel) * estVelocity_1;
   float desAcc2 = -(1.0f / timeConst_horizVel) * estVelocity_2;

   float desRoll = -desAcc2 / gravity;
   float desPitch = desAcc1 / gravity + 0.0015; // + 0.002
   float desYaw = 0;

   //low height
   const float desHeight = 1.6f;  // 1.6 CONST
   const float desAcc3 = -2 * dampingRatio_height * natFreq_height * estVelocity_3 - natFreq_height * natFreq_height * (estHeight - desHeight);

   float desNormalizedAcceleration = (gravity + desAcc3) / (cosf(estRoll) * cosf(estPitch));


  // CODE ADDED (10/27)
  // constants
  float const timeConstant_rollRate = 0.022f; // 0.04[s]
  float const timeConstant_pitchRate = timeConstant_rollRate;
  float const timeConstant_yawRate = 0.1f; // 0.1[s]
  float const timeConstant_rollAngle = 0.12; // 0.12[s]
  float const timeConstant_pitchAngle = timeConstant_rollAngle;
  float const timeConstant_yawAngle = 0.2f; // 0.2[s]

  // values
//  float c_sum = 8.0f; // Desired acceleration [m/s^2]
  Vec3f cmdAngAcc = Vec3f(0, 0, 0); // [rad/s^2]
  Vec3f cmdAngVel = Vec3f(0, 0, 0); // [rad/s]
//  Vec3f desAngPos = Vec3f(0, 0, 0); // [rad]

  // Angular Velocity calculated from current and desired angular position
  cmdAngVel.x = (-1.0f / timeConstant_rollAngle) * (estRoll - desRoll);
  cmdAngVel.y = (-1.0f / timeConstant_pitchAngle) * (estPitch - desPitch);
  cmdAngVel.z = (-1.0f / timeConstant_yawAngle) * (estYaw - desYaw);

  // Angular Acceleration calculated from measured and calculated angular velocity
  cmdAngAcc.x = (-1.0f / timeConstant_rollRate) * (rateGyro_corr.x - cmdAngVel.x);
  cmdAngAcc.y = (-1.0f / timeConstant_pitchRate) * (rateGyro_corr.y - cmdAngVel.y);
  cmdAngAcc.z = (-1.0f / timeConstant_yawRate) * (rateGyro_corr.z - cmdAngVel.z);

  // Mixer Matrix calculations
  c1 = (0.25f) * ((mass * desNormalizedAcceleration) + ((1.0f/l) * (inertia_xx * cmdAngAcc.x)) + ((-1.0f/l) * (inertia_yy * cmdAngAcc.y)) + ((1.0f/k) * (inertia_zz * cmdAngAcc.z)));
  c2 = (0.25f) * ((mass * desNormalizedAcceleration) + (-1.0f/l) * (inertia_xx * cmdAngAcc.x) + (-1.0f/l) * (inertia_yy * cmdAngAcc.y) + (-1.0f/k) * (inertia_zz * cmdAngAcc.z));
  c3 = (0.25f) * ((mass * desNormalizedAcceleration) + (-1.0f/l) * (inertia_xx * cmdAngAcc.x) + (1.0f/l) * (inertia_yy * cmdAngAcc.y) + (1.0f/k) * (inertia_zz * cmdAngAcc.z));
  c4 = (0.25f) * ((mass * desNormalizedAcceleration) + (1.0f/l) * (inertia_xx * cmdAngAcc.x) + (1.0f/l) * (inertia_yy * cmdAngAcc.y) + (-1.0f/k) * (inertia_zz * cmdAngAcc.z));

  outVals.motorCommand1 = pwmCommandFromSpeed(speedFromForce(c1));
  outVals.motorCommand2 = pwmCommandFromSpeed(speedFromForce(c2));
  outVals.motorCommand3 = pwmCommandFromSpeed(speedFromForce(c3));
  outVals.motorCommand4 = pwmCommandFromSpeed(speedFromForce(c4));
  // CODE ADDED (9/29)
  if (in.userInput.buttonBlue) {  // conditional statement to read when user presses blue button
    outVals.motorCommand1 = pwmCommandFromSpeed(speedFromForce(c1));
    outVals.motorCommand2 = pwmCommandFromSpeed(speedFromForce(c2));
    outVals.motorCommand3 = pwmCommandFromSpeed(speedFromForce(c3));
    outVals.motorCommand4 = pwmCommandFromSpeed(speedFromForce(c4));
  }

  // CODE ADDED (11/30)
  if (in.userInput.buttonYellow) {
    Vec3f estGyroBias = Vec3f(0, 0, 0);
    Vec3f rateGyro_corr = Vec3f(0, 0, 0);

    float estRoll = 0.0f;
    float estPitch = 0.0f;
    float estYaw = 0.0f;

    float estHeight = 0;
    float estVelocity_1 = 0;
    float estVelocity_2 = 0;
    float estVelocity_3 = 0;

    float lastHeightMeas_meas = 0;
    float lastHeightMeas_time = 0;

  }

  if (in.userInput.buttonGreen) {
    outVals.motorCommand1 = pwmCommandFromSpeed(speedFromForce(c1 * 0.48));
    outVals.motorCommand2 = pwmCommandFromSpeed(speedFromForce(c2 * 0.48));
    outVals.motorCommand3 = pwmCommandFromSpeed(speedFromForce(c3 * 0.48));
    outVals.motorCommand4 = pwmCommandFromSpeed(speedFromForce(c4 * 0.48));
  }

  // after you’ve computed the command angular acc:
//  outVals.telemetryOutputs_plusMinus100[3] = cmdAngAcc.x;
//  outVals.telemetryOutputs_plusMinus100[4] = cmdAngAcc.y;
//  outVals.telemetryOutputs_plusMinus100[5] = cmdAngAcc.z;
//  outVals.telemetryOutputs_plusMinus100[6] = cmdAngVel.x;
//  outVals.telemetryOutputs_plusMinus100[7] = cmdAngVel.y;
//  outVals.telemetryOutputs_plusMinus100[8] = cmdAngVel.z;
//  outVals.telemetryOutputs_plusMinus100[9] = desAngPos.y;

  outVals.telemetryOutputs_plusMinus100[3] = estVelocity_1;
  outVals.telemetryOutputs_plusMinus100[4] = estVelocity_2;
  outVals.telemetryOutputs_plusMinus100[5] = estVelocity_3;
  outVals.telemetryOutputs_plusMinus100[6] = estHeight;
  outVals.telemetryOutputs_plusMinus100[7] = desRoll;
  outVals.telemetryOutputs_plusMinus100[8] = desPitch;
  outVals.telemetryOutputs_plusMinus100[9] = desNormalizedAcceleration;

  //copy the inputs and outputs:
  lastMainLoopInputs = in;
  lastMainLoopOutputs = outVals;

  return outVals;
}

void PrintStatus() {
  //For a quick reference on the printf function, see: http://www.cplusplus.com/reference/cstdio/printf/
  // Note that \n is a "new line" character.
  // Also, note that to print a `float` variable, you have to explicitly cast it to
  //  `double` in the printf function, and explicitly specify precision using something
  //  like %6.3f (six significant digits, three after the period). Example:
  //   printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));

  //Testing
//  printf("Test Val: %6.3f",float());
//  printf("\n");

  //Sensor check
  printf("Last range = %6.3fm, ",
         double(lastMainLoopInputs.heightSensor.value));
  printf("Last flow: x = %6.3f, y = %6.3f \n ",
         double(lastMainLoopInputs.opticalFlowSensor.value_x),
         double(lastMainLoopInputs.opticalFlowSensor.value_y));

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

  printf("Example variable values:\n");
  printf("  exampleVariable_int = %d\n", exampleVariable_int);
  //Note that it is somewhat annoying to print float variables.
  //  We need to cast the variable as double, and we need to specify
  //  the number of digits we want (if you used simply "%f", it would
  //  truncate to an integer.
  //  Here, we print 6 digits, with three digits after the period.
  printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));

  //We print the Vec3f by printing it's three components independently:
  printf("  exampleVariable_Vec3f = (%6.3f, %6.3f, %6.3f)\n",
         double(exampleVariable_Vec3f.x), double(exampleVariable_Vec3f.y),
         double(exampleVariable_Vec3f.z));

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
