/*
 Name:		polulo_goal_finding.ino
 Created:	4/15/2022 12:59:32 PM
 Authors:	Thomas Diaz-Piedra, Scott Scherzer, Christopher Fioti
*/

#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
Motors motors;
Buzzer buzzer;

/* encoder data */

// timers 
unsigned long encodersT1, encodersT2;
const unsigned long ENCODER_PERIOD = 20;

// encoder counts
long countsLeft = 0, countsRight = 0;
long prevLeft = 0, prevRight = 0;

//TODO: make calculation a const
const float CLICKS_PER_ROTATION = 12.0f;
const float GEAR_RATIO = 75.81f;
const float WHEEL_DIAMETER = 3.2f; //TODO unused
const float WHEEL_CIRCUMFRENCE = 10.0531f;

/* location data */

const float B = 8.5f;

// distance before applying dampening to slow down 
const float DAMPEN_RANGE = 20.0f;

// distance traveled by wheel in cm
float sL = 0.0f, sR = 0.0f;

// container to store the previous distance traveled
float prevSL = 0.0f, prevSR = 0.0f;

// difference between current and previous distance traveled
float sL_Delta = 0.0f, sR_Delta = 0.0f;

// position coordinates
float x = 0.0f, y = 0.0f, theta = 0.0f;

// change in position between last 2 intervals
float xDelta = 0.0f, yDelta = 0.0f, thetaDelta = 0.0f;

// change in distance traveled between last 2 intervals
float positionDelta = 0.0f;

/* goal data */

// index of GOAL array to select which goal to navigate to 
int currentGoal = 0;

//len of GOALS array, how many goals we want to navigate to 
const int NUMBER_OF_GOALS = 4;

// goal containers
float xGoals[NUMBER_OF_GOALS] = { 30.0f, -30.0f, 30.0f, 0.0f };
float yGoals[NUMBER_OF_GOALS] = { 15.0f, 0.0f, -15.0f, 0.0f };
float xGoal = xGoals[currentGoal];
float yGoal = yGoals[currentGoal];

// allow a slight error within this range
float goalPrecision = 0.75f;

// starting linear distance from goal. Updated on goal change
float startGoalDistance = sqrt(sq(xGoal - x) + sq(y - yGoal));

// current linear distance from goal. Updated on motor period
float currentGoalDistance = startGoalDistance;

/* motor data */

// speed constants
const float MOTOR_BASE_SPEED = 75.0f, MOTOR_MIN_SPEED = 40.0f, MOTOR_MAX_SPEED = 150.0f;

// init speed to base
float leftSpeed = 0.0f, rightSpeed = 0.0f;

// timers
unsigned long motorT1, motorT2;
const unsigned long MOTOR_PERIOD = 20;

/* PID data */

// proportional error factor
const float KP = 100.0f;

// suggested PID correction object
float PIDCorrection = 0.0f;

// error data
float currentError = 0.0f;

/* debugging switches */
bool bEncoderDebug = false;
bool bPositionDebug = false;
bool bMotorDebug = false;
bool bPIDDebug = false;

void setup()
{
  Serial.begin(9600);
  delay(1000);
}

void loop()
{
  if (currentGoal < NUMBER_OF_GOALS)
  {
    checkEncoders();
    setMotors(PIDCorrection);
    checkGoalStatus();
  }
}

void checkEncoders()
{
  encodersT1 = millis();

  if (encodersT1 > encodersT2 + ENCODER_PERIOD)
  {

    // read current encoder count
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    // write previous distance traveled
    prevSL = sL;
    prevSR = sR;

    // update the distance traveled by each wheel
    sL += ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFRENCE);
    sR += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFRENCE);

    // get change of current and previous distance traveled
    sL_Delta = sL - prevSL;
    sR_Delta = sR - prevSR;

    // write previous encoder count
    prevLeft = countsLeft;
    prevRight = countsRight;

    // reset timer
    encodersT2 = encodersT1;

    // send encoder data to calculate x,y,theta position
    getPosition();
  }
}

void getPosition()
{
  // update distances vs their delta
  positionDelta = (sL_Delta + sR_Delta) / 2.0f;
  thetaDelta = (sR_Delta - sL_Delta) / B;

  xDelta = positionDelta * cos(theta + thetaDelta / 2.0f);
  yDelta = positionDelta * sin(theta + thetaDelta / 2.0f);

  //update the current location
  x += xDelta;
  y += yDelta;
  theta += thetaDelta;
  currentGoalDistance = sqrt(sq(xGoal - x) + sq(y - yGoal));

  if (bPositionDebug) debugPosition();

  //TODO New. Test
  // send position data to PID controller to get a correction
  getPIDCorrection();
}

/**
 * get a proportionate correction based on current theta vs goal
 * a positive currentError will suggest a left turn
 * a negative currentError will suggest a right turn
 * @returns void set PIDcorrection to a proportional angle correction
 */
void getPIDCorrection()
{
  currentError = theta - atan2(yGoal - y, xGoal - x);

  if (bPIDDebug) debugPID();

  PIDCorrection = KP * currentError;
}

/**
 * check status of goals
 * triggered when goalCompleted is set
 * @returns void. selects next goal and resets startGoalDistance
 */
void checkGoalStatus()
{

  bool goalCompleted = false;

  // check completed goal and set status
  if ((xGoal - goalPrecision <= x && xGoal + goalPrecision >= x) && (yGoal - goalPrecision <= y && yGoal + goalPrecision >= y))
    goalCompleted = true;

  // advance to next goal
  if (goalCompleted)
  {
    // uncomment if you want to be annoyed
    //buzzer.play("c32");

    // cycle next goal
    currentGoal++;
    xGoal = xGoals[currentGoal];
    yGoal = yGoals[currentGoal];

    // update start goal distance
    startGoalDistance = sqrt(sq(xGoal - x) + sq(y - yGoal));

    // sleep after returning home
    if (currentGoal == NUMBER_OF_GOALS)
    {
      motors.setSpeeds(0, 0);
      ledGreen(1);
    }
  }
}

/**
 * set motor speeds with PID input
 * @param controllerOutput PID result of angle correction
 * @returns void. sets left and right global wheelspeeds.
 */
void setMotors(float controllerOutput)
{
  motorT1 = millis();

  if (motorT1 > motorT2 + MOTOR_PERIOD)
  {

    leftSpeed = angleController(controllerOutput, 1);
    rightSpeed = angleController(controllerOutput, -1);

    motors.setSpeeds(leftSpeed, rightSpeed);

    if (bMotorDebug) debugMotors();

    motorT2 = motorT1;
  }
}

/**
 * set each wheel individually and apply a dampen force
 * @param speedInput PID controller correction
 * @param polarity positive or negative to adjust direction
 * @returns wheelSpeed speed for the desired wheel
 */
float angleController(float thetaCorrection, int polarity)
{
  // get a theta corrected wheelSpeed
  float wheelSpeed = MOTOR_BASE_SPEED + (thetaCorrection * polarity);

  // reduce wheelspeed if within threshold
  if (currentGoalDistance <= DAMPEN_RANGE)
    wheelSpeed *= (currentGoalDistance / DAMPEN_RANGE);

  //check max and min speed limits
  if (wheelSpeed < MOTOR_MIN_SPEED) wheelSpeed = MOTOR_MIN_SPEED;
  else if (wheelSpeed > MOTOR_MAX_SPEED) wheelSpeed = MOTOR_MAX_SPEED;

  return wheelSpeed;
}

void debugPID()
{
  Serial.print("PID ");
  Serial.print("currentError: ");
  Serial.print(currentError);
  Serial.print(" output: ");
  Serial.println(PIDCorrection);
}

void debugEncoders()
{
  Serial.print("ENC ");
  Serial.print(sL);
  Serial.print(", ");
  Serial.print(sR);

  Serial.print(" sDeltas: ");
  Serial.print(sL_Delta);
  Serial.print(", ");
  Serial.print(sR_Delta);
  Serial.print(" TraveledDist: ");
  Serial.println(positionDelta);
}

void debugMotors()
{
  Serial.print("MOT ");
  Serial.print(leftSpeed);
  Serial.print(", ");
  Serial.println(rightSpeed);
}

void debugPosition()
{
  Serial.print("POS ");
  Serial.print("goal: ");
  Serial.print(xGoal);
  Serial.print(", ");
  Serial.print(yGoal);
  Serial.print(" goalDist: ");
  Serial.print(currentGoalDistance);

  /* uncomment if needed
  Serial.print(" deltas: (");
  Serial.print(xChange);
  Serial.print(", ");
  Serial.print(yChange);
  Serial.print(", ");
  Serial.print(thetaChange);
  Serial.print(")");
  */

  Serial.print(" pos: (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(theta);
  Serial.println(")");
}