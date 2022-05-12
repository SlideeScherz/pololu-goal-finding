/**
 * Name: polulo_goal_finding.ino
 * Created: 4/15/2022 12:59:32 PM
 * Authors: Thomas Diaz-Piedra, Scott Scherzer, Christopher Fioti
 * 
 * All distance calculations done in CM.
 * All trig calculations are expressed in radians
 */

#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;

struct Coordinate {
public:
  double x;
  double y;
  double theta;

  Coordinate(double _x, double _y, double _theta = NULL)
  {
    x = _x;
    y = _y;
    theta = _theta;
  }
};

struct Timer {
public:
  unsigned long t1;
  unsigned long t2;
  unsigned long period;

  Timer(unsigned long _period)
  {
    t1 = 0ul;
    t2 = 0ul;
    period = _period;
  }

  void reset() { t2 = t1; }

  bool ready() 
  { 
    t1 = millis();
    return t1 > t2 + period; 
  }
};

/**
 * debug switches
 * Data outputted in CSV format
 * Disable to boost performance
 * If not using any, set CSV_PERIOD to 10,000 ms
 */

const bool LOC_DEBUG = true;        // localization
const bool ENCODER_DEBUG = false;    // wheel encoders
const bool MOTOR_DEBUG = true;      // wheel motors
const bool PID_DEBUG = true;        // pid and erros

/* scheduler data */

Timer motorTimer(20ul);
Timer encoderTimer(20ul);
Timer csvTimer(50ul);

/* encoder data */

// encoder counts
long countsL = 0l, countsR = 0l;

// previous counts
long prevCountsL = 0l, prevCountsR = 0l;

// distance traveled by wheel in cm
double distL = 0.0, distR = 0.0;

// previous distance traveled
double prevDistL = 0.0, prevDistR = 0.0;

// difference between current and previous distance traveled
double deltaDistL = 0.0, deltaDistR = 0.0;

// change in distance traveled between last 2 intervals
double deltaDistTotal = 0.0;

/* wheel data */

// wheel and encoder constants, turtle edition
const double CLICKS_PER_ROTATION = 12.0;

// const double GEAR_RATIO = 75.81; // turtle edition
const double GEAR_RATIO = 29.86; // standard edition
const double WHEEL_DIAMETER = 3.2;

// cm traveled each gear tick
const double DIST_PER_TICK = (WHEEL_DIAMETER * PI) / (CLICKS_PER_ROTATION * GEAR_RATIO);

// distance between the 2 drive wheels from the center point of the contact patches
const double B = 8.5;

/* position data */

// positional polar coordinates
Coordinate pos(0, 0, 0);

// change in position between last 2 intervals
Coordinate deltaPos(0, 0, 0);

/* goal data */

// index of GOAL array to select which goal to navigate to 
int currentGoal = 0;

// len of GOALS array, how many goals we want to navigate to 
const int NUM_GOALS = 4;

// goal containers
Coordinate goal1(100, 80);
Coordinate goal2(-50, 30);
Coordinate goal3(100, -150);
Coordinate home(0, 0);

Coordinate goals[] = { goal1, goal2, goal3, home };

Coordinate goal = goals[currentGoal];

bool xAccepted = false, yAccepted = false;
bool goalComplete = false, allGoalsComplete = false;

// allow a slight error within this range
const double GOAL_PRECISION = 1.0;

/* motor data */

// speed limits
const int MIN_SPEED = 40, MAX_SPEED = 80;

// speed constants
const double BASE_SPEED = 60.0;

// wheelSpeed containers. Set by PID + Repulsive forces
double speedL = 0.0, speedR = 0.0;

/* PID data */

// max error is about 6
// max diff to apply is MAX - BASE = 20
// 20 = KP * 3.5

const double KP = 27.50;

// suggested PID correction
double gain = 0.0;

// current theta vs theta of goal. Derived from arctan
double angleError = 0.0;

// TODO use for  smooth accel / decel =======================
// starting linear distance from goal. Updated on goal change
//double startDistanceError = 0.0;
// percentage of goal complete. Used to dampen
//double distanceFactor = 0.0;

// current linear distance from goal. Updated on motor period
double distanceError = 0.0;

void setup()
{
  Serial.begin(9600);

  // delete this block if not using robot backwards
  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);
  encoders.flipEncoders(true);

  // init errors
  distanceError = eucDistance(pos, goal);
  angleError = getAngleError(pos, goal);

  delay(1000ul); // dont you run away from me...
  buzzer.play("c32");

  printDebugHeadings();
}

void loop()
{
  if (!allGoalsComplete)
  {
    // calls localize, pid, repulsive forces after
    readEncoders();
    setMotors();

    printDebugData();
  }

  // sleep when done
  else if (allGoalsComplete)
  {
    buzzer.play("c32");
    motors.setSpeeds(0, 0);
    while (true)
    {
      ledGreen(true);
      ledYellow(false);
      delay(1000);
      ledGreen(false);
      ledYellow(true);
    }
  }
}

// util methods

/**
 * @brief Calculate Euclidian distance
 * @return double distance from target points
 */
double eucDistance(Coordinate p1, Coordinate p2)
{
  return sqrt(sq(p2.x - p1.x) + sq(p2.y - p1.y));
}

/**
 * @brief Get the Angle Error of robot vs target
 * @return double error of orientation
 */
double getAngleError(Coordinate p, Coordinate g)
{
  return p.theta - atan2(g.y - p.y, g.x - p.x);
}

/**
 * @brief check max and min for an input value
 * @param input distance, speed, voltage, etc
 * @param min floor value
 * @param max ceiling value
 * @returns int modified value of bounds exceeded
 */
int handleLimit(int input, int min, int max)
{
  if (input <= min)
    input = min;
  else if (input >= max)
    input = max;

  return input;
}

/**
 * @brief check status of goals.
 * @param posC pos array
 * @param goalC goal array
 * @param errorThreshold value to adjust goal
 * @return true both accepted within acceptable goal
 * @return false not within acceptable goal
 */
void checkGoalStatus(Coordinate p, Coordinate g, double errorThreshold)
{
  xAccepted = (g.x - errorThreshold <= p.x) && (g.x + errorThreshold >= p.x);
  yAccepted = (g.y - errorThreshold <= p.y) && (g.y + errorThreshold >= p.y);

  // check completed goal and set status
  goalComplete = (xAccepted && yAccepted);

  if (goalComplete)
  {
    //cycle next goal
    currentGoal++;
    goal = goals[currentGoal];
    buzzer.play("c32");
  }

  // sleep after returning home
  if (currentGoal == NUM_GOALS)
    allGoalsComplete = true;
}

/**
 * @brief Read encoder data to calculate the distance traveled
 * @returns void sets encoder variables for localization calculation
 */
void readEncoders()
{
  if (encoderTimer.ready())
  {
    // read current encoder count
    countsL += encoders.getCountsAndResetLeft();
    countsR += encoders.getCountsAndResetRight();

    // update the distance traveled by each wheel
    distL += (countsL - prevCountsL) * DIST_PER_TICK;
    distR += (countsR - prevCountsR) * DIST_PER_TICK;

    // get change of current and previous distance traveled
    deltaDistL = distL - prevDistL;
    deltaDistR = distR - prevDistR;

    // write previous encoder count
    prevCountsL = countsL;
    prevCountsR = countsR;

    // write previous distance traveled
    prevDistL = distL;
    prevDistR = distR;

    // send encoder data to calculate x,y,theta position
    localize();

    encoderTimer.reset();
  }
}

/**
 * @brief Update the robots current position using wheel encoder data.
 * After localized, will call PID, repulsiveForces and check goal status
 * @param posItr servo position to write to array
 * @returns void updates position
 */
void localize()
{
  // change in distance traveled
  deltaDistTotal = (deltaDistL + deltaDistR) / 2.0;

  // change in orientation
  deltaPos.theta = (deltaDistR - deltaDistL) / B;

  // get polar coordinates of x and y
  deltaPos.x = deltaDistTotal * cos(pos.theta + deltaPos.theta / 2);
  deltaPos.y = deltaDistTotal * sin(pos.theta + deltaPos.theta / 2);

  // update coordinates
  pos.x += deltaPos.x;
  pos.y += deltaPos.y;
  pos.theta += deltaPos.theta;

  // send position data to PID controller to get a correction
  gain = getPID(pos, goal);

  checkGoalStatus(pos, goal, GOAL_PRECISION);
}

/**
 * @brief get a proportionate correction based on current theta vs goal
 * @returns double proportional angle correction
 */
double getPID(Coordinate p, Coordinate g)
{
  // distance error magnitude
  distanceError = eucDistance(p, g);

  // error magnitude: current state - target state
  angleError = getAngleError(p, g);

  return KP * angleError;
}

/**
 * @brief set motor speeds using attractive and repulsive fields.
 * Always add to left, subtract from right to avoid zigzag when crossing y axis
 * 1. gain > 0, gain will be negative when adding (turn right).
 *  - left + (-30) decreasing, right - (-30) increasing
 * 2. gain < 0, gain will be positive when adding (turn left).
 *  - left + (+30) increasing, right - (+30) decreasing
 * @returns void. sets left and right global wheelspeeds.
 */
void setMotors()
{
  if (motorTimer.ready())
  {
    // TODO reset distance factor here

    speedL = (BASE_SPEED + gain);
    speedR = (BASE_SPEED - gain);

    speedL = handleLimit(speedL, MIN_SPEED, MAX_SPEED);
    speedR = handleLimit(speedR, MIN_SPEED, MAX_SPEED);

    // do not adjust regardless of fwd or backwards use
    motors.setSpeeds(speedL, speedR);

    motorTimer.reset();
  }
}

// headings for csv export
void printDebugHeadings()
{
  Serial.print("\n"); // nextline

  // localization
  if (LOC_DEBUG)
  {
    Serial.print("timestamp,");
    Serial.print("X,");
    Serial.print("Y,");
    Serial.print("theta,");
    Serial.print("xGoal,");
    Serial.print("yGoal,");
  }

  // pid
  if (PID_DEBUG)
  {
    Serial.print("dError,");
    Serial.print("aError,");
    Serial.print("gain, ");
  }

  // motors
  if (MOTOR_DEBUG)
  {
    Serial.print("speedL,");
    Serial.print("speedR,");
  }

  // encoders
  if (ENCODER_DEBUG)
  {
    Serial.print("countsL,");
    Serial.print("countsR,");
    Serial.print("prevCountsL,");
    Serial.print("prevCountsL,");
    Serial.print("distL,");
    Serial.print("distR,");
    Serial.print("prevDistL,");
    Serial.print("prevDistR,");
    Serial.print("deltaDistL,");
    Serial.print("deltaDistR,");
    Serial.print("deltaDist,");
  }

  Serial.println(__TIMESTAMP__);
}

// export csv data for plotting and tuning
void printDebugData()
{
  if (csvTimer.ready())
  {
    // current timestamp
    Serial.print(csvTimer.t1);
    Serial.print(",");

    // localization
    if (LOC_DEBUG)
    {
      Serial.print(pos.x);
      Serial.print(",");
      Serial.print(pos.y);
      Serial.print(",");
      Serial.print(pos.theta);
      Serial.print(",");
      Serial.print(goal.x);
      Serial.print(",");
      Serial.print(goal.y);
      Serial.print(",");
    }

    // pid
    if (PID_DEBUG)
    {
      Serial.print(distanceError);
      Serial.print(",");
      Serial.print(angleError);
      Serial.print(",");
      Serial.print(gain);
      Serial.print(",");
    }

    // motors
    if (MOTOR_DEBUG)
    {
      Serial.print(speedL);
      Serial.print(",");
      Serial.print(speedR);
      Serial.print(",");
      //Serial.print(distanceFactor);
      //Serial.print(",");
    }

    // encoders
    if (ENCODER_DEBUG)
    {
      Serial.print(countsL);
      Serial.print(",");
      Serial.print(countsR);
      Serial.print(",");
      Serial.print(prevCountsL);
      Serial.print(",");
      Serial.print(prevCountsR);
      Serial.print(",");
      Serial.print(distL);
      Serial.print(",");
      Serial.print(distR);
      Serial.print(",");
      Serial.print(prevDistL);
      Serial.print(",");
      Serial.print(prevDistR);
      Serial.print(",");
      Serial.print(deltaDistL);
      Serial.print(",");
      Serial.print(deltaDistR);
      Serial.print(",");
      Serial.print(deltaDistTotal);
      Serial.print(",");
    }

    Serial.print("\n");

    csvTimer.reset();
  }
}