/*
 Name:		pololu_goal_finding.ino
 Created:	4/7/2022 9:39:32 PM
 Author:	Scott
*/

#include <math.h>
#include <Pololu3piPlus32U4OLED.h>
#include <Pololu3piPlus32U4Motors.h>
#include <Pololu3piPlus32U4LineSensors.h>
#include <Pololu3piPlus32U4LCD.h>
#include <Pololu3piPlus32U4IMU_declaration.h>
#include <Pololu3piPlus32U4IMU.h>
#include <Pololu3piPlus32U4Encoders.h>
#include <Pololu3piPlus32U4Buzzer.h>
#include <Pololu3piPlus32U4Buttons.h>
#include <Pololu3piPlus32U4BumpSensors.h>
#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

Motors motors;
Encoders encoders;
Buzzer buzzer;

// timers
unsigned long t1, t2; 
const unsigned long PERIOD = 20; // poll the eoncoders each 20ms

// motor data =====
const int CLICKS_PER_ROTATION = 12; // gear teeth clicks each time motor makes 1 rotation TURTLE
const float GEAR_RATIO = 75.81f; // turtle edition

// wheel data =====
const float WHEEL_DIAMETER = 3.2f;
const float DIST_PER_TICK = (WHEEL_DIAMETER * PI) / (CLICKS_PER_ROTATION * GEAR_RATIO);

// count of wheel rotations
long countsLeft = 0, countsRight = 0;
long prevLeft = 0, prevRight = 0;

// track wheel distance
float sL = 0.0f, sR = 0.0f;

void setup() {
  Serial.begin(9600);

  //add a delay so user has time before it runs away
  delay(1000);

}

// action to repeat while alive
void loop() {
  
  readEncoders();

}

void readEncoders() {

  //fetch the current time
  t1 = millis();

  //check if enough time has elapsed between polls
  if (t1 > t2 + PERIOD) {

    //reset the wheel encoder register and store it
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    sL += (countsLeft - prevLeft) * DIST_PER_TICK;
    sR += (countsRight - prevRight) * DIST_PER_TICK;

    prevLeft = countsLeft;
    prevRight = countsRight;

    Serial.print(sL);
    Serial.print(" ");
    Serial.println(sR);
    
    t2 = t1;
  }
}



/*
 * set the LEDS to on or off.
 * @param (color)State 0 (off) or 1 (on)
 * @returns void. Sets the pololu LED pins
 */
void setLEDs(int yellowState, int greenState, int redState) {
  ledYellow(yellowState);
  ledGreen(greenState);
  ledRed(redState);
}