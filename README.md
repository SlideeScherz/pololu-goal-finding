# pololu goal finding

**v1.0.1** 
4/19/2022

[![build](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)

---
## Features

Given any number of waypoints as (x,y,theta) **polar** coordinates, use the wheel encoders and **localization** to determine the robots current position, 
and navigate to it's goal using a **PID Controller**

#### Localization
Begin tracking the wheel encoders at the (0,0) position

Encoder wheel distance tracking:
```cpp
// update the distance traveled by each wheel
sLeftT1 += (countsLeftT1 - countsLeftT2) * DIST_PER_TICK;
sRightT1 += (countsRightT1 - countsRightT2) * DIST_PER_TICK;

// get change of current and previous distance traveled
sLeftDelta = sLeftT1 - sLeftT2;
sRightDelta = sRightT1 - sRightT2;
```

Once encoder data has been updated, update the robots current position

**Distance formula**

![eq](https://latex.codecogs.com/svg.image?\bg{black}{\color{Cyan}distance&space;=&space;\sqrt{(x2&space;-&space;x1)^{2}&space;&plus;&space;(y2&space;-&space;y1)^{2}}})

**For our usage we will implement** 

![eq2](https://latex.codecogs.com/svg.image?\bg{black}{\color{Cyan}goalDistance&space;=&space;\sqrt{(xGoal&space;-&space;xPos)^{2}&space;&plus;&space;(yGoal&space;-&space;yPos)^{2}}})

```cpp
// get linear distance
currentGoalDistance = sqrt(sq(xGoal - x) + sq(y - yGoal));
```

#### PID Controller

Well not really. More so a P controller (proportional error)
Set the error in radians by using ```arctan``` to compare current angle vs the goal.
```cpp
currentError = theta - atan2(yGoal - yPos, xGoal - xPos);
PIDCorrection = KP * currentError;
```  

## Hardware

### Pololu3piPlus32U4 Robot, Turtle Edition

![img](https://a.pololu-files.com/picture/0J11323.600x480.jpg?bf2f67dbe8c5a1035409af8b78b78f97)

Purchase Pololu 3pi+ OLED Robot from [pololu](https://www.pololu.com/product/4976)

## Libraries

### Pololu3piPlus32U4 

Pull and explore [pololu git repository](https://github.com/pololu/pololu-3pi-plus-32u4-arduino-library) and add as a project dependency.

***Pololu Usage***

```cpp
#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

Buzzer buzzer;
Motors motors;
Encoders encoders;
```

## License

### MIT 

Free Software, nice
