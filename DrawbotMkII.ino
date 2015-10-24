#include <Arduino.h>
#include <Servo.h>
#include "ScaraArm.h"

int HUMERUS = 103;
int ULNA = 100;

ScaraArm robotArm(HUMERUS, ULNA, 0, 0, 20);

// Create and configure servos here, use dependancy injection to provide them to the joint class.
Servo shoulderServo;
Servo elbowServo;

Parser parser(&robotArm);

// Perform one time setup and initialization.
void setup()
{
  shoulderServo.attach(2, 500, 2500);
  elbowServo.attach(3, 500, 2500);
  robotArm._shoulder.setParameters(&shoulderServo, 995, 560); // 510); // should probably be 560
  robotArm._elbow.setParameters(&elbowServo, 2300, -563);

 // elbow Pi = 2390, 7Pi/8 = 2170, 3Pi/4 = 1956, 5Pi/8 = 1730, Pi/2 = 1500, 3Pi/8 = 1270, Pi/4 = 1050, Pi/8 = 840, 0 = 640

 // shoulder Pi = 685, 7Pi/8 = 882, 3Pi/4 = 1080, 5Pi/8 = 1290, Pi/2 = 1500, 3Pi/8 = 1737, Pi/4 = 1975, Pi/8 = 2200, 0 = 2450

  Serial.begin( 57300 );
  delay( 3000 );

  // Identify as GRBL so I can use the CNC GUI.
  parser.reportMessage(STATUS_VERSION);
  parser.reset();

  robotArm.park();
}

// This is called in a tight loop.
// The parser awaits input and acts upon it.
void loop()
{
  parser.listen();
}

//------------------------------------------------------------------------------
// Copyright (C) 2015 Martin Heermance (mheermance@gmail.com)
/*
┌──────────────────────────────────────────────────────────────────────────┐
│                                                   TERMS OF USE: MIT License                                                   │
├──────────────────────────────────────────────────────────────────────────┤
│Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation     │
│files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,     │
│modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software │
│is furnished to do so, subject to the following conditions:                                                                    │
│                                                                                                                               │
│The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. │
│                                                                                                                               │
│THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE           │
│WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR          │
│COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,    │
│ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                          │
└──────────────────────────────────────────────────────────────────────────┘
*/

