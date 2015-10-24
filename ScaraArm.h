#ifndef ScaraArm_H
#define ScaraArm_H

#include "Parser.h"
#include "Joint.h"

class ScaraArm : public GCodeProcessor
{
public:
  // Angular values for common angles
  static const float STEP_ANGLE      = PI / 360;
  static const float RIGHT_ANGLE     = PI / 2;
  static const float STRAIGHT_ANGLE  = PI;
  static const float FULL_ROTATION   = 2.0 * PI;
  
  static const float DEG2RAD         = PI/180.0f;
  static const float RAD2DEG         = 180.0f/PI;
  
  // Joints hold the position, but also require setting scalling parameters.
  Joint _shoulder;
  Joint _elbow;

private:

  // Size of robot bones in consistent units (mm recommended).
  int _humerus;
  int _ulna;
  int _humerusSq;
  int _ulnaSq;

  // Coordinate of pen tip in Cartesian space.
  int _x, _y;

  // Scara arms don't work well close to the origin. It's best to offset the
  // work surface origin away from the pillar the arm is resting upon.
  int _xOffset;
  int _yOffset;
  
  // Feed rate delay.
  int _feedRateDelay;

public:
  /**
   * Constructor used to initialize arm parameters.
   * @param humerus - shoulder-to-elbow "bone" in mm
   * @param ulna - elbow-to-wrist "bone" in mm
   * @param xOffset - amount to move coordinates away from pillar.
   * @param yOffset - amount to move away from the pillar.
   * @param feedRateDelay - amount of milliseconds to pause.
   */
  ScaraArm(int humerus, int ulna, int xOffset, int yOffset, int feedRateDelay)
  {
    _humerus = humerus;
    _ulna = ulna;

    /* pre-calculations */
    _humerusSq = _humerus * _humerus;
    _ulnaSq = _ulna * _ulna;
    
    _xOffset = xOffset;
    _yOffset = yOffset;
    
    _feedRateDelay = feedRateDelay;
  }

  /**
   * Park - Move the arm servos to the parked position.  Called during start and stop.
   * By parking the arm the servos will start from a known position and minimize startup
   * lurch.
   */
  void park()
  {
    setPosition(50, 50);
  }
  
  /**
   * setFeedrate - used to set the number of interpolated points which effects
   * servo ramping speed.
   */
  void setFeedrate(float f)
  {
    // delay ms/mm = Constant ms/min / feedrate mm/min
    _feedRateDelay = f/60000; 
  }
  
  /**
   * setHome - used to set a location as the origin for future calculations.
   * Useful to translate a set of motions relative to a new location.
   */
  void setHome(float x, float y, float z, float a, float b, float c)
  {
    _xOffset = (int)x;
    _yOffset = (int)y;
  }

  /**
   * setPosition : Arm positioning routine utilizing inverse kinematics.  Since the arm
   * is resting on a surface Z can only be positive.  Servo movement constraints prevent
   * y from being negative, But X can be a signed value.
   * Note: This must be called before and of the move routines to initialize arm state.
   * @param x - the side to side displacement.
   * @param y - the distance out from the base center.
   */
  void setPosition( int x, int y )
  {
    // Save the Cartesian space coordinates.
    _x = x;
    _y = y;
    
    // Move the origin by the offset.
    x = x + _xOffset;
    y = y + _yOffset;

    // Use Pythagorean theorem to calculate shoulder to wrist distance.
    int s_w = ( x * x ) + ( y * y );
    float s_w_sqrt = sqrt( s_w );

    // s_w angle to centerline
    float a1 = atan2( y, x );

    // s_w angle to humerus.
    float q = (float)(_humerusSq - _ulnaSq + s_w) / (2.0 * _humerus * s_w_sqrt);

    // if > 1 or < -1 the result would be NAN which means point is out of range.
    if (q > 1 || q < -1)
    {
      return;
    }

    float a2 = acos(q);

    // shoulder angle. Note there are two solutions for a right or
    // left arm.  We're using the solution for the right arm.
    float shoulderRads = a1 - a2;

    // Set the shoulder angle.
    setShoulder(shoulderRads);

    // elbow angle
    float elb_angle_r = acos((float)(_humerusSq + _ulnaSq - s_w) / ( 2.0 * _humerus * _ulna ));

    // Right arm solution requires coordinate rotation to use oblique (or negative) elbow angles.
    elb_angle_r = FULL_ROTATION - elb_angle_r;

    // Set the joints
    setElbow(elb_angle_r);
  }

  /**
   * SetY : Arm positioning routine utilizing inverse kinematics.  Moves the arm from
   * the current Y coordinate to the newY passed in.  It maintains all other position
   * state.
   * @param newY - the new forward/back displacement
   */
  void setY(int newY)
  {
    setPosition(_x, newY);
  }

  /**
   * SetX : Arm positioning routine utilizing inverse kinematics.  Moves the arm from
   * the current X coordinate to the newX passed in.  It maintains all other position
   * state.
   * @param newX - the new height above the table
   */
  void setX(int newX)
  {
    setPosition(newX, _y);
  }

  float getX()
  {
    return _x;
  }
    
  float getY()
  {
    return _y;
  }

  void setPosition( float x, float y, float z, float a, float b, float c)
  {
    setPosition((int)x, (int)y);
  }
  
  void moveY(int y)
  {
    int inc = (y > _y) ? 1 : -1;
    
    while (_y != y)
    {
      setPosition(_x, _y + inc);
      delay(_feedRateDelay);
    }
  }
  
  float getSlope(int x, int y)
  {
    float deltaY = y - _y;
    float deltaX = abs(x - _x);
    
    return deltaY / deltaX;
  }

  void movePosition(float x, float y, float z, float a, float b, float c)
  {
    int xLimit = (int)x;
    if (xLimit == _x)
    {
      moveY((int)y);
    }
    else
    {
      float slope = getSlope(x, y);
      int inc = (xLimit > _x) ? 1 : -1;
      a = _y;
      while(_x != xLimit)
      {
        a += slope;
        setPosition(_x + inc, a);
        delay(_feedRateDelay);
      }
    }
  }

  // unused gcode parser callbacks.
  float getZ() { return 0; }
  float getA() { return 0; }
  float getB() { return 0; }
  float getC() { return 0; }

  void enableVacuum(boolean enable)
  {
  }

  /**
   * setShoulder: Sets the should angle member, computes the servo pulse width for that
   * angle, and sets the associated servo.
   * @param shoulderAngleRads - 32-bit floating point radian value
   */
  void setShoulder(float shoulderAngleRads)
  {
    // Rotate the coordinate system by a right angle so that a straight angle is
    // a full extension of the joint.
    _shoulder.setPosition(/*RIGHT_ANGLE -*/ shoulderAngleRads);
  }

  /**
   * SetElbow: Sets elbow angle data member, computes the servo pulse width for that
   * angle, and sets the associated servo.
   * @param elbowAngle - floating point radian value
   */
  void setElbow(float elbowAngle)
  {
    // Rotate the coordinate system so that a straight angle is
    // a full extension of the joint and 2 PI is joint closed.
    _elbow.setPosition(elbowAngle - STRAIGHT_ANGLE);
  }
};

#endif  // ScaraArm_H

//------------------------------------------------------------------------------
// Copyright (C) 2013-2015 Martin Heermance (mheermance@gmail.com)
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

