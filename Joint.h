#ifndef Joint_H
#define Joint_H

#include <math.h>

// float to int conversion with rounding
#define fti(x) ((x) >= 0 ? (int)((x)+0.5) : (int)((x)-0.5)) 

// Defines and manages a single joint of the arm.
class Joint
{
  public:
    /*
      setParameters : configures the joint with a number of measure constants used to position the joint.
      Parameters:
        servo     the configured servo object.
        center    the pulse width which centers the joint.
        widthPerRadian    the pulse width to radian ratio which is signed to handle inverted servos.
     */
    void setParameters(Servo * servo, int center, float widthPerRadian)
    {
      _center = center;
      _widthPerRadian = widthPerRadian;
      _servo = servo;
    }

    /*
      setPosition: Computes the pulse width that matches the desired joint angle
      using the constraints of known angle to pulse values.  Servo center is zero,
      counter clockwise a quarter circle is Pi/2, while clockwise is -Pi/2.

      Parameters:
        angle    32-bit floating point radian value
    */
    void setPosition(float angle)
    {
      _angle = angle;
      int pulseWidth = _center + fti(_widthPerRadian * angle);
      
      _servo->writeMicroseconds(pulseWidth);
    }
    
    float getPosition()
    {
      return _angle;
    }
    
    private:
      float        _angle;
      int          _center;
      float        _widthPerRadian;
      Servo*       _servo;
};

#endif  // Joint_H

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

