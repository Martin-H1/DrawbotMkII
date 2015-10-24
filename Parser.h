#ifndef Parser_H
#define Parser_H

//------------------------------------------------------------------------------
// Parser class - this class is a mashup of some grbl code and my own gcode
// parser. Honestly I've lost track of how much is original versus grbl.
// mheermance@gmail.com
//------------------------------------------------------------------------------
// Copyright at end of file.

#define LINE_BUFFER_SIZE 64

// Define Grbl status codes.
#define STATUS_OK 0
#define STATUS_BAD_NUMBER_FORMAT 1
#define STATUS_EXPECTED_COMMAND_LETTER 2
#define STATUS_UNSUPPORTED_STATEMENT 3
#define STATUS_ARC_RADIUS_ERROR 4
#define STATUS_MODAL_GROUP_VIOLATION 5
#define STATUS_INVALID_STATEMENT 6
#define STATUS_SETTING_DISABLED 7
#define STATUS_SETTING_VALUE_NEG 8
#define STATUS_SETTING_STEP_PULSE_MIN 9
#define STATUS_SETTING_READ_FAIL 10
#define STATUS_IDLE_ERROR 11
#define STATUS_ALARM_LOCK 12
#define STATUS_OVERFLOW 13
#define STATUS_VERSION 14

// define an abstract class to process g code commands.
// The consumer's subclass provide implementation which
// binds the parser to their robot. This allows the reuse
// of the parse with different robot types.

class GCodeProcessor
{
  public:
    // Parks the arm.
    virtual void park() = 0;
    virtual float getX() = 0;
    virtual float getY() = 0;
    virtual float getZ() = 0;
    virtual float getA() = 0;
    virtual float getB() = 0;
    virtual float getC() = 0;
    virtual void setFeedrate(float f);
    virtual void setHome(float x, float y, float z, float a, float b, float c) = 0;
    virtual void setPosition(float x, float y, float z, float a, float b, float c) = 0;
    virtual void movePosition(float x, float y, float z, float a, float b, float c) = 0;
    virtual void enableVacuum(boolean enable);
};

class Parser
{
  public:
    /*
      constructor used to bind the parser to a robot arm.
      Parameters:
        arm  reference to the robot arm.
     */
    Parser(GCodeProcessor *processor)
    {
      _processor = processor;
    }

    /**
     * Prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
     */
    void reset() {
      iter = 0;              // clear input buffer
      Serial.print(F("> ")); // signal ready to receive input
    }

    /**
     * Listen to the serial port for incoming commands and deal with them
     */
    void listen()
    {
      boolean iscomment = false;

      // listen for serial commands
      while(Serial.available() > 0) {
        // Read input when it is available.
        char c = Serial.read();

        // if end of line reached
        if ((c == '\n') || (c == '\r')) {
          if (iter > 0) {// Line is complete. Then execute!
            buffer[iter] = 0; // Terminate string
            reportMessage(processCommand());
          }
          else {
            // Empty or comment line. Skip block.
            reportMessage(STATUS_OK); // Send status message for syncing purposes.
          }

          reset();
        }
        else {
          if (iscomment) {
            // Throw away all comment characters
            if (c == ')') {
              // End of comment. Resume line.
              iscomment = false;
            }
          }
          else {
            if (c <= ' ') {
              // Throw away whitepace and control characters
              // except control x which gets the version string
              if (c == 24)
                reportMessage(STATUS_VERSION);
            }
            else if (c == '/') { 
              // Block delete not supported. Ignore character.
            }
            else if (c == '(') {
              // Enable comments flag and ignore all characters until ')' or EOL.
              iscomment = true;
            }
            else if (iter >= LINE_BUFFER_SIZE-1) {
              // Report line buffer overflow and reset
              reportMessage(STATUS_OVERFLOW);
              reset();
            }
            else if (c >= 'a' && c <= 'z') { // Upcase lowercase
              buffer[iter++] = c-'a'+'A';
            }
            else {
              buffer[iter++] = c;
            }
          }
        }
      }
    }
    
    /*
     *returns a status to the serial line
     */
    void reportMessage(int status_code)
    {
      if (status_code == 0)
      { // STATUS_OK
        Serial.print(F("ok\r\n"));
      }
      else if (status_code == STATUS_VERSION)
      {
        Serial.print(F("Grbl v0.8c ['$' for help]\r\n"));
      }
      else
      {
        Serial.print(F("error: "));
        switch (status_code)
        {          
          case STATUS_BAD_NUMBER_FORMAT:
            Serial.print(F("Bad number format")); break;
          case STATUS_EXPECTED_COMMAND_LETTER:
            Serial.print(F("Expected command letter")); break;
          case STATUS_UNSUPPORTED_STATEMENT:
            Serial.print(F("Unsupported statement")); break;
          case STATUS_ARC_RADIUS_ERROR:
            Serial.print(F("Invalid radius")); break;
          case STATUS_MODAL_GROUP_VIOLATION:
            Serial.print(F("Modal group violation")); break;
          case STATUS_INVALID_STATEMENT:
            Serial.print(F("Invalid statement")); break;
          case STATUS_SETTING_DISABLED:
            Serial.print(F("Setting disabled")); break;
          case STATUS_SETTING_VALUE_NEG:
            Serial.print(F("Value < 0.0")); break;
          case STATUS_SETTING_STEP_PULSE_MIN:
            Serial.print(F("Value < 3 usec")); break;
          case STATUS_SETTING_READ_FAIL:
            Serial.print(F("EEPROM read fail. Using defaults")); break;
          case STATUS_IDLE_ERROR:
            Serial.print(F("Busy or queued")); break;
          case STATUS_ALARM_LOCK:
            Serial.print(F("Alarm lock")); break;
          case STATUS_OVERFLOW:
            Serial.print(F("Line overflow")); break;
        }
      }
      Serial.print(F("\r\n"));
    }

  private:
    GCodeProcessor * _processor;
    char buffer[LINE_BUFFER_SIZE];
    int iter;  

    /** Allows human to enter degrees
     */
    float deg2Rad(float degValue)
    {
      return degValue * PI / 180.0;
    }

    /** trasforms radians for display.
     */
    float rad2Deg(float degValue)
    {
      return degValue * 180.0 / PI;
    }

    /**
     * Extracts the floating point argument from the desire code.
     * @return the value found.  If nothing is found, val is returned.
     * @input code the character to look for.
     * @input val the return value if code is not found.
     **/
    float getArgument(char code, float val) {
      char* ptr = buffer;
      while (ptr && *ptr && ptr < buffer + iter) {
        if (*ptr == code) {
          return atof(ptr + 1);
        }
        ptr++;
      }
      return val;
    }

    /**
     * Read the input buffer and find any recognized commands.  One G or M command per line.
     */
    int processCommand() {
      int cmd = getArgument('G', -1);
      switch(cmd) {
      case  0: // fast linear (use sparingly because of inertia).
        _processor->setPosition( getArgument('X', _processor->getX()), getArgument('Y', _processor->getY()), getArgument('Z', _processor->getZ()),
                           deg2Rad( getArgument('A', rad2Deg(_processor->getA())) ),
                           deg2Rad( getArgument('B', rad2Deg(_processor->getB())) ),
                           deg2Rad( getArgument('C', rad2Deg(_processor->getC())) ));
        break;
      case  1: // move linear
        _processor->movePosition( getArgument('X', _processor->getX()), getArgument('Y', _processor->getY()), getArgument('Z', _processor->getZ()),
                           deg2Rad( getArgument('A', rad2Deg(_processor->getA())) ),
                           deg2Rad( getArgument('B', rad2Deg(_processor->getB())) ),
                           deg2Rad( getArgument('C', rad2Deg(_processor->getC())) ));
        break;
      // pause
      case  4:
        delay( getArgument('P', 0) * 1000);
        break;

      // home g code parks the arm
      case 28:
        _processor->park();
        break;

      // don't do anything on an unrecognized code.
      default:
        break;
      }
      
      // Feed rate command used to change end effector interpolation
      cmd = getArgument('F', -1);
      if (cmd != -1)
      {
        _processor->setFeedrate(cmd);
      }

      // Miscellaneous commands used for end effector control.
      cmd = getArgument('M', -1);
      switch (cmd)
      {
        // Motor command is used to enable vacuum system
      case 10:
        _processor->enableVacuum(true);
        break;

      // Motor command is used to disable vacuum system
      case 11:
        _processor->enableVacuum(false);
        break;
        
      case 114:
        Serial.print(F("X="));
        Serial.print(_processor->getX());
        Serial.print(F(", Y="));
        Serial.print(_processor->getY());
        Serial.print(F(", Z="));
        Serial.print(_processor->getZ());
        Serial.print(F(", A="));
        Serial.print(rad2Deg(_processor->getA()));
        Serial.print(F(", B="));
        Serial.print(rad2Deg(_processor->getB()));
        Serial.print(F(", C="));
        Serial.println(rad2Deg(_processor->getC()));
        break;
      case 206:
        _processor->setHome( getArgument('X', _processor->getX()), getArgument('Y', _processor->getY()), getArgument('Z', _processor->getZ()),
                           deg2Rad( getArgument('A', rad2Deg(_processor->getA())) ),
                           deg2Rad( getArgument('B', rad2Deg(_processor->getB())) ),
                           deg2Rad( getArgument('C', rad2Deg(_processor->getC())) ));
        break;

      // don't do anything on an unrecognized code.
      default:
        break;
      }
      
      return STATUS_OK;
    }
};

#endif  // Parser_H

//------------------------------------------------------------------------------
// Copyright (C) 2014-2015 Martin Heermance (mheermance@gmail.com)
// Portions Copyright of the GRBL project.
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

