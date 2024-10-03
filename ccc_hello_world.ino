//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
// This sketch is a good place to start if you're just getting started with 
// Pixy and Arduino.  This program simply prints the detected object blocks 
// (including color codes) through the serial console.  It uses the Arduino's 
// ICSP SPI port.  For more information go here:
//
// https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:hooking_up_pixy_to_a_microcontroller_-28like_an_arduino-29
//
  
#include <Pixy2.h>
#include<Servo.h>

#define kp 5
#define ki 0
#define kd 5

int servoPin = 5;
Servo myServo;

// For PID
double priError = 0;
double toError = 0;
double minMotorAngle = 0;
double maxMotorAngle = 120;

// This is the main Pixy object 
Pixy2 pixy;

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
  
  pixy.init();

 myServo.attach(servoPin);
}

void loop()
{ 
  //getDistance();

  PID();
}

double convertBallWidthToBallDistance(int width_pixels)
{
  //p2;45;200
  //p1;214;0;off 50mm
  //delta;p1-p2
  double p1, p2, p3, t, offsetP, distance, range;
  p1=214;
  p2=45;
  p3=width_pixels;
  t=(p3-p1)/(p2-p1);
  offsetP=50; //(mm)
  range=200; //(mm)
  distance=(t*range+offsetP)/10;
  //distance=t;


  // // Negative error -> away from camera from center point
  // // Positive error -> closer to camera from center point
  // int max_error_mm = 140;
  // int min_error_mm = -140;
  // Number of pixels ball has when it's at the center point.
  //double center_point_width_pixels = 30; //15;
  //return (width_pixels-center_point_width_pixels);
  return distance; 
}

double getDistance()
{
  double distance = 0; 
  // grab blocks!
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks)
  {
    for (int i=0; i<pixy.ccc.numBlocks; i++)
    {
      // Signature 4 correlates to color yellow
      if (pixy.ccc.blocks[i].m_signature==4)
      {
        // Serial.print("width: ");
        // Serial.print(pixy.ccc.blocks[i].m_width);
        // Serial.print("\n");
        distance = convertBallWidthToBallDistance(pixy.ccc.blocks[i].m_width);
        break;
      }
    }
  }

  Serial.print("distance: ");
  Serial.print(distance);
  Serial.print("\n");

  return distance;
}

void PID() {
  double dis = getDistance();
  int setP = 15;
  double error = setP - dis;

  double Pvalue = error * kp;
  double Ivalue = toError * ki;
  double Dvalue = (error - priError) * kd;

  double PIDvalue = Pvalue + Ivalue + Dvalue;
  priError = error;
  toError += error;
  Serial.println(PIDvalue);
  int Fvalue = (int)PIDvalue;

  // TODO: fix 2nd and 3rd parameter
  Fvalue = map(Fvalue, -maxMotorAngle, maxMotorAngle, minMotorAngle, maxMotorAngle);


  if (Fvalue < minMotorAngle) {
    Fvalue = minMotorAngle;
  }
  if (Fvalue > maxMotorAngle) {
    Fvalue = maxMotorAngle;
  }

  myServo.write(Fvalue);
}



