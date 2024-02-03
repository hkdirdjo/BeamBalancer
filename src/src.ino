// Acknowledgements: Based on Brett Beauregard's PID library after I could not get it working.

#include <HCSR04.h>
#include <Servo.h>
#include <PID_v1.h>
#include <RunningAverage.h>

// Function Declarations
// ... //

// Definitions

// Variables
double r, y, e, x; 
double eInt = 0, eDiff = 0, ePrev = 0;
static double integralMaxWindup = 80;
static double maxOutput = 20;
static int nominalActuatorAngle = 95; //degrees
double Kp = -1.1, Ki = -0.3, Kd = 0;
unsigned long currentTime, lastTime = 0;
static unsigned long sampleTime = 10; //ms

// HCSR04 Operations Manual recommends at least a 60ms measurement cycle, which translates to ~16Hz. 
// However, because this application is at maximum 25cm = 1.45ms, we'll be able to get away with less.

// Objects
RunningAverage LowPassYFilter(5);
RunningAverage LowPassRFilter(5);
Servo Actuator;
HCSR04 Sensors(3, new int[2]{5,4}, 2); //initialisation class HCSR04 (trig pin , echo pins)
// Setpoint Sensor 5
// Close Loop Sensor is on Pin 4

void setup() {
  Actuator.attach(9);
  Actuator.write(nominalActuatorAngle);
  
  r = 15;
  Serial.begin(9600);
}

void loop() {
  currentTime = millis();
  double deltaTime = (double)(currentTime - lastTime); //in milliseconds
  
  if(deltaTime < sampleTime) {
    return;
  }
  //Serial.println(deltaTime);

  // LowPassRFilter.addValue(Sensors.dist(0));
  LowPassYFilter.addValue(Sensors.dist(1));
  // r = LowPassRFilter.getAverage();
  y = LowPassYFilter.getAverage();
  
  e = r - y;
  eInt += e*(deltaTime/1000);
  // Anti-windup
  eInt = constrain(eInt,-integralMaxWindup,integralMaxWindup);
  // Serial.println(eInt);
  eDiff = (e - ePrev)/(deltaTime/1000);
  // Serial.println(eDiff);

  x = (Kp*e + Ki*eInt + Kd*eDiff);

  // Clamp the outputs
  x = constrain(x,-maxOutput,maxOutput);

  // Remember these for the next loop
  ePrev = e;
  lastTime = currentTime;

  Actuator.write(x + nominalActuatorAngle);
  //Serial.println((x + nominalActuatorAngle));

  Serial.print("r:");
  Serial.print(r);
  Serial.print(",");
  Serial.print("y:");
  Serial.print(y);
  Serial.print(",");
  Serial.print("x:");
  Serial.print(x);
  Serial.print(",");
  Serial.print("e:");
  Serial.print(e);
  Serial.println(",");
}

