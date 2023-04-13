/*=============================================================

  Arduino Hovercraft Control Code

  This code was written for the university course (ENGR 290)
  where an miniature autonomous hovercraft is built.

  The hovercraft has a lift fan and a thrust fan.
  The lift fan is close to the center of the body while the 
  thrust fan is located at the front of the body on top of a
  servo. This servo directs the thrust in the desired direction
  for steering. 

  The sensors used for autonomy include an MPU6050 (IMU),
  two IR sensors and an ultrasonic sensor. The IMU is attached
  to the body of the HC, while the others are attached to the 
  thrust vectoring fan. One IR sensor is placed on each side of
  the fan facing outwards (Left and Right). The ultrasonic sensor
  is facing forwards.

  The thrust vectoring steering control is controlled using a 
  PID controller in reverse. A finite state machine is used 
  to determine the setpoint for the controller depending on
  the conditions the HC is in.
  
=============================================================*/
#include <PID_v1.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Servo.h>

MPU6050 mpu(Wire);

// ------- PID ---------------
double Input;
double Output;
double Setpoint;

bool frontDetect = false;
bool leftWallClose = false;
bool rightWallClose = false;
bool turningStateLeft = false;
bool turningStateRight = false;
int sidewallDistance = 10;
int frontwallDistance = 40;

int YawOffset = 0;

PID mPid_Servo(&Input, &Output, &Setpoint, 1, 0, 0, REVERSE);

// ------- FAN PINS ----------
const int lift_fan_pin = 7;
const int thrust_fan_pin = 6;


// --------- Time variables ------------
unsigned long timer = 0;
unsigned long current_time = 0;

// -------------------- UltraSonic init -----------------------
int distance;
long duration;
const int echoPin = 3;
const int trigPin = 13;
float angle;


// -------IR PINS ------
const uint8_t cPin_IR_L = A0; // P5 PC0
const uint8_t cPin_IR_R = A2; // P14 PC2

// -------------------- SERVO VARIABLE INIT --------------------
Servo myservo;  //servo OBJ

int pos = 0;  // variable to store the servo position
int incomingByte = 0;


// ---------------- TIME CONSTANTS
int TURN_TIME = 4000;
int US_SAMPLE_RATE_MILISECONDS = 300;

// Reference Voltage
const uint8_t cReferenceVoltage = 3.3;
// Variables
int mAnalogValue = 0;
int mDistance = 0;

/*=============================================================

  Initial Setup

=============================================================*/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();
  bool status = mpu.begin(1, 0);
  while (status != 0) {};
  mpu.calcOffsets(true, true);
  mpu.setFilterGyroCoef(0.98);

  mpu.update();
  Input = mpu.getAngleZ() + 90;
  Output = 0;
  Setpoint = 90;
  mPid_Servo.SetMode(AUTOMATIC);
  mPid_Servo.SetSampleTime(10);
  mPid_Servo.SetOutputLimits(-90, 90);

  myservo.attach(9);
  pinMode(lift_fan_pin, OUTPUT);  // LIFT FAN
  pinMode(thrust_fan_pin, OUTPUT);
  //Serial.println("hello");
  // ---------------- Ultrasonic INIT CHANNEL DIR -----------
  pinMode(echoPin, INPUT);  // echo
  pinMode(trigPin, OUTPUT);
  pinMode(cPin_IR_L, INPUT);

  analogWrite(lift_fan_pin, 255);
  analogWrite(thrust_fan_pin, 255);
  delay(2000);
}

/*=============================================================

  Loop function
  
=============================================================*/
void loop() {
  // put your main code here, to run repeatedly:
  mpu.update();
  Input = mpu.getAngleZ() + 90 + YawOffset;
  calculateSetpoint();
  if(mPid_Servo.Compute())
  {
    myservo.write(Output+90);
    Serial.println(Output);
  }
}

/*=============================================================

  Setpoint Selector FSM

  This function chooses which setpoint the Servo PID (thrust fan)
  should target.

  There is a general IMU stabilization state where the fan
  directs itself towards YAW zero in order to go straight.
  This is needed because the lift fan's torque causes unwanted 
  clockwise rotation.

  There is a wall distance correction state where if the HC
  gets too close to a side wall (detected by IR sensors), the
  fan will slightly direct itself away from the approaching wall
  in order to avoid collision and side dragging.

  There is a front wall detection state which leads to a turning
  state. Once a front wall is detected, the code checks on which
  side the wall is closest to the HC, and directs the fan in the 
  opposite direction to initiate a turn. 

  In the turning state, the state becomes locked (meaning that no
  other state can occur) until a full 90 degree rotation of the
  HC's body is completed. This is checked by monitoring when the 
  error between the IMU angle and targeted setpoint is fully 
  corrected. Once that occurs, the FSM goes back to the IMU
  stablizer state.
  
=============================================================*/
void calculateSetpoint(){
  double leftWall = get_IR_distance_L();
  double rightWall = get_IR_distance_R();
  double frontWall = readFromUltrasonic();

  frontDetect = (frontWall < frontwallDistance ? true : false);
  leftWallClose = (leftWall < sidewallDistance ? true : false);
  rightWallClose = (rightWall < sidewallDistance ? true : false);

  if (frontDetect && leftWall < 30 && !turningStateLeft && !turningStateRight)
  {
    turningStateRight = true;
    Setpoint = 0;
    //analogWrite(thrust_fan_pin, 255);
    return;
  }
  else if (frontDetect && rightWall < 30 && !turningStateLeft && !turningStateRight)
  {
    turningStateLeft = true;
    Setpoint = 180;
    //analogWrite(thrust_fan_pin, 255);
    return;
  }
  else if (turningStateLeft)
  { 
    if (Setpoint - Input <= 0)
    {
      turningStateLeft = false;
      YawOffset -= 90;
    }
  }
  else if (turningStateRight)
  {
    if (Setpoint - Input >= 0)
    {
      turningStateRight = false;
      YawOffset += 90;
    }
  }
  else if (leftWallClose)
  {
    Setpoint = 75;
    return;
  }
  else if (rightWallClose)
  {
    Setpoint = 105;
    return;
  }
  else {
    Setpoint = 90;
    return;
  }

}

void controlServo(){
}

double get_IR_distance_L(){
  mAnalogValue = analogRead(cPin_IR_L); 
  mDistance = convertDataToDistance5(mAnalogValue); 
  return mDistance;
}

double get_IR_distance_R(){
  mAnalogValue = analogRead(cPin_IR_R); 
  mDistance = convertDataToDistance5(mAnalogValue); 
  return mDistance;
}

double convertDataToDistance5(double wAnalogVal){ 

  //float voltage = wAnalogVal / 333;

  float voltage = float(wAnalogVal) * cReferenceVoltage/1024; 

  float invD = 27.511* pow(voltage , -1.198); 

  return (invD);  

} 

int readFromUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  return distance;
}