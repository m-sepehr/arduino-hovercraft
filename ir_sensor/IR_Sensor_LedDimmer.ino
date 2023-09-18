//================================================
// IR Sensor LED Dimmer
//
// This code is used to control the brightness
// of an LED using a Sharp GP2Y0A21 IR sensor
// connected to an Arduino Nano
//================================================

//================================================
// Initial Values
//================================================

// Assigning Pins
const uint8_t cPin_IR = A0;
const uint8_t cPin_LED = 11;

// Variables
int mAnalogValue = 0;
int mDistance = 0;

//================================================
// Initial Setup
//================================================
void setup() {
  // Setting BAUD rate
  Serial.begin(9600);

  // Assigning Pin transmission direction
  pinMode(cPin_IR, INPUT);
  pinMode(cPin_LED, OUTPUT);

}

//================================================
// Loop function
//================================================
void loop() {
  // Reading data from IR sensor and converting to distance
  getSensorData();
  // Controlling LED brightness based on distance (off when > 45cm, variable between 15 and 45 cm, fully on <= 15)
  controlLed();
  // Delay of 60 ms.
  delay(60);
}

//================================================
//getSensorData() reads the analog data coming from
//the IR sensor, converts it to a distance value
//in centimeters and prints it to the console.
//================================================
void getSensorData(){

  mAnalogValue = analogRead(cPin_IR);
  mDistance = convertDataToDistance(mAnalogValue);

  Serial.println(mDistance);

}

//================================================
//convertDataToDistance() takes in the analog value 
//of the sensor and converts it to a voltage value, 
//which is then converted to a distance value in cm, 
//and returned.
//================================================
int convertDataToDistance(int wAnalogVal){

  float wVoltage = float(wAnalogVal) * 5/1024;

  // The multiplication factor can be changed to calibrate the outputted distance against real world measurements.
  return (30 * pow(wVoltage, -1));
}

//================================================
//controlLed() outputs the brightness value to the
//LED depending on the calculated distance value.
//================================================
void controlLed(){
  // If the is less than or equal to 15 cm, set LED to max brightness
  if (mDistance <= 15)
  {
    digitalWrite(cPin_LED, LOW);
  }
  // If the distance is between 15 and 45 cm, control LED brightness by dimming as distance increases
  else if (mDistance > 15 && mDistance <= 45)
  {
    analogWrite(cPin_LED, (255*(mDistance - 15))/30);
  }
  // If the distance is greater than 45 cm, turn LED off
  else
  {
    digitalWrite(cPin_LED, HIGH);
  }
}