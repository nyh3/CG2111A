/*RGB Color Sensor Demonstration
  rgb-color-sensor-demo.ino
  Read RGB values from Color Sensor
  Must use calibration values from Color Sensor Calibration Sketch
 
  DroneBot Workshop 2020
  https://dronebotworkshop.com
*/
 
// Define color sensor pins
 
#define S0 7
#define S1 8
#define S2 12
#define S3 13
#define sensorOut 4
 
// Calibration Values
// Get these from Calibration Sketch
 
int redMin = 130; // Red minimum value
int redMax = 320; // Red maximum value
int greenMin = 110; // Green minimum value
int greenMax = 294; // Green maximum value
int blueMin = 78; // Blue minimum value
int blueMax = 211; // Blue maximum value
 
// Variables for Color Pulse Width Measurements
 
int redPW = 0;
int greenPW = 0;
int bluePW = 0;
 
// Variables for final Color values
 
int redValue;
int greenValue;
int blueValue;

 //TColor
  typedef enum
{
  RED = 0,
  BLUE = 1,
  GREEN = 2,
} TColour;
 
void setup() {
 
  // Set S0 - S3 as outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  
  // Set Sensor output as input
  pinMode(sensorOut, INPUT);
  
  // Set Frequency scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);

 
  
  // Setup Serial Monitor
  Serial.begin(9600);
}
 
void loop() {
  
  // Read Red value
  redPW = getRedPW();
  // Map to value from 0-255
  redValue = map(redPW, redMin,redMax,255,0);
  // Delay to stabilize sensor
  delay(200);
  
  // Read Green value
  greenPW = getGreenPW();
  // Map to value from 0-255
  greenValue = map(greenPW, greenMin,greenMax,255,0);
  // Delay to stabilize sensor
  delay(200);
  
  // Read Blue value
  bluePW = getBluePW();
  // Map to value from 0-255
  blueValue = map(bluePW, blueMin,blueMax,255,0);
  // Delay to stabilize sensor
  delay(200);
  
  // Print output to Serial Monitor
  /*Serial.print("Red = ");
  Serial.print(redValue);
  Serial.print(" - Green = ");
  Serial.print(greenValue);
  Serial.print(" - Blue = ");
  Serial.println(blueValue); */
  TColour color;
  if (redValue > greenValue && redValue > blueValue) {
    color = RED;
  } else if (greenValue > redValue && greenValue > blueValue) {
    color = GREEN;
  } else {
    color = BLUE;
  }
  Serial.println(color);
  
}
 
 
// Function to read Red Pulse Widths
int getRedPW() {
 
  // Set sensor to read Red only
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  // Define integer to represent Pulse Width
  int PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut, LOW);
  // Return the value
  return PW;
 
}
 
// Function to read Green Pulse Widths
int getGreenPW() {
 
  // Set sensor to read Green only
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Define integer to represent Pulse Width
  int PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut, LOW);
  // Return the value
  return PW;
 
}
 
// Function to read Blue Pulse Widths
int getBluePW() {
 
  // Set sensor to read Blue only
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Define integer to represent Pulse Width
  int PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut, LOW);
  // Return the value
  return PW;
 
}
