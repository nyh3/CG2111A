
// Ultrasonic sensor pins
#define TRIGPINF A0
#define ECHOPINF A1
#define TRIGPINB A3
#define ECHOPINB A2

// Stores the duration read from the front and back ultrasonic
int duration_front;
int duration_back;

// Stores the distance from the front and the back ultrasonic
int distance_front;
int distance_back;

void setup() {
  // Sets the TRIGPIN as an Output
  pinMode(TRIGPINF, OUTPUT); 
  pinMode(TRIGPINB, OUTPUT); 
  // Sets the ECHOPIN as an Input
  pinMode(ECHOPINF, INPUT); 
  pinMode(ECHOPINB, INPUT); 
  Serial.begin(9600);
}

void loop() {
  // Clears TRIGPIN
  digitalWrite(TRIGPINF, LOW);
  //digitalWrite(TRIGPINB, LOW);
  delayMicroseconds(2);
  // Sets TRIGPIN on HIGH state for 10 microseconbds
  digitalWrite(TRIGPINF, HIGH);
  //digitalWrite(TRIGPINB, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPINF, LOW);
  //digitalWrite(TRIGPINB, LOW);
  // Reads the ECHOPIN, returns the soundwave travel time in microseconds
  duration_front = pulseIn(ECHOPINF, HIGH);
  //duration_back = pulseIn(ECHOPINB, HIGH);
  Serial.print("FRONT_TIME = ");
  Serial.println(duration_front);
  //Serial.print("BACK_TIME = ");
  //Serial.println(duration_back);
  // Calculate the distance
  distance_front = (duration_front * 0.034) / 2;
  //distance_back = (duration_back * 0.034) / 2;
  Serial.print("FRONT = ");
  Serial.println(distance_front);
  //Serial.print("BACK = ");
  //Serial.println(distance_back);
}
