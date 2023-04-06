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

void ultrasetup() {
  // Sets the TRIGPIN as an Output
  pinMode(TRIGPINF, OUTPUT); 
  pinMode(TRIGPINB, OUTPUT); 
  // Sets the ECHOPIN as an Input
  pinMode(ECHOPINF, INPUT); 
  pinMode(ECHOPINB, INPUT); 
}

int calculate_front() {
  // Clears the TRIGPIN
  digitalWrite(TRIGPINF, LOW);
  delayMicroseconds(2);
  // Sets the TRIGPIN on HIGH state for 10 microseconds
  digitalWrite(TRIGPINF, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPINF, LOW);
  // Reads the ECHOPIN, returns the soundwave travel time in microseconds
  duration_front = pulseIn(ECHOPINF, HIGH);
  // Calculating the distance
  distance_front = (duration_front * 0.034) / 2;
  return distance_front;
}

int calculate_back() {
  // Clears the TRIGPIN
  digitalWrite(TRIGPINB, LOW);
  delayMicroseconds(2);
  // Sets the TRIGPIN on HIGH state for 10 microseconds
  digitalWrite(TRIGPINB, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPINB, LOW);
  // Reads the ECHOPIN, returns the soundwave travel time in microseconds
  duration_back = pulseIn(ECHOPINB, HIGH);
  // Calculating the distance
  distance_back = (duration_back * 0.034) / 2;
  return distance_back;
}
