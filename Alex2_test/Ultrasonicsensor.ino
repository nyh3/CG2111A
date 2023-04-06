
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
  // Set TRIGPIN as Output
  pinMode(TRIGPINF, OUTPUT); 
  pinMode(TRIGPINB, OUTPUT);
  // Sets ECHOPIN as input
  pinMode(ECHOPINB, INPUT);
  pinMode(ECHOPINB, INPUT);
}

bool too_close() {
  long duration;
  long distance;
  // Clears the TRIGPIN
  digitalWrite(TRIGPINF, LOW);
  delayMicroseconds(2);
  // Sets the TRIGPIN on HIGH state for 10 microseconds
  digitalWrite(TRIGPINF, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPINF, LOW);
  // Reads the ECHOPIN, returns the soundwave travel yime in microseconds
  duration = pulseIn(ECHOPINF, HIGH);
  // Calculating the distance
  distance = (duration * 0.034) / 2;
  if (distance <= 5) {
    return true;
  } else {
    return false;;
  }
}
