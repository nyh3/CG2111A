
// Ultrasonic sensor pins
#define TRIGPINF A0
#define ECHOPINF A1
#define TRIGPINB A2
#define ECHOPINB A3

void ultrasetup() {
  // Sets the TRIGPIN as an Output
  pinMode(TRIGPINF, OUTPUT); 
  pinMode(TRIGPINB, OUTPUT); 
  // Sets the ECHOPIN as an Input
  pinMode(ECHOPINF, INPUT); 
  pinMode(ECHOPINB, INPUT); 
  
}

bool too_close_forward() {
  long duration;
  long distance;
  // Clears the TRIGPIN
  digitalWrite(TRIGPINF, LOW);
  delayMicroseconds(2);
  // Sets the TRIGPIN on HIGH state for 10 microseconds
  digitalWrite(TRIGPINF, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPINF, LOW);
  // Reads the ECHOPIN, returns the soundwave travel time in microseconds
  duration = pulseIn(ECHOPINF, HIGH);
  // Calculating the distance
  distance = (duration * 0.034) / 2;
  if (distance <= 5) {
    return true;
  } else {
    return false;;
  }
}

bool too_close_backwards() {
  long duration;
  long distance;
  // Clears the TRIGPIN
  digitalWrite(TRIGPINB, LOW);
  delayMicroseconds(2);
  // Sets the TRIGPIN on HIGH state for 10 microseconds
  digitalWrite(TRIGPINB, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPINB, LOW);
  // Reads the ECHOPIN, returns the soundwave travel time in microseconds
  duration = pulseIn(ECHOPINB, HIGH);
  // Calculating the distance
  distance = (duration * 0.034) / 2;
  if (distance <= 5) {
    return true;
  } else {
    return false;;
  }
}
