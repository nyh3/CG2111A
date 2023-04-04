
// Ultrasonic sensor pins
#define TRIGPIN A0
#define ECHOPIN A1

void ultrasetup() {
  pinMode(TRIGPIN, OUTPUT); // Sets the TRIGPIN as an Output
  pinMode(ECHOPIN, INPUT); // Sets the ECHOPIN as an Input
}

bool too_close() {
  long duration;
  long distance;
  // Clears the TRIGPIN
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);
  // Sets the TRIGPIN on HIGH state for 10 microseconds
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW);
  // Reads the ECHOPIN, returns the soundwave travel yime in microseconds
  duration = pulseIn(ECHOPIN, HIGH);
  // Calculating the distance
  distance = (duration * 0.034) / 2;
  if (distance <= 5) {
    return true;
  } else {
    return false;;
  }
}
