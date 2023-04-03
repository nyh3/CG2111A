/*
// Ultrasonic sensor pins
#define trigpin 1
#define echopin 2

bool too_close;

void ultrasetup() {
  pinMode(trigpin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echopin, INPUT); // Sets the echoPin as an Input
}

long mstocm(long microseconds) {
   return microseconds / 29 / 2;
}

void ultrasonic() {
  long duration;
  long distance;

  digitalWrite(trigpin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin, LOW);
  duration = pulseIn(echopin, HIGH);
  distance = mstocm(duration);
  if (distance < 2) {
    too_close = true;
  } else {
    too_close = false;
  }
}
*/
