// TCS3200/230 colour sensor pins
#define S0 8
#define S1 7
#define S2 13
#define S3 9
#define OUT 12
#define FAKE 0
#define RED 1
#define GREEN 2 

// Stores frequency read by the photodiodes
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

// Stores the RGB values
int red = 0;
int green = 0;
int blue = 0;

// Setup the pins
void coloursetup() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);
  // Setting frequency-scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
}

void findcolour() {
  // Setting red filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  // Reading the output frequency
  redFrequency = pulseIn(OUT, LOW);
  // Remap the value of redFrequency from 0 to 255
  // map(redFrequency, lowest measured, highest measured, 255, 0)
  red = map(redFrequency, 0, 255, 255, 0);
  // Print the R value
  //Serial.print("R = ");
  //Serial.print(red);
  delay(100);

  // Setting green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  greenFrequency = pulseIn(OUT, LOW);
  // Remap the value of greenFrequency from 0 to 255
  green = map(greenFrequency, 0, 255, 255, 0);
  // Print the G value
  //Serial.print("G = ");
  //Serial.print(green);
  delay(100);

  // Setting blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  blueFrequency = pulseIn(OUT, LOW);
  // Remap the value of blueFrequency from 0 to 255
  blue = map(blueFrequency, 0, 255, 255, 0);
  // Print the B value
  //Serial.print("B = ");
  //Serial.print(blue);
  delay(100);

  // Check the colour detected
  if (red > green) {
    //Serial.println("RED");
    sendcolour(RED);
  } else if(green > red + 25) {
    //Serial.println("GREEN");
    sendcolour(GREEN);
  } else {
    //Serial.println("FAKE");
    sendcolour(FAKE);
  }
}
