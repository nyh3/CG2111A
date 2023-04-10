// TCS3200/230 colour sensor pins
#define S0 8
#define S1 7
#define S2 13
#define S3 9
#define OUT 12
#define FAKE 0
#define RED 1
#define GREEN 2 

/*
  volatile unsigned long WHITE = 0;
  volatile unsigned long RED = 1;
  volatile unsigned long GREEN = 2;
*/

// Stores frequency read by the photodiodes
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

// Stores the RGB values
int red = 0;
int green = 0;
int blue = 0;

// Setup the pins
void setup() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);
  // Setting frequency-scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  Serial.begin(9600);
}

void loop() {
  // Setting red filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  // Reading the output frequency
  redFrequency = pulseIn(OUT, LOW);
  //Serial.print("RF=");
  //Serial.println(redFrequency);
  // Remap the value of redFrequency from 0 to 255
  // map(redFrequency, lowest measured, highest measured, 255, 0)
  red = map(redFrequency, 125, 240, 255, 0);
  // Print the R value
  Serial.print("R = ");
  Serial.println(red);
  delay(100);

  // Setting green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  greenFrequency = pulseIn(OUT, LOW);
  //Serial.print("GF=");
  //Serial.println(greenFrequency);
  // Remap the value of greenFrequency from 0 to 255
  green = map(greenFrequency, 120, 211, 255, 0);
  // Print the G value
  Serial.print("G = ");
  Serial.println(green);
  delay(100);

  // Setting blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  blueFrequency = pulseIn(OUT, LOW);
  //Serial.print("BF=");
  //Serial.println(blueFrequency);
  // Remap the value of blueFrequency from 0 to 255
  blue = map(blueFrequency, 90, 161, 255, 0);
  // Print the B value
  Serial.print("B = ");
  Serial.println(blue);
  delay(100);

  // Check the colour detected
  if (red > 175 && green < 90 && blue < 80) {
    //200, 60, 50
    //220,80,50
    Serial.println("RED");
    //sendcolour(RED);
  } else if(red +50 > green && red < green +10 && 1.6 * blue < green && green>0) {
    //R<70, 60<115, B<70 blueish dark green R*1.4 
    //125<R<145, 140<G<160, 60<B<80 light green
    Serial.println("GREEN");
    //sendcolour(GREEN);
  } else {
    Serial.println("FAKE");
    //sendcolour(FAKE);
  }
}
