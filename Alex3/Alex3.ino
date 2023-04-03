#include <serialize.h>
#include <math.h>
#include <stdarg.h>

#include "packet.h"
#include "constants.h"

typedef enum
{
  STOP=0,
  FORWARD=1,
  BACKWARD=2,
  LEFT=3,
  RIGHT=4
} TDirection;

volatile TDirection dir = STOP;

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      92

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.42

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  10   // Left forward pin
#define LR                  11   // Left reverse pin
#define RF                  6  // Right forward pin
#define RR                  5  // Right reverse pin

#define pin_2 0b100
#define pin_3 0b1000

//pins for colour sensor
#define S0 7
#define S1 8
#define S2 12
#define S3 13
#define sensorOut 4

#define PI 3.141592654

#define ALEX_LENGTH 24.6
#define ALEX_BREADTH 15

//Alex's diagonal, calculated once on setup
float AlexDiagonal = 0.0;

//Alex's turning circumfeence, calculated once on setup
float AlexCircum = 0.0;
/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;

//variables to keep track of turning angle
volatile unsigned long deltaTicks;
volatile unsigned long targetTicks;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

//volatile unsigned long int paperColour;

unsigned long deltaDist;
unsigned long newDist;

////colour sensor parameters
//int redMin = 93; // Red minimum value
//int redMax = 280; // Red maximum value
//int greenMin = 111; // Green minimum value
//int greenMax = 370; // Green maximum value
//int blueMin = 88; // Blue minimum value
//int blueMax = 300; // Blue maximum value
//
//// Variables for Color Pulse Width Measurements
// 
//int redPW = 0;
//int greenPW = 0;
//int bluePW = 0;
// 
//// Variables for final Color values
// 
//int redValue;
//int greenValue;
//int blueValue;

/*
 * 
 * Alex Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_RESPONSE;
  messagePacket.command=RESP_STATUS;
  messagePacket.params[0]= leftForwardTicks;
  messagePacket.params[1]= rightForwardTicks;
  messagePacket.params[2]= leftReverseTicks;
  messagePacket.params[3]= rightReverseTicks;
  messagePacket.params[4]= leftForwardTicksTurns;
  messagePacket.params[5]= rightForwardTicksTurns;
  messagePacket.params[6]= leftReverseTicksTurns;
  messagePacket.params[7]= rightReverseTicksTurns;
  messagePacket.params[8]= forwardDist;
  messagePacket.params[9]= reverseDist;
  sendResponse(&messagePacket);
}

void setupColour()
{
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  
  // Set Sensor output as input
  pinMode(sensorOut, INPUT);
  
  // Set Frequency scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
}
//
//// Function to read Red Pulse Widths
//int getRedPW() {
// 
//  // Set sensor to read Red only
//  digitalWrite(S2,LOW);
//  digitalWrite(S3,LOW);
//  // Define integer to represent Pulse Width
//  int PW;
//  // Read the output Pulse Width
//  PW = pulseIn(sensorOut, LOW);
//  // Return the value
//  return PW;
// 
//}
// 
//// Function to read Green Pulse Widths
//int getGreenPW() {
// 
//  // Set sensor to read Green only
//  digitalWrite(S2,HIGH);
//  digitalWrite(S3,HIGH);
//  // Define integer to represent Pulse Width
//  int PW;
//  // Read the output Pulse Width
//  PW = pulseIn(sensorOut, LOW);
//  // Return the value
//  return PW;
// 
//}
// 
//// Function to read Blue Pulse Widths
//int getBluePW() {
// 
//  // Set sensor to read Blue only
//  digitalWrite(S2,LOW);
//  digitalWrite(S3,HIGH);
//  // Define integer to represent Pulse Width
//  int PW;
//  // Read the output Pulse Width
//  PW = pulseIn(sensorOut, LOW);
//  // Return the value
//  return PW;
// 
//}
//
//
//int getColour() {
//  // Read Red value
//  int color;
//  for (int i = 0; i < 4; i ++){
//  redPW = getRedPW();
//  // Map to value from 0-255
//  redValue = map(redPW, redMin,redMax,255,0);
//  // Delay to stabilize sensor
//  delay(200);
//  
//  // Read Green value
//  greenPW = getGreenPW();
//  // Map to value from 0-255
//  greenValue = map(greenPW, greenMin,greenMax,255,0);
//  // Delay to stabilize sensor
//  delay(200);
//  
//  // Read Blue value
//  bluePW = getBluePW();
//  // Map to value from 0-255
//  blueValue = map(bluePW, blueMin,blueMax,255,0);
//  // Delay to stabilize sensor
//  delay(200);
//  if (redValue > greenValue && redValue > blueValue) {
//    color = 0;
//  } else if (greenValue > redValue && greenValue > blueValue) {
//    color = 1;
//  }
//  else {
//  
//    color = 2;
//  }
//  }
////  Serial.println(color);
////  Serial.println("RED: ");  
////  Serial.println(redPW);
////  Serial.println("GREEN: "); 
////  Serial.println(greenPW);
////  Serial.println("BLUE: "); 
////  Serial.println(bluePW);
//  return color;
//
// 
//}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf(char *format, ...) {
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}


void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
// checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  DDRD = (~(pin_2) & ~(pin_3));
  PORTD |= (pin_2|pin_3);
  
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  switch(dir)
  {
  case(FORWARD):
    leftForwardTicks++;
//    Serial.print("LEFT FORWARD: ");
//    Serial.println(leftForwardTicks);
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  break;
  
  case(BACKWARD):
    leftReverseTicks++;
//    Serial.print("LEFT BACKWARD: ");
//    Serial.println(leftReverseTicks);
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  break;
  
  case(LEFT):
    leftReverseTicksTurns++;
//    Serial.print("LEFT LEFTTURN: ");
//    Serial.println(leftReverseTicksTurns);
  break;
  
  case(RIGHT):
    leftForwardTicksTurns++;
//    Serial.print("LEFT RIGHTTURN: ");
//    Serial.println(leftForwardTicksTurns);
  break;
  }
  
}

void rightISR()
{
  switch(dir)
  {
  case(FORWARD):
    rightForwardTicks++;
//    Serial.print("RIGHT FORWARD: ");
//    Serial.println(rightForwardTicks);
  break;
  
  case(BACKWARD):
    rightReverseTicks++;
//    Serial.print("RIGHT BACKWARD: ");
//    Serial.println(rightReverseTicks);
  break;
  
  case(LEFT):
    rightForwardTicksTurns++;
//    Serial.print("RIGHT LEFTTURN: ");
//    Serial.println(rightForwardTicksTurns);
  break;
  
  case(RIGHT):
    rightReverseTicksTurns++;
//    Serial.print("RIGHT RIGHTTURN: ");
//    Serial.println(rightReverseTicksTurns);
  break;
  }
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  EICRA |= 0b00001010;
  EIMSK |= 0b00000011;
  sei();
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.
ISR(INT0_vect)
{
  leftISR();
}

ISR(INT1_vect)
{
  rightISR();
}



// Implement INT0 and INT1 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(9600);
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
  
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count=0;

  while(Serial.available())
    buffer[count++] = Serial. read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
}

/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 10, PB2, OC1B
   *    B2In - pIN 11, PB3, OC2A
   */
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
  
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  if(dist > 0)
      deltaDist = dist;
  else
      deltaDist=9999999;
  
  newDist=forwardDist + deltaDist;
  
  dir = FORWARD;
  
  int val = pwmVal(speed);

  // For now we will ignore dist and move
  // forward indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  
  analogWrite(LF, val);
  analogWrite(RF, val-4);
  analogWrite(LR,0);
  analogWrite(RR, 0);
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  if(dist > 0)
      deltaDist = dist;
  else
      deltaDist=9999999;
  
  newDist=reverseDist + deltaDist;
  
  dir = BACKWARD;
  
  int val = pwmVal(speed);

  // For now we will ignore dist and 
  // reverse indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  analogWrite(LR, val);
  analogWrite(RR, val);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
}

unsigned long computeDeltaTicks (float ang)
{
  unsigned long ticks = (unsigned long)((ang*0.60*AlexCircum*COUNTS_PER_REV)/(360.0*WHEEL_CIRC));
  return ticks;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{

  if(ang > 0)
      deltaTicks = computeDeltaTicks (ang);
  else
      deltaTicks=9999999;
  
  targetTicks=leftReverseTicksTurns + deltaTicks;
  
  dir = LEFT;
  int val = pwmVal(speed);

  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  analogWrite(LR, val);
  analogWrite(RF, val);
  analogWrite(LF, 0);
  analogWrite(RR, 0);
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  
  if(ang > 0)
      deltaTicks = computeDeltaTicks (ang);
  else
      deltaTicks=9999999;
  
  targetTicks=rightReverseTicksTurns + deltaTicks;
  
  dir = RIGHT;
  int val = pwmVal(speed);

  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  analogWrite(RR, val);
  analogWrite(LF, val);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
analogWrite(LF, 0);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  analogWrite(RR, 0);
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks=0;
  leftReverseTicks=0;
  rightForwardTicks=0;
  rightReverseTicks=0;
  leftForwardTicksTurns=0;
  leftReverseTicksTurns=0;
  rightForwardTicksTurns=0;
  rightReverseTicksTurns=0;
  leftRevs=0;
  rightRevs=0;
  forwardDist=0;
  reverseDist=0; 
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((float) command->params[0], (float) command->params[1]);
      break;
      
    case COMMAND_REVERSE:
        sendOK();
        reverse((float) command->params[0], (float) command->params[1]);
      break;
      
    case COMMAND_TURN_LEFT:
        sendOK();
        left((float) command->params[0], (float) command->params[1]);
      break;
      
    case COMMAND_TURN_RIGHT:
        sendOK();
        right((float) command->params[0], (float) command->params[1]);
      break;
      
    case COMMAND_STOP:
        sendOK();
        stop();
      break;    
    case COMMAND_GET_STATS:
        sendOK();
        sendStatus(); 
      break;
    case COMMAND_CLEAR_STATS:
        sendOK();
        clearOneCounter(command->params[0]);
      break;   
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  // put your setup code here, to run once:

  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  setupColour();
  sei();

  AlexDiagonal = sqrt((ALEX_LENGTH*ALEX_LENGTH)+(ALEX_BREADTH * ALEX_BREADTH));
  AlexCircum = PI * AlexDiagonal;
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {

//  paperColour = getColour();

 // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      } 
  if (deltaDist > 0){
      if (dir == FORWARD){
          if (forwardDist > newDist){
              deltaDist = 0;
              newDist = 0;
              stop();
          }
      }

      else if(dir == BACKWARD){
           if (reverseDist > newDist){
              deltaDist = 0;
              newDist = 0;
              stop();
          } 
      }

      else if(dir == STOP){
          deltaDist = 0;
          newDist = 0;
          stop();
      }      
      
  } 
  if (deltaTicks > 0){
      if (dir == LEFT){
          if (leftReverseTicksTurns >= targetTicks){
              deltaTicks = 0;
              targetTicks = 0;
              stop();
          }
      }

      else if(dir == RIGHT){

      if (rightReverseTicksTurns >= targetTicks){
              deltaTicks = 0;
              targetTicks = 0;
              stop();
          } 
      }

      else 
        if(dir == STOP){
          deltaTicks = 0;
          targetTicks = 0;
          stop();
      }      
      
  } 
  
     
}
