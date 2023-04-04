#include <math.h>

#include <NewPing.h>

#include "packet.h"
#include "serialize.h"
#include "constants.h"

#define DEBUG

#ifdef DEBUG
#include <stdarg.h>
#warning "Compiling with debug output"

// For writing debug messages
void sendMessage(const char *message);

void dbprint(char *format, ...) {
  va_list args;
  char buffer[128];

  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}
#else
void dbprint(char *format, ...) {}
#endif // DEBUG

typedef enum
{
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;

#define SERIAL_BAUD         115200

/**
 *  Physical Constants
 */

// Ticks per revolution from the wheel encoder
#define TICKS_PER_REV   182

// Effectively an infinite distance
#define DIST_MAX        9999999

// Wheel circumference (cm); used to calculate forward/backward distance
#define WHEEL_CIRC      21.5
// Separation between midpoint of wheels (cm)
#define WHEEL_SEP       10
// Circumference of circle traced out by a full rotation of robot
#define ROTATE_CIRC     (PI * WHEEL_SEP)

// Maximal distance for ultrasonic sensor (cm)
#define USONIC_MAX_DIST     400
// Period between each ping (ms)
// Note: The actual period for EACH sensor is twice of this
#define USONIC_PING_PERIOD  100

/**
 *  Pin connections
 */

// Motor control pins
#define LF  6     // Left forward pin
#define LR  5     // Left reverse pin
#define RF  10    // Right forward pin
#define RR  9     // Right reverse pin

// Encoders
#define LENC  3   // Left wheel encoder
#define RENC  2   // Right wheel encoder

// Ultrasonic Sensors
#define TRIG_PIN_L  A0
#define ECHO_PIN_L  A1
#define TRIG_PIN_R  A2
#define ECHO_PIN_R  A3

/**
 *  State Variables
 */

// Encoder ticks for moving forward/backward
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

// Encoder ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Revolution counter for wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// For tracking distance moved since last command
unsigned long deltaDist;
unsigned long newDist;

// For tracking angle turned since last command
unsigned long deltaTicks;
unsigned long leftTargetTicks;
unsigned long rightTargetTicks;

// Ultrasonic sensors
NewPing ultrasonicSensor[2] = {
  NewPing(TRIG_PIN_L, ECHO_PIN_L, USONIC_MAX_DIST),
  NewPing(TRIG_PIN_R, ECHO_PIN_R, USONIC_MAX_DIST)
};

// Distance from ultrasonic sensors (cm)
float ultrasonicDist[2];

// Ultrasonic sensor states for NewPing
uint32_t pingTimer[2];
int ultrasonicIdx;

/**
 *  Packet sending
 */

void sendPacket(TPacket *packet)
{
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

void sendStatus()
{
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;

  // statusPacket.params[0] = leftForwardTicks;
  // statusPacket.params[1] = rightForwardTicks;
  // statusPacket.params[2] = leftReverseTicks;
  // statusPacket.params[3] = rightReverseTicks;  
  // statusPacket.params[4] = leftForwardTicksTurns;
  // statusPacket.params[5] = rightForwardTicksTurns;
  // statusPacket.params[6] = leftReverseTicksTurns;
  // statusPacket.params[7] = rightReverseTicksTurns;
  // statusPacket.params[8] = forwardDist;
  // statusPacket.params[9] = reverseDist;

  sendPacket(&statusPacket);
}

void sendUltrasonic()
{
  TPacket ultrasonicPacket;
  ultrasonicPacket.packetType = PACKET_TYPE_ULTRASONIC;
  ultrasonicPacket.command = RESP_STATUS;

  // Here's a little lesson in trickery -- cast floats as uint32_t
  uint32_t u1, u2;
  memcpy(&u1, &ultrasonicDist[0], sizeof(float));
  memcpy(&u2, &ultrasonicDist[1], sizeof(float));
  ultrasonicPacket.params[0] = u1;
  ultrasonicPacket.params[1] = u2;

  sendPacket(&ultrasonicPacket);
}  

void sendMessage(const char *message)
{ 
  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;

  strncpy(messagePacket.data, message, MAX_STR_LEN);

  sendPacket(&messagePacket);
}

void sendBadPacket()
{  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;

  sendPacket(&badPacket);
}

void sendBadChecksum()
{
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;

  sendPacket(&badChecksum);  
}

void sendBadCommand()
{
  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;

  sendPacket(&badCommand);
}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;

  sendPacket(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;

  sendPacket(&okPacket);  
}

/**
 *  Setup and start codes for external interrupts and 
 *  pullup resistors.
 */

void enablePullups()
{
  // TODO: Use bare metal functions
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  
  // Pullup for wheel encoders
  pinMode(LENC, INPUT_PULLUP);
  pinMode(RENC, INPUT_PULLUP);
}

// Functions to be called by INT0 and INT1 ISRs.
void updateLeftTicks()
{
  if (dir == FORWARD) {
    leftForwardTicks++;
    forwardDist = (unsigned long) ((float) leftForwardTicks / TICKS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == BACKWARD) {
    leftReverseTicks++;
    reverseDist = (unsigned long) ((float) leftReverseTicks / TICKS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == LEFT) {
    leftReverseTicksTurns++;
  }
  else if (dir == RIGHT) {
    leftForwardTicksTurns++;
  }
}

void updateRightTicks()
{
  if (dir == FORWARD) {
    rightForwardTicks++;
  }
  else if (dir == BACKWARD) {
    rightReverseTicks++;
  }
  else if (dir == LEFT) {
    rightForwardTicksTurns++;
  }
  else if (dir == RIGHT) {
    rightReverseTicksTurns++;
  }
}

ISR(INT0_vect) {
  updateLeftTicks();
}

ISR(INT1_vect) {
  updateRightTicks();
}

void setupEINT()
{
  // Falling edge trigger for INT0 and INT1
  EICRA = 0b00001010;
  // Enable INT0 and INT1 interrupts
  EIMSK = 0b00000011;
  
}

/**
 *  Serial communications
 */

//
// Reads in data from serial port and deserialize into TPacket pointer
//
TResult readPacket(TPacket *packet)
{
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if (len == 0) {
      return PACKET_INCOMPLETE;
    }
     
    return deserialize(buffer, len, packet);
}

void setupSerial()
{
  // TODO: To replace later with bare-metal.
  Serial.begin(SERIAL_BAUD);
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.
void startSerial()
{
  // TODO: To be replaced with bare-metal code 
}

//
// Read the serial port. Returns the read character in
// ch if available. 
//  Returns TRUE if ch is valid. 
// 
int readSerial(char *buffer)
{
  int count = 0;
  // TODO: Bare-metal
  while (Serial.available()) {
    buffer[count++] = Serial.read();
  }

  return count;
}

void writeSerial(const char *buffer, int len)
{
  // TODO: Bare-metal
  Serial.write(buffer, len);
}

/**
 *  Alex's motor drivers.
 */

// TODO
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
// TODO
void startMotors()
{
  
}

// 
//  Convert percentages to PWM values
//
int pwmVal(float speed)
{
  // Clamp values
  if (speed < 0.0) return 0;
  if (speed > 100.0) return 255;

  return (int) ((speed / 100.0) * 255.0);
}

//
//  Move forward by dist cm at `speed`% of full speed
//  dist of 0 indicates indefinite movement
//
void forward(float dist, float speed)
{
  deltaDist = (dist > 0) ? dist : DIST_MAX; // Indefinite movement
  newDist = forwardDist + deltaDist;

  dir = FORWARD;
  int val = pwmVal(speed);
  
  // TODO: Bare-metal
  analogWrite(LF, val);
  analogWrite(RF, val);
  analogWrite(LR, 0);
  analogWrite(RR, 0);
}

//
//  Move backwards by dist cm at `speed`% of full speed
//  dist of 0 indicates indefinite movement
//
void reverse(float dist, float speed)
{
  deltaDist = (dist > 0) ? dist : DIST_MAX; // Indefinite movement
  newDist = reverseDist + deltaDist;

  dir = BACKWARD;
  int val = pwmVal(speed);

  // TODO: Bare-metal
  analogWrite(LR, val);
  analogWrite(RR, val);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
}

//
//  Compute number of wheel ticks needed to turn by angle
//
unsigned long computeDeltaTicks(float angle)
{
  double dist = (angle / 360.0) * ROTATE_CIRC / WHEEL_CIRC;
  unsigned long ticks = TICKS_PER_REV * dist;

  return ticks;
}

//
//  Move left by angle degrees at `speed`% of full speed
//  angle of 0 indicates indefinite movement
//
void left(float angle, float speed)
{
  dir = LEFT;
  int val = pwmVal(speed);

  deltaTicks = (angle == 0) ? DIST_MAX : computeDeltaTicks(angle);
  leftTargetTicks = leftReverseTicksTurns + deltaTicks;
  rightTargetTicks = rightForwardTicksTurns + deltaTicks;

  // TODO: Bare-metal
  analogWrite(LR, val);
  analogWrite(RF, val);
  analogWrite(LF, 0);
  analogWrite(RR, 0);
}

//
//  Move right by angle degrees at `speed`% of full speed
//  angle of 0 indicates indefinite movement
//
void right(float angle, float speed)
{
  dir = RIGHT;
  int val = pwmVal(speed);

  deltaTicks = (angle == 0) ? DIST_MAX : computeDeltaTicks(angle);
  leftTargetTicks = leftForwardTicksTurns + deltaTicks;
  rightTargetTicks = rightReverseTicksTurns + deltaTicks;

  // TODO: Bare-metal
  analogWrite(RR, val);
  analogWrite(LF, val);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
}

void stop()
{
  dir = STOP;
  
  // TODO: Bare-metal
  analogWrite(LF, 0);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  analogWrite(RR, 0);
}

/**
 *  State Management
 */

//
//  Clears all our counters
//
void clearCounters()
{
  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;
  leftForwardTicksTurns = 0;
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightReverseTicksTurns = 0;
  forwardDist = 0;
  reverseDist = 0; 
}

//
//  Clears one particular counter
//
void clearOneCounter(int counterIdx)
{
  switch (counterIdx) {
    case 0:
      leftForwardTicks = 0;
      break;
    case 1:
      rightForwardTicks = 0;
      break;
    case 2:
      leftReverseTicks = 0;
      break;
    case 3:
      rightReverseTicks = 0;
      break;
    case 4:
      leftForwardTicksTurns = 0;
      break;
    case 5:
      rightForwardTicksTurns = 0;
      break;
    case 6:
      leftReverseTicksTurns = 0;
      break;
    case 7:
      rightReverseTicksTurns = 0;
      break;
    case 8:
      forwardDist = 0;
      break;
    case 9:
      reverseDist = 0;
      break;
  }
}

//
//  Intialize Alex's internal states
//
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

    case COMMAND_TURN_LEFT:
      sendOK();
      left((float) command->params[0], (float) command->params[1]);
    break;
      
    case COMMAND_TURN_RIGHT:
      sendOK();
      right((float) command->params[0], (float) command->params[1]);
    break;
      
    case COMMAND_REVERSE:
      sendOK();
      reverse((float) command->params[0], (float) command->params[1]);
    break;
      
    case COMMAND_STOP:
      sendOK();
      stop();
    break;
    
    case COMMAND_GET_STATS:
      sendStatus();
    break;

    case COMMAND_CLEAR_STATS:
      sendOK();
      if (command->params[0] == 255) {
        clearCounters();
      }
      else {
        clearOneCounter(command->params[0]);
      }
    break;
        
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit = 0;

  while (!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } 
    while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK)
    {
      if (hello.packetType == PACKET_TYPE_HELLO)
      {
        sendOK();
        exit = 1;
      }
      else
      {
        sendBadResponse();
      }
    }
    else if (result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else if (result == PACKET_CHECKSUM_BAD)
    {
      sendBadChecksum();
    }
  } // !exit
}

void setupUltrasonic()
{
  pinMode(TRIG_PIN_L, OUTPUT);
  pinMode(ECHO_PIN_L, INPUT);
  pinMode(TRIG_PIN_R, OUTPUT);
  pinMode(ECHO_PIN_R, INPUT);

  // Start ultrasonic sensors
  ultrasonicDist[0] = 0;
  ultrasonicDist[1] = 0;

  // Start other sensor out of phase
  ultrasonicIdx = 0;
  pingTimer[0] = millis() + 500;
  pingTimer[1] = pingTimer[0] + USONIC_PING_PERIOD;
}

void setup() {
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  setupUltrasonic();
  sei();

  waitForHello();
  delay(500);
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

void echoCheck() 
{
  if (ultrasonicSensor[ultrasonicIdx].check_timer()) {
    ultrasonicDist[ultrasonicIdx] = (float)ultrasonicSensor[ultrasonicIdx].ping_result / US_ROUNDTRIP_CM;
  }
}

void loop() 
{
  TPacket recvPacket; // This holds commands from the Pi
  TResult result = readPacket(&recvPacket);
  
  if (result == PACKET_OK) {
    handlePacket(&recvPacket);
  }
  else if (result == PACKET_BAD)
  {
    sendBadPacket();
  }
  else if (result == PACKET_CHECKSUM_BAD)
  {
    sendBadChecksum();
  } 

  for (int i = 0; i < 2; i++) {
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += 2 * USONIC_PING_PERIOD;
      if (i == 0 && ultrasonicIdx == 1) {
        sendUltrasonic();
      }

      ultrasonicSensor[ultrasonicIdx].timer_stop();

      ultrasonicIdx = i;
      ultrasonicDist[ultrasonicIdx] = 0;
      ultrasonicSensor[ultrasonicIdx].ping_timer(echoCheck);
    }
  }

  if (deltaDist > 0) {
    if (dir == FORWARD) {
      if (forwardDist >= newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == BACKWARD) {
        if (reverseDist >= newDist) {
          deltaDist = 0;
          newDist = 0;
          stop();
        }
    }
    else if (dir == STOP) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }

  if (deltaTicks > 0) {
    if (dir == LEFT) {
      if (leftReverseTicksTurns >= leftTargetTicks || rightForwardTicksTurns >= rightTargetTicks) {
        deltaTicks = 0;
        leftTargetTicks = 0;
        rightTargetTicks = 0;
        stop();
      }
    }
    else if (dir == RIGHT) {
      if (rightReverseTicksTurns >= rightTargetTicks || leftForwardTicksTurns >= leftTargetTicks) {
        deltaTicks = 0;
        leftTargetTicks = 0;
        rightTargetTicks = 0;
        stop();
      }
    }
    else if (dir == STOP) {
      deltaTicks = 0;
      leftTargetTicks = 0;
      rightTargetTicks = 0;
      stop();
    }
  }

}
