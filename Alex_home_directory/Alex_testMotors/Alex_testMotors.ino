
#include <serialize.h>

#include "packet.h"
#include "constants.h"
#include <math.h>

#define _STOP 0
#define _FORWARD 1
#define _BACKWARD 2
#define _LEFT 3
#define _RIGHT 4


/*
 * Alex's configuration constants
 */

 ////List of connections made to the Arduino//////
 //Pin 2: INT0 (PD2)
 //Pin 3: INT1 (PD3) 
 //Pin 5: OC1A (PD5) AIN1 AOUT1 RED LEFT MOTOR 
 //Pin 6: OC1B (PD6) AIN2 AOUT2 BLACK LEFT MOTOR
 //Pin 10: OC1B (PB2) BIN1 BOUT1 RED RIGHT MOTOR
 //Pin 11: OC2A (PB3) BIN2 BOUT2 BLACK RIGHT MOTOR

int count = 0;
typedef enum{
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      180

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          22

#include <stdarg.h>

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  10   // Left forward pin
#define LR                  11   // Left reverse pin
#define RF                  6   // Right forward pin
#define RR                  5   // Right reverse pin

#define ALEX_LENGTH 16
#define ALEX_BREADTH 6
//#define PI 3.141592654

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long rightReverseTicks; 

//Turn counter
volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long rightReverseTicksTurns; 
volatile unsigned long _leftForwardTicks = 0;
volatile unsigned long _rightForwardTicks = 0;


// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

unsigned long deltaDist;
unsigned long newDist;
unsigned long deltaTicks;
unsigned long targetTicks;

float AlexDiagonal = 0.0;
float AlexCirc = 0.0;

//bool leftMotorCalibrated = false;
//bool rightMotorCalibrated = false;

double tickDifference = 0.0;
double curr_pwm_l = 0.0;
int count_pwm = 0;
double curr_pwm_r = 0.0;

int kval[10] = {5,5,5,5,5};

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
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicks;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;

  sendResponse(&statusPacket);
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
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
  DDRD &= ~((1<<2)|(1<<3));
  PORTD |= ((1<<2)|(1<<3));
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  if(dir == RIGHT) leftForwardTicksTurns++;
  
  else if(dir == LEFT) leftReverseTicksTurns++;

  
  
  else if(dir == FORWARD) {
    forwardDist = (unsigned long)((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC); 
    _leftForwardTicks++;
    leftForwardTicks++;
  }
  
  else if(dir == BACKWARD) {
    reverseDist = (unsigned long)((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC); 
    leftReverseTicks++;
  }
}

void rightISR()
{

    if(dir == FORWARD) {
      rightForwardTicks++; 
      _rightForwardTicks++;
    }
    else if (dir == BACKWARD) rightReverseTicks++; 
    else if (dir == RIGHT) rightReverseTicksTurns++; 
    else if (dir == LEFT) rightForwardTicksTurns++; 


  //rightTicks++;
  /*Serial.print("RIGHT: ");
  Serial.println(rightTicks);*/
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  cli();
  EICRA = 0b00001010; // falling edge
//  DDRD &= 0b11110011;
  EIMSK |= 0b00000011; // activate
  sei();
  
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.
ISR(INT0_vect){
  leftISR();
}
ISR(INT1_vect){
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
    buffer[count++] = Serial.read();

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









void dbprint(char *format, ...) {
  va_list args;
  char buffer[128];

  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}













// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
int moderror(int leftval, int rightval){
    signed int error = leftval - rightval;
    return (error <0)? -error : error;
}





/*

void calibrateMotors(){
  int error;
  //error = (error < 0)? -error: error;
  double val = curr_pwm;
  
  switch(dir){
    case FORWARD:

      error = _leftForwardTicks - _rightForwardTicks;
      
      if(error!= 0){
        val += (error *30);
        Serial.println("val is ");
        Serial.println(val);
        Serial.println("error is ");
        Serial.println(error);
        analogWrite(RF, val);
      }
      _leftForwardTicks = 0;
      _rightForwardTicks = 0;
      delay(100);
    break;

    case BACKWARD:
      error = leftReverseTicks - rightReverseTicks;
      while(error){
        val += error / kval[_BACKWARD];
        analogWrite(RR, val);
        delayMicroseconds(1);
      
      }
    break;

    case RIGHT:
      error = leftForwardTicksTurns - rightReverseTicksTurns;
      if(error != 0){
        val += error / 5;
        analogWrite(RR, val);
        delayMicroseconds(1);
      }
    break;

    case LEFT:
      error = leftReverseTicksTurns - rightForwardTicksTurns;
      while(error){
        val += error / kval[_LEFT];
        analogWrite(RF, val);
        delayMicroseconds(1);
      }
    break;
  }
}
*/

void calibrateMotors(){
  double error;
  int curr_left = 0;
  int curr_right = 0;
  
  int val = curr_pwm_r;
  
  switch(dir){
    case FORWARD:
      curr_left = leftForwardTicks;
      curr_right = rightForwardTicks;

      delayMicroseconds(50000);
      curr_left = leftForwardTicks - curr_left;
      curr_right = rightForwardTicks - curr_right;
      error = curr_left - curr_right;
      
      if(curr_left == 0 && curr_right == 0){
        if(count_pwm < 500) {
          count_pwm++;
          return;
        }
        
        analogWrite(LF, 255);
        analogWrite(RF, 255);
        delay(50);
        analogWrite(LF, curr_pwm_l);
        analogWrite(RF, curr_pwm_r);       
        return;
      }
      count_pwm = 0;
      if(error){
        //Serial.println(error);
        val += error*20 ;
        curr_pwm_r = (val > 255)? 255: 
                   (val < 0)? 0:
                   val;
      //  Serial.print("new error is ");
       // Serial.println(error*20);
       // Serial.print("new val is ");
        //Serial.println(curr_pwm_r);
        
        analogWrite(RF, curr_pwm_r);
      }
    break;

    case BACKWARD:
      curr_left = leftReverseTicks;
      curr_right = rightReverseTicks;
      delay(50);
      curr_left = leftReverseTicks - curr_left;
      curr_right = rightReverseTicks - curr_right;
      error = curr_left - curr_right;
      
      if(error){
        val += error*20 ;
        curr_pwm_r = (val > 255)? 255: 
                   (val < 0)? 0:
                   val;
        Serial.print("new error is ");
        
        Serial.println(error);
        Serial.print("new val is ");
        Serial.println(curr_pwm_r);
        //curr_pwm = val;
        analogWrite(RR, curr_pwm_r);
      }
    break;

    case RIGHT:
      curr_left = leftForwardTicksTurns;
      curr_right = rightReverseTicksTurns;
      delay(50);
      curr_left = leftForwardTicksTurns - curr_left;
      curr_right = rightReverseTicksTurns - curr_right;
      error = curr_left - curr_right;
      
      if(error){
        val += error*20;
        curr_pwm_r = (val > 255)? 255: 
                   (val < 0)? 0:
                   val;
        analogWrite(RR, curr_pwm_r);
        Serial.print("new error is ");
        Serial.println(error*20);
        Serial.print("new val is ");
        Serial.println(curr_pwm_r);
      }
    break;

    case LEFT:
      curr_left = leftReverseTicksTurns;
      curr_right = rightForwardTicksTurns;
      delay(50);
      curr_left = leftReverseTicksTurns - curr_left;
      curr_right = rightForwardTicksTurns - curr_right;
      
      error = curr_left - curr_right;
      if(error){
        val += error*20;
        curr_pwm_r = (val > 255)? 255: 
                   (val < 0)? 0:
                   val;
        analogWrite(RF, curr_pwm_r);
        Serial.print("The error is");
        Serial.println(error);
        Serial.print("The pwval is ");
        Serial.println(val);
      }
    break;

    case STOP:
      stop();
    break;
  }
}
























void forward(float dist, float speed)
{
  dir = FORWARD;
  
  curr_pwm_l = pwmVal(speed);
  curr_pwm_r = 1.36* curr_pwm_l;
  
  int error = leftForwardTicks - rightForwardTicks;
  // For now we will ignore dist and move
  // forward indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.


  if(dist > 0) deltaDist = dist;
  else deltaDist = 9999999;
  newDist = forwardDist + deltaDist;
  analogWrite(LF, curr_pwm_l);
  analogWrite(RF, curr_pwm_r);
  analogWrite(LR, 0);
  analogWrite(RR, 0);
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  dir = BACKWARD;
  int val = pwmVal(speed);
  curr_pwm_r = 1.36 * val;

  if(dist > 0) deltaDist = dist;
  else deltaDist = 9999999;
  newDist = reverseDist + deltaDist;
  // For now we will ignore dist and 
  // reverse indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  analogWrite(LR, val);
  analogWrite(RR, curr_pwm_r);
  analogWrite(LF, 0);
  analogWrite(RF, 0);

  
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.

unsigned long computeDeltaTicks(float ang){
  unsigned long ticks = (unsigned long)((ang * AlexCirc * COUNTS_PER_REV)/(360.0 * WHEEL_CIRC));
  return ticks;
}

void left(float ang, float speed)
{
  dir = LEFT;
  int val = pwmVal(speed);
  curr_pwm_r = (1*val > 255)? 255: 1*val;
  if(ang == 0) deltaTicks = 9999999;
  else deltaTicks = computeDeltaTicks(ang);
  targetTicks = leftReverseTicksTurns + deltaTicks;
  
  
  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  analogWrite(RF, curr_pwm_r);
  analogWrite(LR, val);
  analogWrite(RR, 0);
  analogWrite(LF, 0);

}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  dir = RIGHT;
  int val = pwmVal(speed);
  curr_pwm_r = (1.36*val > 255)? 255: 1.36*val;

  if(ang == 0) deltaTicks = 9999999;
  else deltaTicks = computeDeltaTicks(ang);
  targetTicks = rightReverseTicksTurns + deltaTicks;

  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  analogWrite(RR, curr_pwm_r);
  analogWrite(LF, val);
  analogWrite(RF, 0);
  analogWrite(LR, 0);
 
}
    

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;
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
  /*leftTicks=0;
  rightTicks=0;
  leftRevs=0;
  rightRevs=0;
  forwardDist=0;
  reverseDist=0; */
leftForwardTicks = 0; 
leftReverseTicks = 0; 
rightForwardTicks = 0;
rightReverseTicks = 0; 

//Turn counter
leftForwardTicksTurns = 0; 
leftReverseTicksTurns = 0; 
rightForwardTicksTurns = 0;
rightReverseTicksTurns = 0; 
}

// Clears one particular counter
void clearOneCounter(int which)
{
  //clearCounters();
  switch(which)
  {
    case 0:
      clearCounters();
      break;

    case 1:
      leftForwardTicks=0;
      break;

    case 2:
      rightForwardTicks=0;
      break;

    case 3:
      leftForwardTicksTurns = 0;
      leftReverseTicksTurns = 0;
      break;

    case 4:
      rightForwardTicksTurns = 0;
      rightReverseTicksTurns = 0;
      break;

    case 5:
      forwardDist=0;
      break;

    case 6:
      reverseDist=0;
      break;
  }
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
    /*
     * Implement code for other commands here.
     * 
     */
    case COMMAND_CLEAR_STATS:
        sendOK();
        clearOneCounter(command->params[0]);
    break;

    case COMMAND_GET_STATS:
        sendOK();
        sendStatus();
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

  AlexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  AlexCirc = PI * AlexDiagonal;
  sei();

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

// Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

// forward(0, 100);

// Uncomment the code below for Week 9 Studio 2
  
 

 // put your main code here, to run repeatedly:
  /*TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else{
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      } 
  }


  calibrateMotors();
  
  
  if(deltaDist > 0){
    if(dir == FORWARD){
      if(forwardDist > newDist){
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
  }
  else{
    if(dir == BACKWARD){
      if(reverseDist > newDist){
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else{
      if(dir == STOP){
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
  }

  if(deltaTicks > 0){
    if(dir == LEFT){
      if(leftReverseTicksTurns >= targetTicks){
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else{
      if(dir == RIGHT){
        if(rightReverseTicksTurns >= targetTicks){
          deltaTicks = 0;
          targetTicks = 0;
          stop();
        }
        else{
          if(dir == STOP){
            deltaTicks = 0;
            targetTicks = 0;
            stop();
          }
        }
      }
    }
  }*/
  signed int error = leftForwardTicks - rightForwardTicks;
  left(8,85);
  //reverse(50,50);
  //forward(0, 40);
  //forward(0, 50);
 
  
  
    
    //Serial.print("left forward ticks are ");
  //Serial.println(leftReverseTicks);
  //Serial.print("right forward ticks are ");
  //Serial.println(rightReverseTicks);
  //Serial.print("error is ");

  /*Serial.print("error is ");
  Serial.println(error);*/
  calibrateMotors();
  delay(200);
  stop();
  delay(200);

  
}
