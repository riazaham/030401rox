#include <buffer.h>

#include <buffer.h>
#include <serialize.h>
#include <avr/sleep.h> 
#include "packet.h"
#include "constants.h"
#include "buffer.h"
#include <math.h>
#include <stdio.h>
#include <stdarg.h>

#define _STOP 0
#define _FORWARD 1
#define _BACKWARD 2
#define _LEFT 3
#define _RIGHT 4

#define PRR_TWI_MASK            0b10000000 
#define PRR_SPI_MASK            0b00000100 
#define ADCSRA_ADC_MASK         0b10000000 
#define PRR_ADC_MASK            0b00000001 
#define PRR_TIMER2_MASK         0b01000000 
#define PRR_TIMER0_MASK         0b00100000 
#define PRR_TIMER1_MASK         0b00001000 
#define SMCR_SLEEP_ENABLE_MASK  0b00000001 
#define SMCR_IDLE_MODE_MASK     0b11110001 
//#define PIN5 0b00010000;/

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

//Serial bare metal
#define RECV_SIZE 128
#define XMIT_SIZE 512
TBuffer _recvBuffer;
TBuffer _xmitBuffer;

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
// right encoders. [continuously updated]
volatile unsigned long leftForwardTicks; 
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long rightReverseTicks; 

//Turn counter [continuously updated]
volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long rightReverseTicksTurns; 

// Store the ticks from Alex's left and
// right encoders. [stored]
volatile unsigned long _leftForwardTicks; 
volatile unsigned long _leftReverseTicks; 
volatile unsigned long _rightForwardTicks;
volatile unsigned long _rightReverseTicks; 

//Turn counter [stored]
volatile unsigned long _leftForwardTicksTurns; 
volatile unsigned long _leftReverseTicksTurns; 
volatile unsigned long _rightForwardTicksTurns;
volatile unsigned long _rightReverseTicksTurns; 

//Dummy temp OCRnX values
volatile unsigned int _OCR0A = 0; 
volatile unsigned int _OCR0B = 0; 
volatile unsigned int _OCR1A = 0; 
volatile unsigned int _OCR1BL = 0; 
volatile unsigned int _OCR2A = 0; 
volatile unsigned int _OCR2B = 0; 

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// Distance and tick variables to update and monitor change
unsigned long deltaDist;
unsigned long newDist;
unsigned long deltaTicks;
unsigned long targetTicks;
int ok_flag = 1;

volatile int _count = 0;

float AlexDiagonal = 0.0;
float AlexCirc = 0.0;

double tickDifference = 0.0;
double curr_pwm = 0.0;


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
    
    unsigned char buffer[PACKET_SIZE];
    int len;

    cli();
    len = readSerial(buffer);
    sei();

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize((const char*) buffer, len, packet);
    
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

void sendCommand(int command)
{
  TPacket commandPacket;
  commandPacket.packetType = PACKET_TYPE_COMMAND;
  switch(command){
      case COMMAND_RPLIDAR_SLEEP:
        commandPacket.command = command;
        sendResponse(&commandPacket);
        break;
  }
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
  // Tell the Pi to send us COMMAND_OK
 
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
  cli();
  writeSerial((const unsigned char*) buffer, len);
  sei();
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

// Functions to be called by INT0 and INT1 ISRs. [Update of ticks]
void leftISR()
{
  if(dir == RIGHT) leftForwardTicksTurns++;
  else if(dir == LEFT) leftReverseTicksTurns++;

  else if(dir == FORWARD) {
    forwardDist = (unsigned long)((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC); 
    leftForwardTicks++;
  }
  
  else if(dir == BACKWARD) {
    reverseDist = (unsigned long)((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC); 
    leftReverseTicks++;
  }
}

void rightISR()
{
    if(dir == FORWARD) rightForwardTicks++; 
    else if (dir == BACKWARD) rightReverseTicks++; 
    else if (dir == RIGHT) rightReverseTicksTurns++; 
    else if (dir == LEFT) rightForwardTicksTurns++; 
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  cli();
  EICRA = 0b00001010; // falling edge
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
// Set up the serial connection
void setupSerial()
{
  // To replace later with bare-metal.
  //Serial.begin(9600)
 
 //Bare Metal Implementation
  UBRR0L = 103;
  UBRR0H = 0;

  UCSR0C = 0b00000110; 
  UCSR0A = 0;
}

// Start the serial connection
void startSerial()
{
  // Bare Metal Implementation.
  UCSR0B = 0b10111000;
}

// Initialize the receive and transmit buffers.
void setupBuffers()
{
    initBuffer(&_recvBuffer, RECV_SIZE);
    initBuffer(&_xmitBuffer, XMIT_SIZE);
}

// Receive and write data to buffer [Interrupt when data received]
ISR(USART_RX_vect) {
 
    // Write received data
    unsigned char data = UDR0;
    writeBuffer(&_recvBuffer, data);
}

// Read the serial port
int readSerial(unsigned char *buffer)
{
   //Bare Metal Implementation
    int count = 0;

    TBufferResult result;

    do {
        result = readBuffer(&_recvBuffer, &buffer[count]);
        if (result == BUFFER_OK)
            count++;
    } while (result == BUFFER_OK);

    return count;
}

//Transmit serialised data
ISR(USART_UDRE_vect)
{ 
    unsigned char data;
    TBufferResult result = readBuffer(&_xmitBuffer, &data);
 
    if (result == BUFFER_OK)
        UDR0 = data;
    else
        if (result == BUFFER_EMPTY) {
            //Disable interrupt once all the data has been sent
            UCSR0B &= 0b11011111;    
        }
}


// Write to the serial port
void writeSerial(const unsigned char *buffer, int len)
{

  //Bare Metal Implementation
   TBufferResult result = BUFFER_OK;
   for(int i = 1; i < len && result == BUFFER_OK; i++)
   {
    result = writeBuffer(&_xmitBuffer, buffer[i]);
   }
 UDR0 = buffer[0];
 //Enable transmit interrupt
 UCSR0B |= 0b00100000;
}

/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors [setup PWM to run the motors]
void setupMotors()
{
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 10, PB2, OC1B
   *    B2In - pIN 11, PB3, OC2A
   */
  
    //Bare Metal Implementation
    DDRD |= 0b01100000; // Pin 5 and 6
    DDRB |= 0b00001100; // Pin 9 and 10

    //TCN0
    TCNT0 = 0;
    OCR0A = 0;
    OCR0B = 0;
    TIMSK0 |= 0b110;
    TCCR0B = 0b00000011;

    //TCN1
    TCNT1 = 0;
    OCR1BL = 0;
    OCR1BH = 0;
    TIMSK1 |= 0b110;
    TCCR1B = 0b00000011;

    //TCN2
    TCNT2 = 0;
    OCR2A = 0;
    TIMSK2 |= 0b110;
    TCCR2B = 0b00000011;
}

ISR(TIMER0_COMPA_vect)
{
  OCR0A = _OCR0A;
}
ISR(TIMER0_COMPB_vect)
{
  OCR0B = _OCR0B;
}
ISR(TIMER1_COMPA_vect)
{
}
ISR(TIMER1_COMPB_vect)
{
  OCR1BL = _OCR1BL;
}
ISR(TIMER2_COMPA_vect)
{
  OCR2A = _OCR2A;
}
ISR(TIMER2_COMPB_vect)
{
}

// Start the PWM for Alex's motors
void startMotors()
{
   //Bare Metal Implementation
    TCCR0A = 0b00000001;
    TCCR1A = 0b00000001;
    TCCR2A = 0b00000001;
}

//Bare Metal Implementation
//Substitute to analogWrite()
//Controls Output Compare pins IO to produce
//required motion.
void _analogWrite(int dir, int pwm){
  switch(dir){
    case LF: 
      if(pwm == 0) TCCR1A &= 0b11001111;
      else TCCR1A |= 0b00100000;
      _OCR1BL = pwm;
      break;
    case RF:
      if(pwm == 0) TCCR0A &= 0b00111111;
      else TCCR0A |= 0b10000000;
      _OCR0A = pwm;
      break;
    case LR:
      if(pwm == 0) TCCR2A &= 0b00111111;
      else TCCR2A |= 0b10000000;
      _OCR2A = pwm;
      break;
    case RR:
      if(pwm == 0) TCCR0A &= 0b11001111;
      else TCCR0A |= 0b00100000;
      _OCR0B = pwm;
      break;
  }
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

//Used purely for debugging
void dbprint(char *format, ...) {
  va_list args;
  char buffer[128];

  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}



//Calibrates the pwm value supplied to the slave motor so that the robot can move straight as desired.
//Error is calculated by difference in rate of change of left ticks and that of right ticks.
//A constant proportionate to the error value is then summed with the slave motor's current pwm value,
//which can increase and decrease depending on value of error.
//The proportion of the constant to the error is obtained via exhaustive trial and error.

void calibrateMotors(){
  double error;
  int curr_left = 0;
  int curr_right = 0;
  
  double val = curr_pwm;
  switch(dir){
    case FORWARD:
      
      curr_left = leftForwardTicks;
      curr_right = rightForwardTicks;
      delayMicroseconds(50000);
      curr_left = leftForwardTicks - curr_left;
      curr_right = rightForwardTicks - curr_right;
      error = curr_left - curr_right;
      
      if(error){
        val += error*20;

        //PWM value supplied to the motors cannot be out of range of [0, 255].
        curr_pwm = (val>255)?255:
                   (val < 0)?0:
                   val;
        _analogWrite(RF, curr_pwm);
      }
    break;

    case BACKWARD:
      curr_left = leftReverseTicks;
      curr_right = rightReverseTicks;
      delayMicroseconds(50000);
      curr_left = leftReverseTicks - curr_left;
      curr_right = rightReverseTicks - curr_right;
      
      error = curr_left - curr_right;
      
      if(error){
        val += error*20;

        //PWM value supplied to the motors cannot be out of range of [0, 255].
        curr_pwm = (val>255)?255:
                   (val < 0)?0:
                   val;
        _analogWrite(RR, curr_pwm);
      }
    break;

    case RIGHT:
      curr_left = leftForwardTicksTurns;
      curr_right = rightReverseTicksTurns;
      delayMicroseconds(50000);
      curr_left = leftForwardTicksTurns - curr_left;
      curr_right = rightReverseTicksTurns - curr_right;
      error = curr_left - curr_right;
      
      if(error){
        val += error*20;

        //PWM value supplied to the motors cannot be out of range of [0, 255].
        curr_pwm = (val>255)?255:
                   (val < 0)?0:
                   val;
        _analogWrite(RR, curr_pwm);
      }
    break;

    case LEFT:
      curr_left = leftReverseTicksTurns;
      curr_right = rightForwardTicksTurns;
      delayMicroseconds(50000);
      curr_left = leftReverseTicksTurns - curr_left;
      curr_right = rightForwardTicksTurns - curr_right;
      
      error = curr_left - curr_right;
      if(error){
        val += error*20;

        //PWM value supplied to the motors cannot be out of range of [0, 255].
        curr_pwm = (val>255)?255:
                   (val < 0)?0:
                   val;
        _analogWrite(RF, curr_pwm);
      }
    break;

    case STOP:
      stop();
    break;
  }
  
}

// Alex's 2 directional move functions (forward and reverse): 
// move "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.

// LF = Left forward pin, LR = Left reverse pin (implemented with bare metal)
// RF = Right forward pin, RR = Right reverse pin (implemented with bare metal)

void forward(float dist, float speed)
{
  dir = FORWARD;
  
  int val = pwmVal(speed);
  curr_pwm = 1.36*val;
  int error = leftForwardTicks - rightForwardTicks;

  if(dist > 0) deltaDist = dist;
  else deltaDist = 9999999;
  newDist = forwardDist + deltaDist;
  _analogWrite(LF, val);
  _analogWrite(RF, curr_pwm);
  _analogWrite(LR, 0);
  _analogWrite(RR, 0);
}

void reverse(float dist, float speed)
{
  dir = BACKWARD;
  int val = pwmVal(speed);
  curr_pwm = 1.36*val;

  if(dist > 0) deltaDist = dist;
  else deltaDist = 9999999;
  newDist = reverseDist + deltaDist;

  _analogWrite(LR, val);
  _analogWrite(RR, curr_pwm);
  _analogWrite(LF, 0);
  _analogWrite(RF, 0);

  
}

// Alex's 2 directional move functions (left and right): 
// Turn Alex at "ang" degrees at speed "speed".
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
  curr_pwm = (1.36*val > 255)?255: 1.36*val;
  if(ang == 0) deltaTicks = 9999999;
  else deltaTicks = computeDeltaTicks(ang);
  targetTicks = leftReverseTicksTurns + deltaTicks;
  
  // Left turn: reverse the left wheel and move forward the right wheel
  _analogWrite(RF, curr_pwm);
  _analogWrite(LR, val);
  _analogWrite(RR, 0);
  _analogWrite(LF, 0);

}

void right(float ang, float speed)
{
  dir = RIGHT;
  int val = pwmVal(speed);
  curr_pwm = (val*1.36 > 255)? 255: 1.36*val;

  if(ang == 0) deltaTicks = 9999999;
  else deltaTicks = computeDeltaTicks(ang);
  targetTicks = rightReverseTicksTurns + deltaTicks;

  // Right turn: move forward the left wheel and reverse the right wheel
  _analogWrite(RR, curr_pwm);
  _analogWrite(LF, val);
  _analogWrite(RF, 0);
  _analogWrite(LR, 0);
 
}
    

// Stop Alex
void stop()
{
  dir = STOP;
  _analogWrite(LF, 0);
  _analogWrite(LR, 0);
  _analogWrite(RF, 0);
  _analogWrite(RR, 0);
 
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

// Intialize Alex's internal states
void initializeState()
{
  clearCounters();
}

void waitForReady(){
  ok_flag = 0;
  while(!ok_flag){
    TPacket temp;
    TResult result = readPacket(&temp);
    if(result == PACKET_OK) handleResponse(&temp);
  }
}

void handleCommand(TPacket *command)
{
  int i = 0;
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

    case COMMAND_CLEAR_STATS:
        sendOK();
        clearOneCounter(command->params[0]);
    break;

    case COMMAND_GET_STATS:
        sendOK();
        ok_flag = 0;
        waitForReady();
        sendStatus();
    break;
        
    default:
      sendBadCommand();
  }
}

//Used only for testing
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
  setupBuffers();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  setupPowerSaving();
  
  AlexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  AlexCirc = PI * AlexDiagonal;
  sei();

}

void handleResponse(TPacket *temp)
{
  switch(temp->command){
    case RESP_OK:
      ok_flag = 1;
    break;
  }
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      handleResponse(packet);
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void WDT_off(void) 
{ 
/* Global interrupt should be turned OFF here if not already done so */ 
 
/* Clear WDRF in MCUSR */ 
MCUSR &= ~(1<<WDRF); 
 
/* Write logical one to WDCE and WDE */ 
/* Keep old prescaler setting to prevent unintentional 
time-out */ 
WDTCSR |= (1<<WDCE) | (1<<WDE); 
 
/* Turn off WDT */ 
WDTCSR = 0x00; 
 
/* Global interrupt should be turned ON here if subsequent operations after calling this function do 
not require turning off global interrupt */ 
} 

void setupPowerSaving() 
{ 
  // Turn off the Watchdog Timer   
  WDT_off();
  // Modify PRR to shut down TWI 
  PRR |= PRR_TWI_MASK; 
  // Modify PRR to shut down SPI 
  PRR |= PRR_SPI_MASK;
  // Modify ADCSRA to disable ADC,  
  // then modify PRR to shut down ADC
  ADCSRA &= ~ADCSRA_ADC_MASK;
  PRR |= PRR_ADC_MASK;
  // Set the SMCR to choose the IDLE sleep mode   // Do not set the Sleep Enable (SE) bit yet 
  SMCR &= SMCR_IDLE_MODE_MASK;
  // Set Port B Pin 5 as output pin, then write a logic LOW   // to it so that the LED tied to Arduino's Pin 13 is OFF. 
  DDRB |= (1<<5);
  PORTB &= ~(1<<5);
} 

void putArduinoToIdle() 
{ 
  //waitForReady();
  int i = 0;

  // Modify PRR to shut down TIMER 0, 1, and 2 
  PRR |= (PRR_TIMER2_MASK | PRR_TIMER0_MASK| PRR_TIMER1_MASK);
  
  // Modify SE bit in SMCR to enable (i.e., allow) sleep 
  SMCR |= SMCR_SLEEP_ENABLE_MASK;
  
  // The following function puts ATmega328P’s MCU into sleep; 
  // it wakes up from sleep when USART serial data arrives 
  
  sleep_cpu(); 


  // Modify SE bit in SMCR to disable (i.e., disallow) sleep 
  SMCR &= ~SMCR_SLEEP_ENABLE_MASK;
  
  // Modify PRR to power up TIMER 0, 1, and 2 
  PRR &= ~(PRR_TIMER2_MASK | PRR_TIMER0_MASK | PRR_TIMER1_MASK);
                                                                                                                                     
} 
 


void loop() {

 // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

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

  //Every round of routine, adjustments will be made to the slave motor such that the speed will be closer and closer 
  //to the master motor.
  calibrateMotors();
  
  
  if(deltaDist > 0){
    if(dir == FORWARD){
      if(forwardDist > newDist){
        deltaDist = 0;
        newDist = 0;
        stop();
        putArduinoToIdle();
      }
    }
  
    else{
      if(dir == BACKWARD){
        if(reverseDist > newDist){
          deltaDist = 0;
          newDist = 0;
          stop();
          putArduinoToIdle();
        }
      
        else{
          if(dir == STOP){
            putArduinoToIdle();
            deltaDist = 0;
            newDist = 0;
            stop();
        
          }
        }
      }
    }
  }
  if(deltaTicks > 0){
    if(dir == LEFT){
      if(leftReverseTicksTurns >= targetTicks){
        deltaTicks = 0;
        targetTicks = 0;
        stop();
        putArduinoToIdle();
      }
    }
    else{
      if(dir == RIGHT){
        if(rightReverseTicksTurns >= targetTicks){
          deltaTicks = 0;
          targetTicks = 0;
          stop();
          putArduinoToIdle();
        }
        else{
          if(dir == STOP){
            putArduinoToIdle();
            deltaTicks = 0;
            targetTicks = 0;
            stop();
            
          }
        }
      }
    }
  }
  
}

