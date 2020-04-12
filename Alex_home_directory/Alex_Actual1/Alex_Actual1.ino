#include <serialize.h>
#include <avr/sleep.h> 
#include "packet.h"
#include "constants.h"
#include "buffer.h"
#include <math.h>

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

#define RECV_SIZE      128

static TBuffer _recvBuffer;


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

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long _leftForwardTicks; 
volatile unsigned long _leftReverseTicks; 
volatile unsigned long _rightForwardTicks;
volatile unsigned long _rightReverseTicks; 

//Turn counter
volatile unsigned long _leftForwardTicksTurns; 
volatile unsigned long _leftReverseTicksTurns; 
volatile unsigned long _rightForwardTicksTurns;
volatile unsigned long _rightReverseTicksTurns; 



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

volatile int _count = 0;

float AlexDiagonal = 0.0;
float AlexCirc = 0.0;

//bool leftMotorCalibrated = false;
//bool rightMotorCalibrated = false;

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
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      printdb("Buffer is read\n");
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
  //Serial.begin(9600);
  UBRR0L = 103;
  UBRR0H = 0;

  UCSR0C = 0b00000110; 
  UCSR0A = 0;
}

void setupBuffers()
{
    // Initialize the receive and transmit buffers.
    initBuffer(&_recvBuffer, RECV_SIZE);
}



// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
  UCSR0B = 0b10011000;
}

ISR(USART_RX_vect) {
    // Write received data
    unsigned char data = UDR0;
    writeBuffer(&_recvBuffer, data);
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(unsigned char* line)
{

    /*
    int count=0;
    while(Serial.available())
      buffer[count++] = Serial.read();
    return count;

    */
    int count = 0;

    TBufferResult result;

    do {
        result = readBuffer(&_recvBuffer, &line[count]);
        if (result == BUFFER_OK)
            count++;
    } while (result == BUFFER_OK);

    return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const unsigned char* line, int len) {
    //Serial.write(buffer, len);

    while (len--) {
        while ((UCSR0A & 0b00100000) == 0);
        UDR0 = *line;
        line++;
    }
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
 
 //Bare-metal
 //Set Pin5 and Pin6 as output
 DDRD |= ((1 << PIN6) | (1 << PIN5));
 
 //Setup PWM
 TCNT0 = 0;
 TIMSK0 |= 0b110; //OCIEA = 1, OCIEB = 1
 OCR0A = 128;
 OCR0B = 128;
 TCCR0B = 0b00000011; //clk64
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
//void startMotors()
void right_motor_forward()
{
 TCCR0A = 0b10000001;
}

void right_motor_reverse()
{
 TCCR0A = 0b00100001;
}

void left_motor_forward()
{
 TCCR0A = 0b01000001;
}

void left_motor_reverse()
{
 TCCR0A = 0b00010001;
}

ISR(TIMER0_COMPA_vect){}
ISR(TIMER0_COMPB_vect){}

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


void printdb(char *format, ...) {
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
        //analogWrite(RF, curr_pwm);
       //Bare metal
       OCR0B = curr_pwm;
       right_motor_forward();
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
        //analogWrite(RR, curr_pwm);
       //Bare metal
       OCR0B = curr_pwm;
       right_motor_reverse();
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
        //analogWrite(RR, curr_pwm);
       //Bare metal
       OCR0B = curr_pwm;
       right_motor_reverse();
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
        //analogWrite(RF, curr_pwm);
       //Bare metal
       OCR0B = curr_pwm;
       right_motor_forward();
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
  
  int val = pwmVal(speed);
  curr_pwm = 1.36*val;
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
  //analogWrite(LF, val);
  //analogWrite(RF, curr_pwm);
  //analogWrite(LR, 0);
  //analogWrite(RR, 0);
 OCR0A = val;
 OCR0B = curr_pwm;
 right_motor_forward();
 left_motor_forward();
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
  curr_pwm = 1.36*val;

  if(dist > 0) deltaDist = dist;
  else deltaDist = 9999999;
  newDist = reverseDist + deltaDist;
  // For now we will ignore dist and 
  // reverse indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  //analogWrite(LR, val);
  //analogWrite(RR, curr_pwm);
  //analogWrite(LF, 0);
  //analogWrite(RF, 0);
 OCR0A = val;
 OCR0B = curr_pwm;
 right_motor_reverse();
 left_motor_reverse();
  
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
  curr_pwm = (1.36*val > 255)?255: 1.36*val;
  if(ang == 0) deltaTicks = 9999999;
  else deltaTicks = computeDeltaTicks(ang);
  targetTicks = leftReverseTicksTurns + deltaTicks;
  
  
  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  //analogWrite(RF, curr_pwm);
  //analogWrite(LR, val);
  //analogWrite(RR, 0);
  //analogWrite(LF, 0);
 OCR0A = val;
 OCR0B = curr_pwm;
 right_motor_forward();
 left_motor_reverse();
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
  curr_pwm = (val*1.36 > 255)? 255: 1.36*val;

  if(ang == 0) deltaTicks = 9999999;
  else deltaTicks = computeDeltaTicks(ang);
  targetTicks = rightReverseTicksTurns + deltaTicks;

  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  //analogWrite(RR, curr_pwm);
  //analogWrite(LF, val);
  //analogWrite(RF, 0);
  //analogWrite(LR, 0);
 OCR0A = val;
 OCR0B = curr_pwm;
 right_motor_reverse();
 left_motor_forward();
}
    

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;
  //analogWrite(LF, 0);
  //analogWrite(LR, 0);
  //analogWrite(RF, 0);
  //analogWrite(RR, 0);
 TCCR0A = 0b00000001;
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
  //startMotors();
  enablePullups();
  initializeState();
  setupPowerSaving();
  
  AlexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  AlexCirc = PI * AlexDiagonal;
  sei();

}

void handlePacket(TPacket *packet)
{
     printdb("Packet is ");
    printdb(packet->packetType);
    printdb("\n");
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
  

// Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

// forward(0, 100);

// Uncomment the code below for Week 9 Studio 2
  
 

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
//        /putArduinoToIdle();
      }
    }
  
    else{
      if(dir == BACKWARD){
        if(reverseDist > newDist){
          deltaDist = 0;
          newDist = 0;
          stop();
//          /putArduinoToIdle();
        }
      
        else{
          if(dir == STOP){
            putArduinoToIdle();
            deltaDist = 0;
            newDist = 0;
            stop();
//           
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
//        /putArduinoToIdle();
      }
    }
    else{
      if(dir == RIGHT){
        if(rightReverseTicksTurns >= targetTicks){
          deltaTicks = 0;
          targetTicks = 0;
          stop();
//          /putArduinoToIdle();
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
 
