
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


int dir;

typedef enum{
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;

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

//Turn counter
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

unsigned long deltaDist;
unsigned long newDist;
unsigned long deltaTicks;
unsigned long targetTicks;

int pwm_LF = 0;
int pwm_LR = 0;
int pwm_RF = 0;
int pwm_RR = 0; 

volatile int _count = 0;

float AlexDiagonal = 0.0;
float AlexCirc = 0.0;

//bool leftMotorCalibrated = false;
//bool rightMotorCalibrated = false;

double tickDifference = 0.0;
double curr_pwm = 0.0;


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
/*
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

*/

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.
/*
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
 /*   int count = 0;

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
*/
/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors() {
   //Pin 5: OC1A (PD5) AIN1 AOUT1 RED LEFT MOTOR 
   //Pin 6: OC1B (PD6) AIN2 AOUT2 BLACK LEFT MOTOR
   //Pin 10: OC1B (PB2) BIN1 BOUT1 RED RIGHT MOTOR
   //Pin 11: OC2A (PB3) BIN2 BOUT2 BLACK RIGHT MOTOR

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




void startMotors() {
    TCCR0A = 0b10100001;
    TCCR1A = 0b00100001;
    TCCR2A = 0b10000001;
}

/*
ISR(TIMER0_COMPA_vect) {
    OCR0A = pwm_LF;     
}

ISR(TIMER0_COMPB_vect) {
    OCR0B = pwm_LR;     
}

ISR(TIMER1_COMPB_vect) {    
    OCR1B = pwm_RR;      
}

ISR(TIMER2_COMPA_vect) {
    OCR2A = pwm_RF;     
}*/


/*void right_motor_forward(void) {//pin10
    TCCR1A = 0b01000001;
    PORTB &= 0b11111011;
}

void right_motor_reverse(void) {//pin11
    TCCR1A = 0b00010001;
    PORTB &= 0b11110111;
}

void left_motor_forward(void) {//pin5
    TCCR0A = 0b10000001;
    PORTD &= 0b11011111;
}

void left_motor_reverse(void) {//pin6
    TCCR0A = 0b00100001;
    PORTD &= 0b10111111;
}*/

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return ((speed / 100.0) * 255.0);
}


void forward(float dist, float speed)
{
  dir = FORWARD;
  
  int val = pwmVal(speed);
  curr_pwm = 1.36*val;
  // For now we will ignore dist and move
  // forward indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.


  if(dist > 0) deltaDist = dist;
  else deltaDist = 9999999;
  newDist = forwardDist + deltaDist;
  //Serial.print("val is ");
  //Serial.println(val);
  _analogWrite(LF, val);
  _analogWrite(RF, val);
  _analogWrite(LR, 0);
  _analogWrite(RR, 0);
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
  _analogWrite(LF, 0);
  _analogWrite(RF, 0);
  _analogWrite(LR, val);
  _analogWrite(RR, val);
  
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
  _analogWrite(RF, val);
  _analogWrite(LR, val);
  _analogWrite(RR, 0);
  _analogWrite(LF, 0);
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
  _analogWrite(RR, val);
  _analogWrite(LF, val);
  _analogWrite(RF, 0);
  _analogWrite(LR, 0);
}
    

// Stop Alex. To replace with bare-metal code later.
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
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}


void setup() {
  // put your setup code here, to run once:

  cli();
 
  setupEINT();
  enablePullups();
  initializeState();
  setupMotors();
  startMotors();
  AlexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  AlexCirc = PI * AlexDiagonal;
   Serial.begin(9600);
  sei();
}



void loop() {
  

// Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

reverse(0, 30);

delay(1000);

// Uncomment the code below for Week 9 Studio 2
  
 
/*
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
  }*/
}
 
