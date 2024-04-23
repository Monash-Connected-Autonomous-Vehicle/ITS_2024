#include <util/atomic.h> // ??
//#include "ESP32_PWM.h"


////////////////////////////////////
// Establish the PID control class//
////////////////////////////////////
class PID_control {
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral;
  public:
    // Establish the default constructor(initialise KP = 1, max pwm = 255) this always run when a variable created
    PID_control(): kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}
    // Set function
    void setParams(float kpIn, float kdIn, float kiIn, float umaxIn) {
      kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
    }

    // A function to compute the control signal
    void evalu(int value, int target, float deltaT, int &pwr, int &dir) {
      // error
      int e = target - value;

      // derivative
      float dedt = (e - eprev) / (deltaT);

      // integral
      eintegral = eintegral + e * deltaT;
      
      // control signal
      float u = kp * e + kd * dedt + ki * eintegral;

      // motor power
      pwr = (int) fabs(u); // fabs(x) get absolute/magnitude of a floating number 
      if ( pwr > umax ) { // cap the control signal
        pwr = umax;
      }

      // motor direction based on the sign of control signal
      dir = 1;
      if (u < 0) {
        dir = -1;
      }
      
      // store previous error
      eprev = e;
    }
};


////////////////////////////////////
// Define the Global Variables//////
////////////////////////////////////

// Define the number of motors
#define NUM_MOTORS 4
// Define the number of counts of encoder for a full revolution.
#define ENC_COUNT_REV 330

// Define the PINS
const int enca[]     = {36, 39, 34, 35};     // Define the input pin for Encoder A (pins needs interrupt capabilities) 
const int encb[]     = {23, 22, 1, 3};     // Define the input pin for Encoder B (pins needs interrupt capabilities) 
const int in13[]     = {32, 33, 25, 26};     // Define the direction pin for Motor Driver
const int in24[]     = {27, 14, 12, 13};     // Define the direction pin for Motor Driver

// Global Variables
// Velocity calculation variables
long prevT = 0;                         // Define the previous time point 
volatile int posCur[] = {0, 0, 0, 0};   // Define the current position to be updated by interrupt 

volatile float velocity_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;


//Define PID_control class objects
PID_control pid[NUM_MOTORS];

// define pwm 
const int PWM_CHANNEL1 = 1; // ESP32 has 16 channels which can generate 16 independent waveforms
const int PWM_CHANNEL2 = 2;
const int PWM_FREQ = 500;  // Recall that Arduino Uno is ~490 Hz. Official ESP32 example uses 5,000Hz
const int PWM_RESOLUTION = 8; 
// The max duty cycle value based on PWM resolution (will be 255 if resolution is 8 bits)
const int MAX_DUTY_CYCLE = (int)(pow(2, PWM_RESOLUTION) - 1);


////////////////////////////////////
////////// Define SETUP ////////////
////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  ledcSetup(PWM_CHANNEL1, PWM_FREQ, PWM_RESOLUTION); // set up the pwm channel with pwm channel number, frequency and resolution
  ledcSetup(PWM_CHANNEL2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(in13, PWM_CHANNEL1); // attach pins to channel to generate pwm
  ledcAttachPin(in24, PWM_CHANNEL2);
  ledcWrite(PWM_CHANNEL1, 0); // initialise by writing one to channel
  ledcWrite(PWM_CHANNEL2, 0); // initialise by writing one to channel

  // Set up PINS and PID Params
  for (int k = 0; k < NUM_MOTORS; k++) {
    pinMode(enca[k], INPUT);
    pinMode(encb[k], INPUT);
    pinMode(in1[k], OUTPUT);
    pinMode(in2[k], OUTPUT);
    pid[k].setParams(10, 0, 8, 255);
  }
  
  // Trigger an interrupt when encoder A rises
  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder<2>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[3]), readEncoder<3>, RISING);
}

////////////////////////////////////
////////////// LOOP ////////////////
////////////////////////////////////

void loop() {
  // set target position
  int target[NMOTORS];
  target[0] = 750*sin(prevT/1e6);
  target[1] = -750*sin(prevT/1e6);

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;
  
  // read the position in an atomic block to avoid a potential misread
  int posTemp[NUM_MOTORS];
  int velTemp[NUM_MOTORS];
  // Atomic Block will block other code from running when executing. Therefore ensuring memeory integrity
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (int k = 0; k < NUM_MOTORS; k++) {
      posTemp[k] = posCur[k];
    }
  }
  // Convert count/s to RPM
  float v1 = velocity1/600.0*60.0;
  float v2 = velocity2/600.0*60.0;

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;

  // Set a target
  float vt = 100*(sin(currT/1e6)>0);
  
  //------------------ Implement PID control ----------------------------
  for (int k = 0; k < NUM_MOTORS; k++) {
    int pwr, dir;
    // call PID evaluate to get update pwm and direction
    pid[k].evalu(posTemp[k], target[k], deltaT, pwr, dir);
    setMotor(dir, pwr, in1[k], in2[k]);
  } 
}


// MOTOR COMMUNICATION FUNCTIONS 
// Set the power and direction of the Motor
void setMotor(int dir, int pwmVal, int in1, int in2) {
  
  if(dir == 1){
    ledcWrite(PWM_CHANNEL1, pwmVal);
    ledcWrite(PWM_CHANNEL2, pwmVal);

  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

// Read Encoder Module for Motor i wher i = 0,1,...,N motor
template <int j>
void readEncoder() {
  int b = digitalRead(encb[j]);
  int increment = 0;
  if (b > 0) {
    increment = 1;
  }
  else {
    increment = -1;
  }
  posCur += increment;

  // calculate the velocity with method 2 
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;

}
