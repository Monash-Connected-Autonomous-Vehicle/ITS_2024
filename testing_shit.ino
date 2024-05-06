#include <util/atomic.h> 

// ********************************************* PID CLASS ********************************************* //
// *********************************************************************************************************************** //
// *********************************************************************************************************************** //
class PID_control {
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral;
  public:
    // Establish the default constructor(initialise KP = 1, max pwm = 255) this always run when a variable created
    PID_control(): kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}
    // Set function
    void setParams(float kpIn, float kiIn, float kdIn, float umaxIn) {
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


// ********************************************* GLOBAL VARIABLES ******************************************************** //
// *********************************************************************************************************************** //
// *********************************************************************************************************************** //

// Define the number of motors
#define NUM_MOTORS 4
// Define the number of counts of encoder for a full revolution.
#define ENC_COUNT_REV 330.0

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

// ********************************************* SETUP PWM CHANNELS ********************************************* //
// *********************************************************************************************************************** //
// *********************************************************************************************************************** //

const int PWM_CHANNELS[NUM_MOTORS] = {0,1,2,3} 
const int PWM_FREQ = 500;  
const int PWM_RESOLUTION = 8; 
const int MAX_DUTY_CYCLE = (int)(pow(2, PWM_RESOLUTION) - 1);


// ********************************************* GET TARGET SPEED FROM RASPI ********************************************* //
// *********************************************************************************************************************** //
// *********************************************************************************************************************** //

void getStr2F(String input, float &wl_target, float &wr_target) {
  int index = input.indexOf("[") + 1; // Find the starting index of the values
  int endIndex = input.indexOf("]"); // Find the ending index of the values

  String valuesStr = input.substring(index, endIndex); // Extract the values between '[' and ']'
  valuesStr.replace(" ", ""); // Remove any spaces
  commaIndex = valuesStr.indexOf(",");
  wl_goal = valuesStr.substring(0, commaIndex).toFloat();
  valuesStr = valuesStr.substring(commaIndex + 1);
  
  wr_goal = valuesStr.toFloat(); // The remaining value is the last one
}


// ********************************************* SETUP FUNCTION ********************************************************** //
// *********************************************************************************************************************** //
// *********************************************************************************************************************** //

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // setup pwm channels
  for (int i = 0; i<NUM_MOTORS; i++)
  {
    ledcSetup(PWM_CHANNEL[i], PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(PWM_CHANNEL[i],0); // write to zero initially
  }

  // left side
  ledcAttachPin(in13[0],PWM_CHANNEL[1])
  ledcAttachPin(in13[1],PWM_CHANNEL[1])
  ledcAttachPin(in24[0],PWM_CHANNEL[0])
  ledcAttachPin(in24[1],PWM_CHANNEL[0])
  // right side 
  ledcAttachPin(in13[2],PWM_CHANNEL[3])
  ledcAttachPin(in13[3],PWM_CHANNEL[3])
  ledcAttachPin(in24[2],PWM_CHANNEL[2])
  ledcAttachPin(in24[3],PWM_CHANNEL[2])

  // Set up PINS and PID Params
  for (int k = 0; k < NUM_MOTORS; k++) {
    pid[k].setParams(10, 0, 8, MAX_DUTY_CYCLE);
  }
  
  // Trigger an interrupt when encoder A rises
  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder<2>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[3]), readEncoder<3>, RISING);
}

// ********************************************* LOOP FUNCTION ********************************************* //
// *********************************************************************************************************************** //
// *********************************************************************************************************************** //

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
  float v1 = velocity1/ENC_COUNT_REV*60.0;
  float v2 = velocity2/ENC_COUNT_REV*60.0;

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
    setMotor(dir, pwr);
  } 
}


// ********************************************* MOTOR CONTROLLER FUNCTION *********************************************** //
// *********************************************************************************************************************** //
// *********************************************************************************************************************** //
void setMotor(int dir, int pwmVal) {
  if(dir == 1){
    ledcWrite(PWM_CHANNEL[1], pwmVal);
    ledcWrite(PWM_CHANNEL[3], pwmVal);
    ledcWrite(PWM_CHANNEL[0], 0);
    ledcWrite(PWM_CHANNEL[2], 0);
  }
  else if(dir == -1){
    ledcWrite(PWM_CHANNEL[1], 0);
    ledcWrite(PWM_CHANNEL[3], 0);
    ledcWrite(PWM_CHANNEL[0], pwmVal);
    ledcWrite(PWM_CHANNEL[2], pwmVal);
  }
  else{
    ledcWrite(PWM_CHANNEL[1], 0);
    ledcWrite(PWM_CHANNEL[3], 0);
    ledcWrite(PWM_CHANNEL[0], 0);
    ledcWrite(PWM_CHANNEL[2], 0);
  }  
}

// ********************************************* READ ENCODER FUNCTION *************************************************** //
// *********************************************************************************************************************** //
// *********************************************************************************************************************** //
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
