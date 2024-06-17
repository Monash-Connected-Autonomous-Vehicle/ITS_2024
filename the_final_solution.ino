#define NUM_MOTORS 4
#define ENC_COUNT_REV 330

class PID_control{
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral;
  public:
  // Establish the C constructor(initialise KP = 1)
  PID_control(): kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0){}
  // Set function
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  }

  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, int &dir){
    // error
    int e = target - value;
  
    // derivative
    float dedt = (e-eprev)/(deltaT);
  
    // integral
    eintegral = eintegral + e*deltaT;
    
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
  
    // motor power 
    pwr = (int) fabs(u); // pass control signal to power 
    if( pwr > umax ) // cap the power 
    { 
      pwr = umax;
    }
  
    // motor direction
    dir = 1; 
    if(u<0) // if target less than value, car travelling faster than target speed then reverse direction?
    { 
      dir = -1;
    }
  
    // store previous error
    eprev = e;
  }
};

// right wheel in13 -> in 24 and enca -> encb :))) 
const int in13[]     = {5, 2, 14, 26};     // Define the direction pin for Motor Driver
const int in24[]     = {4, 15, 13, 27};     // Define the direction pin for Motor Driver
const int enca[]     = {18, 19, 32, 35};
const int encb[]     = {22, 23, 25, 33};

/****************************************** define global variables *************************************/

// Velocity calculation variables
long prevT = 0; // Define the time constants
volatile int posPrev[]        = {0, 0, 0, 0}; // Define the prvious position of two motors
float v1[]                    = {0, 0, 0, 0}; // Define the RPM for vel 1 (first method)
float v2[]                    = {0, 0, 0, 0}; // Define the RPM for vel 2 (second method)

// interrupt variables mainly encoder
volatile int pos_i[]          = {0, 0, 0, 0};             // get position from interrupt 
volatile int currT_i[]        = {0, 0, 0, 0};        // store current time from micros()
volatile long prevT_i[]       = {0, 0, 0, 0};       // store previous time from micros()
volatile float deltaT_i[]     = {0, 0, 0, 0};     // store change in time between signals
volatile float velocity2_i[]  = {0, 0, 0, 0};  // store calculated velocity 2 (in count/s)

// Low Pass Filter variable array of 2 element which will represent the speed of left and right side of the vehicle 
volatile float v1Filt[] = {0, 0}; 
volatile float v1Prev[] = {0, 0};
volatile float v2Filt[] = {0, 0};
volatile float v2Prev[] = {0, 0};

// pwm channels 
const int PWM_CHANNELS[] = {0, 1, 2, 3};
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;
const int MAX_DUTY_CYCLE = pow(2, PWM_RESOLUTION) - 1;

// Define PID_control class objects
PID_control pid[2]; // only control left and right wheels 

// Serial Data
float wl_goal = 0;
float wr_goal = 0;

// get the fuckign target speed 
void parseStringToFloats(String input, float &wl_goal, float &wr_goal) {
  // example input "wl:[34.5423], wr[1.4134]"
  
  int numstart1 = input.indexOf("["); 
  int numend1 = input.indexOf("]");

  int numstart2 = input.indexOf("[", numend1 + 1); 
  int numend2 =   input.indexOf("]", numend1 + 1);

  wl_goal = input.subtring(numstart1+1,numend1).toFloat();
  wr_goal = input.subtring(numstart2+1,numend2).toFloat();
}

// setup code
void setup() {
  Serial.begin(9600);
  for (int i = 0; i < NUM_MOTORS; i++) 
  {
    // setup encoder pin
    pinMode(enca[i], INPUT);
    pinMode(encb[i], INPUT);
    // setup pwm channel
    ledcSetup(PWM_CHANNELS[i], PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(PWM_CHANNELS[i],0); // write to zero initially 
  }
  // setup pid 
  pid[0].setParams(1,0,0,MAX_DUTY_CYCLE);
  pid[1].setParams(1,0,0,MAX_DUTY_CYCLE);
  
  // Trigger an interrupt when encoder A rises
  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder<2>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[3]), readEncoder<3>, RISING);
  
  // setup in1,2 motor driver pins
  // left side
  ledcAttachPin(in13[0],PWM_CHANNELS[1]);
  ledcAttachPin(in24[0],PWM_CHANNELS[0]);
  ledcAttachPin(in13[1],PWM_CHANNELS[1]);
  ledcAttachPin(in24[1],PWM_CHANNELS[0]);

  // right side
  ledcAttachPin(in13[2],PWM_CHANNELS[3]);
  ledcAttachPin(in24[2],PWM_CHANNELS[2]);
  ledcAttachPin(in13[3],PWM_CHANNELS[3]);
  ledcAttachPin(in24[3],PWM_CHANNELS[2]);

}

// loop code
void loop() {
  // Read Serial Data!
  if (Serial.available() > 0) 
  {
    String data = Serial.readStringUntil('\n');
    parseStringToFloats(data, wl_goal, wr_goal);
  }

//----------------- 1. Define the target velocity-----------------------
  float vtarget[2]; 
  // 0: target velocity of the left motor 
  // 1: target velocity of the right motor  
  vtarget[0] = wl_goal; 
  vtarget[1] = wr_goal;

//------------------ 2. Initialise position and make sure position updates----------------------
  int pos[] = {0, 0, 0, 0}; // The current position of the motor 
  float velocity2[] = {0, 0, 0, 0}; //The current velocity of the motor 
  // Note here "pos_i" and "velocity_i" are both variables from the interrrupt callback.
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    pos[i] = pos_i[i];
    velocity2[i] = velocity2_i[i];
  }

//------------------ 3. Update the velocity----------------------
  // Compute velocity with method 1 (fixed time interval)
  // Compute the change in speed 
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/1.0e6; // Change in T
  float velocity1[NUM_MOTORS]; // Define the current velocity
  // Velocity for Motor 1
  for (int i = 0; i<NUM_MOTORS; i++)
  {
    velocity1[i] = (pos[i] - posPrev[i])/deltaT;
    posPrev[i] = pos[i];
  }
  prevT = currT;

//------------------ 4. Convert counts/s to RPM ----------------------------
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    v1[i] = (velocity1[i]/ENC_COUNT_REV)*60;
    v2[i] = (velocity2[i]/ENC_COUNT_REV)*60;
  }

  // Low-pass Filter (40 Hz Cutoff Pretty Good) <- Like Really need to change
  v1Filt[0] = 0.843*v1Filt[0] + 0.0782*(v1[0]+v1[1])/2 + 0.0782*v1Prev[0];
  v1Prev[0] = (v1[0]+v1[1])/2 ; 
  
  v2Filt[0] = 0.843*v2Filt[0] + 0.0782*(v2[0]+v2[1])/2 + 0.0782*v2Prev[0];
  v2Prev[0] = (v2[0]+v2[1])/2;

  v1Filt[1] = 0.843*v1Filt[1] + 0.0782*(v1[2]+v1[3])/2 + 0.0782*v1Prev[1];
  v1Prev[1] = (v1[2]+v1[3])/2; 

  v2Filt[1] = 0.843*v2Filt[1] + 0.0782*(v2[2]+v2[3])/2 + 0.0782*v2Prev[1];
  v2Prev[1] = (v2[2]+v2[3])/2;
  
//------------------ 5. Implement PID control ----------------------------
  for(int k = 0; k < NUM_MOTORS/2; k++)
  {
    int pwr, dir;
    // evaluate the control signal
    pid[k].evalu(vtarget[k],v1filt[k],deltaT,pwr,dir);
    // signal the motor
    setMotor(dir,pwr,k);
  }
  delay(1); // Ensure consistent sampling frequency
}

// fucntion that switches pwm channels on or off depending in order to achieve the desired direction of movement
void setMotor(int dir, int pwmVal, int left_OR_right) 
{
  if (dir == 1) 
  {
    ledcWrite(PWM_CHANNELS[2*left_OR_right + 1], pwmVal);
    ledcWrite(PWM_CHANNELS[2*left_OR_right], 0);
  } 
  else if (dir == -1) 
  {
    ledcWrite(PWM_CHANNELS[2*left_OR_right + 1], 0);
    ledcWrite(PWM_CHANNELS[2*left_OR_right], pwmVal);
  }
}

// function to read output signal from encoder, increases by 1 when moving forward, decreases when in reverse
template <int j>
void readEncoder() 
{
  int increment_i = 0; 
  int b = digitalRead(encb[j]); // read encb 
  if(b>0) {
    increment_i = 1;
  }
  else {
    increment_i = -1;
  }

  pos_i[j] += increment_i; 
  /* calculate the velocity with method 2 */
  long currTj = micros(); // get the current time 
  deltaT_i[j] = ((float) (currTj - prevT_i[j]))/1.0e6; // store the time between each signals 
  velocity2_i[j] = increment_i/deltaT_i[j]; // velocity 
  prevT_i[j] = currTj;
}
