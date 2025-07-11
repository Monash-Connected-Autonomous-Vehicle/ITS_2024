/***********************
  before upload please run: sudo chmod a+rw /dev/ttyUSB0
  ensure serial monitor/plotter is closed before pyserial script is run
***********************/
#include <Arduino.h>
class PID_control{
  private:
    float kp, kd, ki, umax, uprev; // Parameters
    float eprev, eintegral;
  public:
  // Establish the C constructor(initialise KP = 1)
  PID_control(): kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0), uprev(0.0){}
  // Set function, init

  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  }
  // A function to compute the control signal
  void evalu(float value, float target, float deltaT, int &pwr){
    // error
    float e = target - value;
    // derivative
    float dedt = (e-eprev)/(deltaT);
    // integral
    eintegral = eintegral + e*deltaT;
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
    // motor power
    pwr = (int) fabs(u);
    if(eintegral>2)
    {eintegral=2;}
    if( pwr > umax ){
      pwr = 255;
    }
    // motor direction
//    dir = 1;
//    if(u<0){
//      dir = -1;
//    }
    // store previous error
    eprev = e;
  }
};
/*************************************************************************************
Manually driving params
**************************************************************************************/
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define STOP 5
int command;
/*************************************************************************************/

#define NUM_MOTORS 4
#define ENC_COUNT_REV 330
const uint8_t enca[4] = {19,18,25,33};
const uint8_t encb[4] = {23,22,32,35};
// pwm channels
//const int PWM_CHANNELS[] = {0, 1, 2, 3, 4, 5, 6, 7};
const int PWM_FREQ = 2000;
const int PWM_RESOLUTION = 8;
const int MAX_DUTY_CYCLE = pow(2, PWM_RESOLUTION) - 1;
/* ************************************************ */
String received = "";
// Velocity calculation variables
long prevT = 0; // Tracking previous time tick
volatile int pos_i[4] = {0,0,0,0}; // pos interrupt
volatile float velocity_i[4] = {0,0,0,0}; // velocity measured in interrupt
volatile int posPrev[4] = {0,0,0,0};
volatile long prevT_i[4] = {0,0,0,0};
volatile long currT[4] = {0,0,0,0};
int pwr[]={0,0,0,0};

// Time calculation variables
long prevT_loop = 0;
long deltaT = 0; // time interval for each loop
// Filter variables
float v1Filt[4] = {0,0,0,0};
float v1Prev[4] = {0,0,0,0};
float vtarget[] = {0,0,0,0};
float wl = 0.0, wr = 0.0;
/* ************************************************ */
PID_control pid[4];
void IRAM_ATTR Encoder_ISR0() {
  currT[0] = micros();
  velocity_i[0] = 60*1.0e6/((currT[0]-prevT_i[0])*330);
  prevT_i[0] = currT[0];
}
void IRAM_ATTR Encoder_ISR1() {
  currT[1] = micros();
  velocity_i[1] = 60*1.0e6/((currT[1]-prevT_i[1])*330);
  prevT_i[1] = currT[1];
}
void IRAM_ATTR Encoder_ISR2() {
  currT[2] = micros();
  velocity_i[2] = 60*1.0e6/((currT[2]-prevT_i[2])*330);
  prevT_i[2] = currT[2];
}
void IRAM_ATTR Encoder_ISR3() {
  currT[3] = micros();
  velocity_i[3] = 60*1.0e6/((currT[3]-prevT_i[3])*330);
  prevT_i[3] = currT[3];
}

void parseStringToFloats (String input, float &wl, float &wr){
  input.replace("wl:", "");
  input.replace(" wr:", "");
  int commaIndex = input.indexOf(",");

  if (commaIndex != -1) {
    wl = input.substring(0, commaIndex).toFloat();
    wr = input.substring(commaIndex+1).toFloat();
  }

}
void GPIO_init(){
  
  pinMode(enca[0], INPUT_PULLUP);
  pinMode(enca[1], INPUT_PULLUP);
  pinMode(enca[2], INPUT_PULLUP);
  pinMode(enca[3], INPUT_PULLUP);
  }

void PID_control_init(PID_control *pid){
  pid[0].setParams(200,0,400,MAX_DUTY_CYCLE);
  pid[1].setParams(200,0,400,MAX_DUTY_CYCLE);
  pid[2].setParams(200,0,400,MAX_DUTY_CYCLE);
  pid[3].setParams(200,0,400,MAX_DUTY_CYCLE);
//  pid[0].setParams(50,0,100,MAX_DUTY_CYCLE);
//  pid[1].setParams(50,0,100,MAX_DUTY_CYCLE);
//  pid[2].setParams(50,0,100,MAX_DUTY_CYCLE);
//  pid[3].setParams(50,0,100,MAX_DUTY_CYCLE);
  }

void encoder_interr_init(){
  attachInterrupt(enca[0], Encoder_ISR0, RISING); // M1
  attachInterrupt(enca[1], Encoder_ISR1, RISING); // M2
  attachInterrupt(enca[2], Encoder_ISR2, RISING); // M3
  attachInterrupt(enca[3], Encoder_ISR3, RISING); // M4
  }

void PWM_init(const int PWM_FREQ , const int PWM_RESOLUTION ){
  const uint8_t in13[4] = {2,5,14,26};
  const uint8_t in24[4] = {15,4,13,27};
  const int PWM_CHANNELS[] = {0, 1, 2, 3, 4, 5, 6, 7};
  const int MAX_DUTY_CYCLE = pow(2, PWM_RESOLUTION) - 1;
  for (int i=0;i<8;i++)
  {
    ledcSetup(PWM_CHANNELS[i], PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(PWM_CHANNELS[i],0);
    }

   for (int i=0;i<4;i++)
   {
      ledcAttachPin(in13[i],PWM_CHANNELS[i]);
      ledcAttachPin(in24[i],PWM_CHANNELS[i+4]);
    }

  }

float filter(float *v1Filt, volatile float *velocity_i, float *v1Prev){
  float vleft, vright;
  for (int i=0;i<4;i++)
  {
    v1Filt[i] = 0.843*v1Filt[i] + 0.0782*velocity_i[i] + 0.0782*v1Prev[i];
    v1Prev[i] = velocity_i[i];
    vleft=(v1Filt[0]+v1Filt[1])/2;
    vright=(v1Filt[2]+v1Filt[3])/2;
  }
  
  
  return vright, vleft;
  
}


void PID_driving(float *target_v, float *v, float deltaT, int *pwr, PID_control *pid){
  const int PWM_CHANNELS[] = {0, 1, 2, 3, 4, 5, 6, 7};
  float r=31.5*pow(10,-3);
  float pi=3.14;
  float l=2*pi*r;
  float v_mps[4];
  for (int i=0; i<4;i++){
    v_mps[i]=l*fabs(v[i])/60;
    if((fabs(target_v[i])>=0.15) && (fabs(v_mps[i])<=0.15))
    {
      pwr[i]=200;
      delay(50);
    }
      if (target_v[i]==0)
         {
          ledcWrite(PWM_CHANNELS[i], 0);
          ledcWrite(PWM_CHANNELS[i+4], 0);
          }

    
     else if(target_v[i]>0)
       {
         if(target_v[i]<0.2 || (target_v[i]>0.4 && v_mps[i]>0.4))
         {
          if(pwr[i]<100)
            {pwr[i]=100;}
          pid[i].setParams(50,0,100,MAX_DUTY_CYCLE);
          }
         else
         {pid[i].setParams(200,0,400,MAX_DUTY_CYCLE);}
         pid[i].evalu(v_mps[i], fabs(target_v[i]), deltaT, pwr[i]);
         ledcWrite(PWM_CHANNELS[i], pwr[i]);
         ledcWrite(PWM_CHANNELS[i+4], 0);
       }
     else
        {
          if(target_v[i]>-0.2 || ((target_v[i]<-0.4 && v_mps[i]<-0.4)))
          {
            if(pwr[i]<100)
            {pwr[i]=100;}
            pid[i].setParams(50,0,100,MAX_DUTY_CYCLE);
            }
          else
          {pid[i].setParams(200,0,400,MAX_DUTY_CYCLE);}
          pid[i].evalu(v_mps[i], fabs(target_v[i]), deltaT, pwr[i]);
          ledcWrite(PWM_CHANNELS[i], 0);
          ledcWrite(PWM_CHANNELS[i+4], pwr[i]);
      }
      
    
    if(i==3)
    {delay(20);}
    
    } 
}

void manually_driving(int driving_mode, float *target_v, float *v, float deltaT, int *pwr, PID_control *pid)
{
  switch(driving_mode)
  {
   case FORWARD:
   target_v[0]= 0.3;
   target_v[1]= 0.3;
   target_v[2]= 0.3;
   target_v[3]= 0.3;
   break;
   case BACKWARD:
   target_v[0]= -0.5;
   target_v[1]= -0.5;
   target_v[2]= -0.5;
   target_v[3]= -0.5;
   break;
   case RIGHT:
   target_v[0]= -0.4;
   target_v[1]= -0.4;
   target_v[2]= 0.4;
   target_v[3]= 0.4;
   break;
   case LEFT:
   target_v[0]= 0.4;
   target_v[1]= 0.4;
   target_v[2]= -0.4;
   target_v[3]= -0.4;
   break;
   case STOP:
   target_v[0]= 0;
   target_v[1]= 0;
   target_v[2]= 0;
   target_v[3]= 0;
   break;
   default:
   target_v[0]= 0;
   target_v[1]= 0;
   target_v[2]= 0;
   target_v[3]= 0;
   break;
    }
  PID_driving(target_v,v,deltaT,pwr,pid);
  }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  GPIO_init();
  PID_control_init(pid);
  encoder_interr_init();
  // setup pwm channel
  PWM_init(2000,8);
}

void loop() {
  
  /************************  read data *********************************/
  if (Serial.available() > 0){
    received = Serial.readStringUntil('\n');
    if(received == "FORWARD"){
      command = FORWARD;
    } 
    else if (received == "BACKWARD") {
      command = BACKWARD;
    } 
    else if (received == "LEFT") {
      command = LEFT;
    }
    else if (received == "RIGHT") {
      command = RIGHT;
    }
    else if (received == "STOP") {
      command = STOP;
    }
  
    //parseStringToFloats(received, wl, wr);
  }
  /************************  get measured speed *****************************/
  //volatile int pos[4] = {pos_i[0],pos_i[1],pos_i[2],pos_i[3]}; // update current pos
  long currT_loop = micros();
  float deltaT = ((float) (currT_loop - prevT_loop))/1.0e6; // Change in Tse
  prevT_loop = currT_loop;
  // filter
  filter(v1Filt, velocity_i, v1Prev);
  float v[]={v1Filt[0],v1Filt[1],v1Filt[2],v1Filt[3]};
  /*************************** set motor speed *****************************/
  float v_test[]={0,0,0,0};
  vtarget[0] = wl;
  vtarget[1] = wl;
  vtarget[2] = wr;
  vtarget[3] = wr;
  //PID_driving(vtarget,v,deltaT,pwr,pid);
  manually_driving(command, vtarget, v, deltaT, pwr, pid);
  for (int i=0;i<4;i++)
  {
    v_test[i]=2*3.14*31.5*pow(10,-3)*v[i]/60;
    Serial.print("v");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(vtarget[i]);
    Serial.print(" ");
    Serial.print("v ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(v_test[i]);
    Serial.print(" ");
    if(i==3){
      Serial.println();
    }
  }
  
  delay(1);
  
}
