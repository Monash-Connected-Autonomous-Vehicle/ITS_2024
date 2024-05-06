
#define NUM_MOTORS 4
#define ENC_COUNT_REV 330



const int in13[NUM_MOTORS]     = {5, 2, 13, 27};     // Define the direction pin for Motor Driver
const int in24[NUM_MOTORS]     = {4, 15, 14, 26};     // Define the direction pin for Motor Driver
const int enca[NUM_MOTORS]     = {18,19,25,33};
const int encb[NUM_MOTORS]     = {22,23,32,35};

//const int PWM_CHANNEL0 = 0; // ESP32 has 16 channels which can generate 16 independent waveforms
//const int PWM_CHANNEL1 = 1;
const int PWM_CHANNELS[NUM_MOTORS] = {0,1,2,3} // 2 channels each side (1 for forward, 1 for backward)
const int PWM_FREQ = 5000;  // Recall that Arduino Uno is ~490 Hz. Official ESP32 example uses 5,000Hz

const int PWM_RESOLUTION = 8; 
const int MAX_DUTY_CYCLE = (int)(pow(2, PWM_RESOLUTION) - 1);


//const int DELAY_MS = 4;    // delay between fade increments
//int _ledFadeStep = 5; // amount to fade per loop

// the setup function runs once when you press reset or power the board
void setup() {
  

  // setup pwm channels
  for (int i = 0; i<NUM_MOTORS; i++)
  {
    ledcSetup(PWM_CHANNEL[i], PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(PWM_CHANNEL[i],0); // write to zero initially
  }
  // setup in1,2 motor driver pins
//  for (int k = 0; k < NUM_MOTORS; k++) 
//  {
//    ledcAttachPin(in13[k], PWM_CHANNEL[); 
//    ledcAttachPin(in24[k], PWM_CHANNEL0);
//  }
  
  // left side
  ledcAttachPin(in13[0],PWM_CHANNEL[1]
  ledcAttachPin(in13[1],PWM_CHANNEL[1]
  ledcAttachPin(in24[0],PWM_CHANNEL[0]
  ledcAttachPin(in24[1],PWM_CHANNEL[0]
  // right side 
  ledcAttachPin(in13[2],PWM_CHANNEL[3]
  ledcAttachPin(in13[3],PWM_CHANNEL[3]
  ledcAttachPin(in24[2],PWM_CHANNEL[2]
  ledcAttachPin(in24[3],PWM_CHANNEL[2]

}


// the loop function runs over and over again forever
void loop() {

  for (int i = 0; i <= MAX_DUTY_CYCLE; i++)
  {
    ledcWrite(PWM_CHANNEL1,i);
    delay(4);
  }
  delay(2000); // 1sec
  ledcWrite(PWM_CHANNEL1,0);
  delay(500);
  
  for (int i = 0; i <= MAX_DUTY_CYCLE; i++)
  {
    ledcWrite(PWM_CHANNEL0,i);
    delay(4);
  }
  delay(2000);
  ledcWrite(PWM_CHANNEL0,0);
  delay(500);
  
}

