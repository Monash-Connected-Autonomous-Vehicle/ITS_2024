
#define NUM_MOTORS 4
#define ENC_COUNT_REV 330

// right wheel in13 -> in 24 :))) 
const int in13[]     = {5, 2, 14, 26};     // Define the direction pin for Motor Driver
const int in24[]     = {4, 15,13, 27};     // Define the direction pin for Motor Driver
const int enca[]     = {18,19,25,33};
const int encb[]     = {22,23,32,35};

// 0: back left
// 1: front left
// 2: front right
// 3: back right 

const int PWM_CHANNELS[] = {0,1,2,3}; // 2 channels each side (1 for forward, 1 for backward)
// 0: going in2 for left side 
// 1: going in1 for left side
// 2: going in2 for right side
// 3: going in1 for right side

const int PWM_FREQ = 5000;  // Recall that Arduino Uno is ~490 Hz. Official ESP32 example uses 5,000Hz
const int PWM_RESOLUTION = 8; 
const int MAX_DUTY_CYCLE = (int)(pow(2, PWM_RESOLUTION) - 1);


// the setup function runs once when you press reset or power the board
void setup() {
  
  // setup pwm channels
  for (int i = 0; i<NUM_MOTORS; i++)
  {
    ledcSetup(PWM_CHANNELS[i], PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(PWM_CHANNELS[i],0); // write to zero initially
  }
  // setup in1,2 motor driver pins
  //// left side
  // back left
  ledcAttachPin(in13[0],PWM_CHANNELS[1]); 
  ledcAttachPin(in24[0],PWM_CHANNELS[0]);
  // front left
  ledcAttachPin(in13[1],PWM_CHANNELS[1]);
  ledcAttachPin(in24[1],PWM_CHANNELS[0]);
  
  //// right side 
  ledcAttachPin(in13[2],PWM_CHANNELS[3]);
  ledcAttachPin(in24[2],PWM_CHANNELS[2]);
  
  ledcAttachPin(in13[3],PWM_CHANNELS[3]);
  ledcAttachPin(in24[3],PWM_CHANNELS[2]);
  
}


// the loop function runs over and over again forever
void loop() {

  for (int i = 0; i <= MAX_DUTY_CYCLE; i++)
  {
    ledcWrite(PWM_CHANNELS[1],i);
    ledcWrite(PWM_CHANNELS[3],i);
    delay(4);
  }
  delay(500); // 2 sec
  ledcWrite(PWM_CHANNELS[1],0);
  ledcWrite(PWM_CHANNELS[3],0);
  delay(500);
  
  for (int i = 0; i <= MAX_DUTY_CYCLE; i++)
  {
    ledcWrite(PWM_CHANNELS[0],i);
    ledcWrite(PWM_CHANNELS[2],i);
    delay(4);
  }
  delay(500);
  ledcWrite(PWM_CHANNELS[0],0);
  ledcWrite(PWM_CHANNELS[2],0);
  delay(500);
  
}
