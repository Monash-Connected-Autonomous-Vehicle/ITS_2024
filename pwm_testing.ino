

// RIGHT WHEELS INPUT PINS
#define in1 13 
#define in2 14 

#define in3 25
#define in4 26

// LEFT WHEELS INPUT PINS
#define IN1 32
#define IN2 33

#define IN3 34
#define IN4 35


//#define ENCA 35 // M+
//#define ENCB 34 // M-

const int PWM_CHANNEL0 = 0; // ESP32 has 16 channels which can generate 16 independent waveforms
const int PWM_CHANNEL1 = 1; 
const int PWM_CHANNEL2 = 2;
const int PWM_CHANNEL3 = 3;
const int PWM_FREQ = 500;  // Recall that Arduino Uno is ~490 Hz. Official ESP32 example uses 5,000Hz

const int PWM_RESOLUTION = 8; 
const int MAX_DUTY_CYCLE = (int)(pow(2, PWM_RESOLUTION) - 1);


//const int DELAY_MS = 4;    // delay between fade increments
//int _ledFadeStep = 5; // amount to fade per loop


// the setup function runs once when you press reset or power the board
void setup() {
  // Set pin states of the encoder
//  pinMode(ENCA , INPUT);
//  pinMode(ENCB , INPUT);
  
//  attachInterrupt(digitalPinToInterrupt(ENCA), set_pos, RISING);

  ledcSetup(PWM_CHANNEL0, PWM_FREQ, PWM_RESOLUTION); // set up the pwm channel with pwm channel number, frequency and resolution
  ledcSetup(PWM_CHANNEL1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL3, PWM_FREQ, PWM_RESOLUTION);
  // odd channel goes odd in , even channel goes even in 
  ledcAttachPin(in1, PWM_CHANNEL1); // attach pins to channel to generate pwm
  ledcAttachPin(in3, PWM_CHANNEL1);
  
  ledcAttachPin(in2, PWM_CHANNEL0);
  ledcAttachPin(in4, PWM_CHANNEL0);

  ledcAttachPin(IN1, PWM_CHANNEL3); // attach pins to channel to generate pwm
  ledcAttachPin(IN3, PWM_CHANNEL3);
  
  ledcAttachPin(IN2, PWM_CHANNEL2);
  ledcAttachPin(IN4, PWM_CHANNEL2);
  
  ledcWrite(PWM_CHANNEL0, 0); // initialise by writing one to channel 
  ledcWrite(PWM_CHANNEL1, 0);
  ledcWrite(PWM_CHANNEL2, 0); // initialise by writing one to channel 
  ledcWrite(PWM_CHANNEL3, 0);
}

// the loop function runs over and over again forever
void loop() {

  for (int i = 0; i <= MAX_DUTY_CYCLE; i++)
  {
    ledcWrite(PWM_CHANNEL1,i);
    ledcWrite(PWM_CHANNEL3,i);
    delay(4);
  }
  delay(2000); // 1sec
  ledcWrite(PWM_CHANNEL1,0);
  ledcWrite(PWM_CHANNEL3,0);
  delay(500);
  
  for (int i = 0; i <= MAX_DUTY_CYCLE; i++)
  {
    ledcWrite(PWM_CHANNEL0,i);
    ledcWrite(PWM_CHANNEL2,i);
    delay(4);
  }
  delay(2000);
  ledcWrite(PWM_CHANNEL0,0);
  ledcWrite(PWM_CHANNEL2,0);
  delay(500);

  /* code to drive motors forward and in reverse
    // Forward Motion
    for (int i = 0; i <= MAX_DUTY_CYCLE; i++) {
      ledcWrite(PWM_CHANNEL1, i);  // in1 = High, in3 = High
      ledcWrite(PWM_CHANNEL0, 0);  // in2 = Low, in4 = Low
      ledcWrite(PWM_CHANNEL3, i);  // IN1 = High, IN3 = Low
      ledcWrite(PWM_CHANNEL2, 0);  // IN2 = Low, IN4 = Low
      delay(4);
    }
    delay(2000); // maintain forward speed
    ledcWrite(PWM_CHANNEL1, 0); // in1 = Low, in3 = Low
    ledcWrite(PWM_CHANNEL3, 0); // IN1 = Low, IN3 = Low
    delay(500);
  
    // Reverse Motion, switching states of PWM CHANNELS
    for (int i = 0; i <= MAX_DUTY_CYCLE; i++) {
      ledcWrite(PWM_CHANNEL0, i);  // in2 = High, in4 = High
      ledcWrite(PWM_CHANNEL1, 0);  // in1 = Low, in3 = Low
      ledcWrite(PWM_CHANNEL2, i);  // IN2 = High, IN4 = High
      ledcWrite(PWM_CHANNEL3, 0);  // IN1 = Low, IN3 = Low
      delay(4);
    }
    delay(2000); // maintain reverse speed
    ledcWrite(PWM_CHANNEL0, 0); // in2 = Low, in4 = Low
    ledcWrite(PWM_CHANNEL2, 0); // IN2 = Low, IN4 = Low
    delay(500);
  */
}




