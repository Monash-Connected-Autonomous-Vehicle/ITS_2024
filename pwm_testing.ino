


#define in1 13 
#define in2 14 

#define in3 27
#define in4 26

//#define ENCA 35 // M+
//#define ENCB 34 // M-

const int PWM_CHANNEL0 = 0; // ESP32 has 16 channels which can generate 16 independent waveforms
const int PWM_CHANNEL1 = 1; 
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

  // odd channel goes odd in , even channel goes even in 
  ledcAttachPin(in1, PWM_CHANNEL1); // attach pins to channel to generate pwm
  ledcAttachPin(in3, PWM_CHANNEL1);
  
  ledcAttachPin(in2, PWM_CHANNEL0);
  ledcAttachPin(in4, PWM_CHANNEL0);
  
  ledcWrite(PWM_CHANNEL0, 0); // initialise by writing one to channel 
  ledcWrite(PWM_CHANNEL1, 0);
  
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




