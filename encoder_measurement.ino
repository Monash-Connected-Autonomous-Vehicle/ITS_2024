#define NUM_MOTORS 4
#define ENC_COUNT_REV 330

// define global variables
const int in13[NUM_MOTORS] = {5, 2, 14, 26};
const int in24[NUM_MOTORS] = {4, 15, 13, 27};
const int enca[NUM_MOTORS] = {18, 19, 25, 33};
const int encb[NUM_MOTORS] = {22, 23, 32, 35};
volatile int posCur[NUM_MOTORS] = {0, 0, 0, 0};
volatile int prevT[NUM_MOTORS] = {0, 0, 0, 0};
long prevT_i = 0;
volatile int currT[NUM_MOTORS] = {0, 0, 0, 0};
const int PWM_CHANNELS[] = {0, 1, 2, 3};
const int PWM_FREQ = 500;
const int PWM_RESOLUTION = 8;
const int MAX_DUTY_CYCLE = pow(2, PWM_RESOLUTION) - 1;

// setup code
void setup() {
  Serial.begin(9600);
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(enca[i], INPUT);
    pinMode(encb[i], INPUT);
    ledcSetup(PWM_CHANNELS[i], PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(PWM_CHANNELS[i],0); // write to zero initially
    
  // Trigger an interrupt when encoder A rises
  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder<2>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[3]), readEncoder<3>, RISING);
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
// function to read output signal from encoder, increases by 1 when moving forward, decreases when in reverse
template <int j>
void readEncoder() {
  int increment = 0;
  int b = digitalRead(encb[j]);
    if(b>0) {
      increment = 1;
    }
    else {
      increment = -1;
    }
    for(int i = 0; i<NUM_MOTORS; i++){
      posCur[i]+= increment;
    }
    /*calculate the velocity with method 2
    long currT = micros();
    float deltaT = ((float) (currT - prevT_i))/1.0e6;
    float velocity_i = increment/deltaT;
    prevT_i = currT;
    Serial.print("Speed: ");
    Serial.println(velocity_i); */
}

// loop code
void loop() {
  int target = 300;
  for (int i = 0; i < NUM_MOTORS; i++) {
    setMotor(1, 127);
    if (posCur[i] < target){
      Serial.print("Motor ");
      Serial.print(i);
      Serial.print(": Position = ");
      Serial.println(posCur[i]);
    } else {
      Serial.print("Motor ");
      Serial.print(i);
      Serial.print(" has passed target at Position = ");
      Serial.println(posCur[i]);
    }
  }
  delay(1000);
}

// fucntion that switches pwm channels on or off depending in order to achieve the desired direction of movement
void setMotor(int dir, int pwmVal) {
  if (dir == 1) {
    ledcWrite(PWM_CHANNELS[1], pwmVal);
    ledcWrite(PWM_CHANNELS[3], pwmVal);
    ledcWrite(PWM_CHANNELS[0], 0);
    ledcWrite(PWM_CHANNELS[2], 0);
  } else if (dir == -1) {
    ledcWrite(PWM_CHANNELS[1],0);
    ledcWrite(PWM_CHANNELS[3],0);
    ledcWrite(PWM_CHANNELS[0], pwmVal);
    ledcWrite(PWM_CHANNELS[2], pwmVal);
  } else {
    ledcWrite(PWM_CHANNELS[1], 0);
    ledcWrite(PWM_CHANNELS[3], 0);
    ledcWrite(PWM_CHANNELS[0], 0);
    ledcWrite(PWM_CHANNELS[2], 0);
  }
}
