// encoder counts per revolution
#define countsPerRev (uint16_t)48
// max speed (experimental) in Rev/s
#define maxSpeed 66.7f
// hard-coded delay time
#define delayTime (uint8_t)20

// Pin Definitions
#define PWMoutp (uint8_t)19
#define PWMoutn (uint8_t)34
#define PWMspeedPin (uint8_t)38
#define potentiometerPWMinput (uint8_t)33
#define encoder0PinA (uint8_t)5
#define encoder1PinA (uint8_t)36
#define encoder0PinB (uint8_t)6
#define encoder1PinB (uint8_t)37

// Variable Definitions
volatile signed int encoder0Pos = 0;
signed int encoderPosLast = 0;
uint16_t potPWMval = 0;
uint8_t motorPWMval = 0;

// PID constants (Use the Ziegler Nicholas Tuning Method as a first guess)
double kp = 0;
double ki = 0;
double kd = 0;

// PID Variables
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastSpeedError;
double motorSpeed, newMotorSpeed, setPoint;
double cumError, rateError;

void setup()
{
  // Don't Change
  Serial.begin(115200);
  pinMode(potentiometerPWMinput, INPUT_PULLUP);
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  pinMode(PWMoutp, OUTPUT);
  pinMode(PWMoutn, OUTPUT);
  pinMode(PWMspeedPin, OUTPUT);
  attachInterrupt(encoder0PinA, doEncoderA, RISING); // Interrupt is fired whenever button is pressed
  attachInterrupt(encoder1PinA, doEncoderA, FALLING);
  attachInterrupt(encoder0PinB, doEncoderB, RISING);
  attachInterrupt(encoder1PinB, doEncoderB, FALLING);

  // Pre-set the direction of the motors
  digitalWrite(PWMoutp, HIGH);
  digitalWrite(PWMoutn, LOW);

}

void loop()
{
  // POTENTIOMETER CONTROL

  potPWMval = analogRead(potentiometerPWMinput);

  switch (0)
  {
    case 1: // directly set motor PWM (motor power) based on potentiometer value
    {
      motorPWMval = map(potPWMval, 0, 1023, -255, 255);
      break;
    }
    case 2: // set desired speed (PID Control) using potentiometer value
    {
      setPoint = map(potPWMval, 0, 1023, -maxSpeed, maxSpeed);
      motorPWMval = map(computePID(motorSpeed), -maxSpeed, maxSpeed, -255, 255);
      break;
    }
    case 3: // set desired speed (PID Control) using a fixed value
    {
      setPoint = 30;
      motorPWMval = map(computePID(motorSpeed), -maxSpeed, maxSpeed, -255, 255);
      break;
    }
    default: // set motor pwm to a fixed value
    {
      motorPWMval = 100; // out of 255
    }
  }

  motorSpeed = ((encoder0Pos - encoderPosLast) / countsPerRev) * 2 * PI / delayTime;

  // Serial.println(potPWMval);
  Serial.println(motorSpeed);

  if (motorPWMval < 0)
  {
    digitalWrite(PWMoutp, HIGH);
    digitalWrite(PWMoutn, LOW);
  }
  else
  {
    digitalWrite(PWMoutp, LOW);
    digitalWrite(PWMoutn, HIGH);
  }

  analogWrite(PWMspeedPin, abs(motorPWMval));

  encoderPosLast = encoder0Pos;
  delay(delayTime); // This is a delay to time the control loop. In the future, this should be a non-blocking version.
}

// **** DONT CHANGE THE FUNCTIONS BELOW ****
//================
// READ ENCODER A
//================
void doEncoderA()
{
  if (digitalRead(encoder0PinA) == HIGH)
  { // found a low-to-high on channel A
    if (digitalRead(encoder0PinB) == LOW)
    {                                // check channel B to see which way
                                     // encoder is turning
      encoder0Pos = encoder0Pos - 1; // CCW
    }
    else
    {
      encoder0Pos = encoder0Pos + 1; // CW
    }
  }
  else // found a high-to-low on channel A
  {
    if (digitalRead(encoder0PinB) == LOW)
    {                                // check channel B to see which way
                                     // encoder is turning
      encoder0Pos = encoder0Pos + 1; // CW
    }
    else
    {
      encoder0Pos = encoder0Pos - 1; // CCW
    }
  }
}
//================
// READ ENCODER B
//================
void doEncoderB()
{
  if (digitalRead(encoder0PinB) == HIGH)
  { // found a low-to-high on channel A
    if (digitalRead(encoder0PinA) == LOW)
    {                                // check channel B to see which way
                                     // encoder is turning
      encoder0Pos = encoder0Pos + 1; // CCW
    }
    else
    {
      encoder0Pos = encoder0Pos - 1; // CW
    }
  }
  else // found a high-to-low on channel A
  {
    if (digitalRead(encoder0PinA) == LOW)
    {                                // check channel B to see which way
                                     // encoder is turning
      encoder0Pos = encoder0Pos - 1; // CW
    }
    else
    {
      encoder0Pos = encoder0Pos + 1; // CCW
    }
  }
}
//================
// PID CONTROL FUNCTION (use this!)
//================
double computePID(double inp)
{
  currentTime = millis();                             // get current time
  elapsedTime = (double)(currentTime - previousTime); // compute time elapsed from previous computation

  error = setPoint - inp;                             // determine error
  cumError += error * elapsedTime;                    // compute integral
  rateError = (error - lastSpeedError) / elapsedTime; // compute derivative

  double out = kp * error + ki * cumError + kd * rateError; // PID output

  lastSpeedError = error;     // remember current error
  previousTime = currentTime; // remember current time
  // Serial.println(error);
  return out; // have function return the PID output
}
