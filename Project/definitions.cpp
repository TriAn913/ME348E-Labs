#include "definitions.h"

GP2Y0A21_Sensor dst_sensor[3];

// line follower
QTRSensors qtr;
uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS] = {875, 858, 663, 759, 588, 797, 679, 950};
uint16_t sensorMinVal[LS_NUM_SENSORS];
uint8_t lineColor = DARK_LINE;
uint32_t linePos = 9999;
uint8_t interCounts; // number of intersections crossed
bool prevInter;      // whether was on intersection last time checked (true: was on intersection; false: was not on intersection)
bool onInter;        // is on intersection (true: on intersection; false: not on intersection)

// linefollower PID
int32_t error;
int32_t prevError;
static const uint32_t setpoint = 3500;
static const float dt = 0.1f;
float integral;
float deriv;
int16_t output;
static const float kp = 1.0f / 3.5f / 100.0f;  // error of 3500 (max) returns 10
static const float ki = 0;                     // not needed for this application (would be useful if needed constant power at the setpoint)
static const float kd = -0.2f / 3.5f / 100.0f; // used to prevent oscillations

// launchpad left button (toggle on/off)
bool onToggle = 0;          // toggle all functions on/off (off disables all motors)
bool lpButtonValue = 0;     // launchpad left button
bool lpButtonLastValue = 1; // the value of the launchpad left button value the last time checked

// encoder
Custom_Encoder encoder[2];
uint32_t leftCount; // left motor encoder count
uint32_t rightCount; // right motor encoder count

// states
enum states // operational state
{
  NONE_,
  LINE_A,
  LINE_B,
  TURN,
  STOP
};
states curr_state[1]; // [0] is the current state, [1] is the previous state

// substates
uint8_t substate; // which substate is active; 0:none, 1:init, 2:main, 3:exit, 4:finished

// turn function parameters
int turnRotations;
#define LEFT 0
#define RIGHT 1
#define NINETY_DEG_TURN 170
#define ONE_EIGHTY_DEG_TURN 340
uint8_t turnDirection; // 0:left, 1:right

void setDefaults()
{
  setDefaultsIntersection();
  setDefaultsLFPID();
  setDefaultsEncoderCnts();

  // turning
  turnRotations = 0;
  turnDirection = LEFT; // 0:left, 1:right

  // states and substates
  curr_state[0] = LINE_A;
  curr_state[1] = NONE_;
  substate = 0;
}

void setDefaultsIntersection()
{
  // line follower intersection values
  interCounts = 0;
  prevInter = true;
  onInter = false;
}

void setDefaultsLFPID()
{
  // line follower PID
  error = 0;
  prevError = 0;
  integral = 0;
  deriv = 0;
  output = 0;
}

void setDefaultsEncoderCnts()
{
  // encoder counts
  leftCount = 0;
  rightCount = 0;
  encoder[0].resetEncoderCnt();
  encoder[1].resetEncoderCnt();
}

void customSetupRSLK()
{ // removed encoder since using custom one
	dst_sensor[0].begin(SHRP_DIST_L_PIN, INPUT_PULLDOWN);
	dst_sensor[1].begin(SHRP_DIST_C_PIN, INPUT_PULLDOWN);
	dst_sensor[2].begin(SHRP_DIST_R_PIN, INPUT_PULLDOWN);

	qtr.setTypeRC();
	qtr.setSensorPins((const uint8_t[]){QTR_7, QTR_6, QTR_5, QTR_4, QTR_3, QTR_2, QTR_1, QTR_0}, LS_NUM_SENSORS);
	qtr.setEmitterPins(QTR_EMITTER_PIN_ODD, QTR_EMITTER_PIN_EVEN);

	encoder[0].begin(ENCODER_ELA_PIN, ENCODER_ELB_PIN); // left wheel encoder on the rslk kit
    encoder[1].begin(ENCODER_ERA_PIN, ENCODER_ERB_PIN); // right wheel encoder on the rslk kit
}

uint16_t readSharpDist(uint8_t num)
{
	if (num < 0 || num > 3)
		return 0;

	return dst_sensor[num].read();
}

void readLineSensor(uint16_t *sensorValues)
{
	qtr.read(sensorValues, QTRReadMode::OddEven);
}

void readCalLineSensor(uint16_t *sensorValues,
					   uint16_t *calVal,
					   uint16_t *sensorMinVal,
					   uint16_t *sensorMaxVal,
					   uint8_t mode)
{
	for (int x = 0; x < LS_NUM_SENSORS; x++)
	{
		if (mode)
		{
			calVal[x] = 0;
			if (sensorValues[x] < sensorMinVal[x])
				calVal[x] = map(sensorValues[x], 0, sensorMinVal[x], 1000, 0);
		}
		else
		{
			calVal[x] = 0;
			if (sensorValues[x] > sensorMaxVal[x])
				calVal[x] = map(sensorValues[x], sensorMaxVal[x], 2500, 0, 1000);
		}
	}
}


uint32_t getLinePosition(uint16_t *calVal, uint8_t mode)
{
  uint32_t avg = 0; // this is for the weighted total
  uint32_t sum = 0; // this is for the denominator, which is <= 64000

  uint32_t _lastPosition;
  for (uint8_t i = 0; i < LS_NUM_SENSORS; i++)
  {
    uint16_t value = calVal[i];

    // only average in values that are above a noise threshold
    if (value > 150)
    {
      avg += (uint32_t)value * (i * 1000);
      sum += value;
    }
  }

  _lastPosition = sum != 0 ? avg / sum : 9999;
  return _lastPosition;
}

void clearMinMax(uint16_t *sensorMin, uint16_t *sensorMax)
{
	for (int x = 0; x < LS_NUM_SENSORS; x++)
	{
		sensorMin[x] = 1100;
		sensorMax[x] = 900;
	}
}

void setSensorMinMax(uint16_t *sensor, uint16_t *sensorMin, uint16_t *sensorMax)
{
	for (int x = 0; x < LS_NUM_SENSORS; x++)
	{
		if (sensor[x] < sensorMin[x])
		{
			sensorMin[x] = sensor[x];
		}
		if (sensor[x] > sensorMax[x])
		{
			sensorMax[x] = sensor[x];
		}
	}
}

void customLog(const String &s, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false)
{
    if (scriptLevel >= printLevel) {
		if(ln) {
			Serial.println(s);
		}
		else{
			Serial.print(s);
		}
	}
}

void customLog(const char str[], logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false)
{
    if (scriptLevel >= printLevel) {
		if(ln) {
			Serial.println(str);
		}
		else{
			Serial.print(str);
		}
	}
}

void customLog(char c, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false)
{
    if (scriptLevel >= printLevel) {
		if(ln) {
			Serial.println(c);
		}
		else{
			Serial.print(c);
		}
	}
}

void customLog(unsigned char b, int base, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false)
{
    if (scriptLevel >= printLevel) {
		if(ln) {
			Serial.println(b);
		}
		else{
			Serial.print(b);
		}
	}
}

void customLog(int n, int base, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false)
{
    if (scriptLevel >= printLevel) {
		if(ln) {
			Serial.println(n, base);
		}
		else{
			Serial.print(n, base);
		}
	}
}

void customLog(unsigned int n, int base, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false)
{
    if (scriptLevel >= printLevel) {
		if(ln) {
			Serial.println(n, base);
		}
		else{
			Serial.print(n, base);
		}
	}
}

void customLog(long n, int base, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false)
{
    if (scriptLevel >= printLevel) {
		if(ln) {
			Serial.println(n, base);
		}
		else{
			Serial.print(n, base);
		}
	}
}

void customLog(unsigned long n, int base, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false)
{
    if (scriptLevel >= printLevel) {
		if(ln) {
			Serial.println(n, base);
		}
		else{
			Serial.print(n, base);
		}
	}
}

void customLog(float n, int digits, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false)
{
    if (scriptLevel >= printLevel) {
		if(ln) {
			Serial.println(n, digits);
		}
		else{
			Serial.print(n, digits);
		}
	}
}

void customLog(const Printable& x, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false)
{
    if (scriptLevel >= printLevel) {
		if(ln) {
			Serial.println(x);
		}
		else{
			Serial.print(x);
		}
	}
}