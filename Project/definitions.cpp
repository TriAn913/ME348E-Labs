#include "definitions.h"

GP2Y0A21_Sensor 	dst_sensor[3];
Romi_Motor_Power	motor[2];
QTRSensors 			qtr;

uint8_t Stepper_Steps = 0;

void customSetupRSLK()
{
	dst_sensor[0].begin(SHRP_DIST_L_PIN, INPUT_PULLDOWN);
	dst_sensor[1].begin(SHRP_DIST_C_PIN, INPUT_PULLDOWN);
	dst_sensor[2].begin(SHRP_DIST_R_PIN, INPUT_PULLDOWN);

	motor[0].begin(MOTOR_L_SLP_PIN, MOTOR_L_DIR_PIN, MOTOR_L_PWM_PIN);
	motor[1].begin(MOTOR_R_SLP_PIN, MOTOR_R_DIR_PIN, MOTOR_R_PWM_PIN);

	pinMode(IR_L_PIN, INPUT_PULLUP);
	pinMode(IR_C_PIN, INPUT_PULLUP);
	pinMode(IR_R_PIN, INPUT_PULLUP);

	pinMode(STEPPER_1_PIN, OUTPUT);
	pinMode(STEPPER_2_PIN, OUTPUT);
	pinMode(STEPPER_4_PIN, OUTPUT);

	setupEncoder(ENCODER_ELA_PIN, ENCODER_ELB_PIN, ENCODER_ERA_PIN, ENCODER_ERB_PIN);

	qtr.setTypeRC();
	qtr.setSensorPins((const uint8_t[]) { QTR_7, QTR_6, QTR_5, QTR_4, QTR_3, QTR_2, QTR_1, QTR_0 }, LS_NUM_SENSORS);
	qtr.setEmitterPins(QTR_EMITTER_PIN_ODD, QTR_EMITTER_PIN_EVEN);
}

uint16_t readSharpDist(uint8_t num)
{
	if (num < 0 || num > 3)
		return 0;

	return dst_sensor[num].read();
}

void readIRSensor(bool* sensorValues)
{
	sensorValues[0] = digitalRead(IR_L_PIN);
	sensorValues[1] = digitalRead(IR_C_PIN);
	sensorValues[2] = digitalRead(IR_R_PIN);
}

void readLineSensor(uint16_t* sensorValues)
{
	qtr.read(sensorValues, QTRReadMode::OddEven);
}

void readCalLineSensor(uint16_t* sensorValues,
	uint16_t* calVal,
	const uint16_t* sensorMinVal,
	const uint16_t* sensorMaxVal,
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


uint32_t getLinePosition(uint16_t* calVal, uint8_t mode)
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

void stepperFull()
{
	// Set Direction
	if (Stepper_Direction == 1)
	{
		Stepper_Steps++;
	}
	if (Stepper_Direction == 0)
	{
		Stepper_Steps--;
	}
	if (Stepper_Steps > 4)
	{
		Stepper_Steps = 0;
	}
	if (Stepper_Steps < 0)
	{
		Stepper_Steps = 4;
	}

	// Step the motor
	switch (Stepper_Steps)
	{
		case 0:
			digitalWrite(STEPPER_1_PIN, LOW);
			digitalWrite(STEPPER_2_PIN, LOW);
			digitalWrite(STEPPER_3_PIN, LOW);
			digitalWrite(STEPPER_4_PIN, HIGH);
			break;
		case 1:
			digitalWrite(STEPPER_1_PIN, LOW);
			digitalWrite(STEPPER_2_PIN, LOW);
			digitalWrite(STEPPER_3_PIN, HIGH);
			digitalWrite(STEPPER_4_PIN, LOW);
			break;
		case 2:
			digitalWrite(STEPPER_1_PIN, LOW);
			digitalWrite(STEPPER_2_PIN, HIGH);
			digitalWrite(STEPPER_3_PIN, LOW);
			digitalWrite(STEPPER_4_PIN, LOW);
			break;
		case 3:
			digitalWrite(STEPPER_1_PIN, HIGH);
			digitalWrite(STEPPER_2_PIN, LOW);
			digitalWrite(STEPPER_3_PIN, LOW);
			digitalWrite(STEPPER_4_PIN, LOW);
			break;
		default:
			digitalWrite(STEPPER_1_PIN, LOW);
			digitalWrite(STEPPER_2_PIN, LOW);
			digitalWrite(STEPPER_3_PIN, LOW);
			digitalWrite(STEPPER_4_PIN, LOW);
			break;
	}
}

void enableMotor(uint8_t motorNum) {
	if (motorNum == 0 || motorNum == 2)
	{
		motor[0].enableMotor();
	}

	if (motorNum == 1 || motorNum == 2)
	{
		motor[1].enableMotor();
	}
}


void disableMotor(uint8_t motorNum) {
	if (motorNum == 0 || motorNum == 2)
	{
		motor[0].disableMotor();
	}

	if (motorNum == 1 || motorNum == 2)
	{
		motor[1].disableMotor();
	}
}

void pauseMotor(uint8_t motorNum) {
	if (motorNum == 0 || motorNum == 2)
	{
		motor[0].pauseMotor();
	}

	if (motorNum == 1 || motorNum == 2)
	{
		motor[1].pauseMotor();
	}
}

void resumeMotor(uint8_t motorNum) {
	if (motorNum == 0 || motorNum == 2)
	{
		motor[0].resumeMotor();
	}

	if (motorNum == 1 || motorNum == 2)
	{
		motor[1].resumeMotor();
	}
}

void setMotorDirection(uint8_t motorNum, uint8_t direction) {
	if (motorNum == 0 || motorNum == 2)
	{
		if (direction == 0) {
			motor[0].directionForward();
		}
		else if (direction == 1) {
			motor[0].directionBackward();
		}
	}

	if (motorNum == 1 || motorNum == 2)
	{
		if (direction == 0) {
			motor[1].directionForward();
		}
		else if (direction == 1) {
			motor[1].directionBackward();
		}
	}
}

void setMotorSpeed(uint8_t motorNum, uint8_t speed) {
	if (motorNum == 0 || motorNum == 2)
	{
		motor[0].setSpeed(speed);
	}

	if (motorNum == 1 || motorNum == 2)
	{
		motor[1].setSpeed(speed);
	}
}

void setRawMotorSpeed(uint8_t motorNum, uint8_t speed) {
	if (motorNum == 0 || motorNum == 2)
	{
		motor[0].setRawSpeed(speed);
	}

	if (motorNum == 1 || motorNum == 2)
	{
		motor[1].setRawSpeed(speed);
	}
}

void setMotorSpeed2(uint8_t motorNum, int8_t speed) {
	if (motorNum == 0 || motorNum == 2)
	{
		if (speed >= 0)
		{
			motor[0].directionForward();
		}
		else
		{
			motor[0].directionBackward();
		}
		motor[0].setSpeed(speed);
	}

	if (motorNum == 1 || motorNum == 2)
	{
		if (speed >= 0)
		{
			motor[1].directionForward();
		}
		else
		{
			motor[1].directionBackward();
		}
		motor[1].setSpeed(speed);
	}
}

void computePID(int32_t setpoint, int32_t& prevError, int32_t input, int16_t& output, float dt, float& integral, float kp, float ki, float kd)
{
	int32_t error = setpoint - input;
	integral += (float)error * dt;
	float deriv = (float)(error - prevError) / dt;
	output = kp * (float)error + ki * integral + kd * deriv;
	prevError = error;
}

uint16_t robotDeg2WheelDeg(uint16_t turnAngle)
{
	return ((float)turnAngle * ROBOT_W2W_RADIUS) / (WHEEL_DIAMETER / 2.0f);
}

uint16_t inches2WheelDeg(uint8_t distance)
{
	return 180.0f * (float)distance / (PI * (WHEEL_DIAMETER / 2.0f));
}

void customLog(const String& s, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false)
{
	if (scriptLevel >= printLevel) {
		if (ln) {
			Serial.println(s);
		}
		else {
			Serial.print(s);
		}
	}
}

void customLog(const char str[], logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false)
{
	if (scriptLevel >= printLevel) {
		if (ln) {
			Serial.println(str);
		}
		else {
			Serial.print(str);
		}
	}
}

void customLog(char c, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false)
{
	if (scriptLevel >= printLevel) {
		if (ln) {
			Serial.println(c);
		}
		else {
			Serial.print(c);
		}
	}
}

void customLog(unsigned char b, int base, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false)
{
	if (scriptLevel >= printLevel) {
		if (ln) {
			Serial.println(b);
		}
		else {
			Serial.print(b);
		}
	}
}

void customLog(int n, int base, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false)
{
	if (scriptLevel >= printLevel) {
		if (ln) {
			Serial.println(n, base);
		}
		else {
			Serial.print(n, base);
		}
	}
}

void customLog(unsigned int n, int base, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false)
{
	if (scriptLevel >= printLevel) {
		if (ln) {
			Serial.println(n, base);
		}
		else {
			Serial.print(n, base);
		}
	}
}

void customLog(long n, int base, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false)
{
	if (scriptLevel >= printLevel) {
		if (ln) {
			Serial.println(n, base);
		}
		else {
			Serial.print(n, base);
		}
	}
}

void customLog(unsigned long n, int base, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false)
{
	if (scriptLevel >= printLevel) {
		if (ln) {
			Serial.println(n, base);
		}
		else {
			Serial.print(n, base);
		}
	}
}

void customLog(float n, int digits, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false)
{
	if (scriptLevel >= printLevel) {
		if (ln) {
			Serial.println(n, digits);
		}
		else {
			Serial.print(n, digits);
		}
	}
}

void customLog(const Printable& x, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false)
{
	if (scriptLevel >= printLevel) {
		if (ln) {
			Serial.println(x);
		}
		else {
			Serial.print(x);
		}
	}
}