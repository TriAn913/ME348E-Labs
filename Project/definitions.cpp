#include "definitions.h"

GP2Y0A21_Sensor dst_sensor[3];
uint16_t calMin[LS_NUM_SENSORS], calMax[LS_NUM_SENSORS];

QTRSensors qtr;

void customSetupRSLK()
{ // removed encoder since using custom one
	dst_sensor[0].begin(SHRP_DIST_L_PIN, INPUT_PULLDOWN);
	dst_sensor[1].begin(SHRP_DIST_C_PIN, INPUT_PULLDOWN);
	dst_sensor[2].begin(SHRP_DIST_R_PIN, INPUT_PULLDOWN);

	qtr.setTypeRC();
	qtr.setSensorPins((const uint8_t[]){QTR_7, QTR_6, QTR_5, QTR_4, QTR_3, QTR_2, QTR_1, QTR_0}, LS_NUM_SENSORS);
	qtr.setEmitterPins(QTR_EMITTER_PIN_ODD, QTR_EMITTER_PIN_EVEN);

	for (uint8_t x = 0; x < LS_NUM_SENSORS; x++)
	{
		calMin[x] = 5000;
		calMax[x] = 0;
	}
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
		if (value > 50)
		{
			avg += (uint32_t)value * (i * 1000);
			sum += value;
		}
	}

	_lastPosition = avg / sum;
	return _lastPosition;
}

void clearMinMax(uint16_t *sensorMin, uint16_t *sensorMax)
{
	for (int x = 0; x < LS_NUM_SENSORS; x++)
	{
		sensorMin[x] = 9000;
		sensorMax[x] = 0;
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