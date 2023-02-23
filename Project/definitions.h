/** @file */
#include <RSLK_Pins.h>
#include <QTRSensors.h>
#include <GP2Y0A21_Sensor.h>
#include "Custom_Encoder.h"

#ifndef definitions_h
#define definitions_h

/**
 * @brief   Total number of sensors on QTR line sensor.
 */
#define LS_NUM_SENSORS 8 // number of sensors used

/**
 * @brief   Represent the left push button on the launchpad
 */
#define LP_LEFT_BTN PUSH2

/**
 * @brief   Represent the right push button on the launchpad
 */
#define LP_RIGHT_BTN PUSH1

/**
 * @brief   Total number of bump switches.
 */
#define TOTAL_BP_SW 6

/**
 * @brief   Can be used to reference the left motor in the below functions.
 */
#define LEFT_MOTOR 0

/**
 * @brief   Can be used to reference the right motor in the below functions.
 */
#define RIGHT_MOTOR 1

/**
 * @brief   Can be used to reference setting the motor function to forward.
 */
#define MOTOR_DIR_FORWARD 0

/**
 * @brief   Can be used to reference setting the motor function to backward.
 */
#define MOTOR_DIR_BACKWARD 1

/**
 * @brief   Used to specify that the robot is running on a floor lighter than the line
 */
#define DARK_LINE 0

/**
 * @brief   Used to specify that the robot is running on a floor darker than the line
 */
#define LIGHT_LINE 1

/**
 * @brief Used to specify Serial print logging levels
 */
enum logLevels
{
	/**
	 * @brief Logging level which used for targeted debugging (reduced output to only those tagged with this log level)
	 */
	SPECIAL_1,
	/**
	 * @brief Logging level which only outputs code-essential logs
	 */
	NONE,
	/**
	 * @brief Logging level which outputs streamlined information
	 */
	INFO,
	/**
	 * @brief Logging level which outputs all written serial prints
	 */
	DEBUG
};

/// \brief Performs a variety of initialization needed for the RSLK.
///
/// This function must be called before calling any other functions listed on this page. Changes from the default setupRSLK() include: removal of the encoder, motor, and button setup
///
void customSetupRSLK();
/// \brief Read distance sensor value.
///
/// \param[in] num of the distance sensor to read. Valid values are 0 - 2. Representing the 3 RSLK's sensors that can be
/// mounted on the RSLK (on top of the bump switch assembly).
/// - 0 for the left sensor.
/// - 1 for the center sensor.
/// - 5 for the right sensor.
/// \return A value from 0 - 4065.
/// - 0 represents object right infront of sensor
/// - ....
/// - 4065 represents no object detected
///
uint16_t readSharpDist(uint8_t num);

/// \brief Read line sensor values
///
/// \param[out] sensor array that stores values read from line sensor. Must pass an array with 8 elements.
/// Array index 0 represents the left most sensor. Array index 7 represents the right most sensor. @n
/// Each index will contain a value from 0 - 2500.
/// - 0 max reflection (light line)
/// - ....
/// - 2500 no reflection (dark line)
///
///
/// Read and store sensor values in the passed in array.
void readLineSensor(uint16_t *sensor);

/// \brief Update sensor's min and max values array based on current data.
///
/// \param[out] sensor is an array to be filled with line sensor values.
///
/// \param[out] calVal is an array that will be filled with the calibrated values based on the sensor,
/// sensorMin and sensorMax array. @n
/// Elements will be filled with values of 0 - 1000
/// - 0 means no line detected
/// - ...
/// - 1000 means line is detected right under sensor.
///
/// \param[in] sensorMin stores sensor's min values.
///
/// \param[in] sensorMax stores sensor's max values.
///
/// \param[in] mode determines if the line is dark or light.
/// - 0 is used when the line is darker than the floor
/// - 1 is used when the line is lighter than the floor.
///
/// Takes the current line sensor values and sets calVal to the calibrated values. Uses
/// sensorMin and sensorMax array along with mode to calibrate value. @n @n
///
/// Calibration:
/// - When the line is dark then calibration subtracts sensorMax values from the sensor value read.
/// - When the line is light then calibration subtracts sensorMin values from the sensor value read.
/// Then the value is subtracted from 1000 to provide a consistent scale.
void readCalLineSensor(uint16_t *sensor,
					   uint16_t *calVal,
					   uint16_t *sensorMin,
					   uint16_t *sensorMax,
					   uint8_t mode);

/// \brief Get line position (modified version of getLinePosition() from SimpleRSLK)
/// \param[in] calVal is an array that is filled with the line sensor calibrated values.
///
/// \param[in] mode determines if the line is dark or light.
/// - 0 is used when the line is darker than the floor
/// - 1 is used when the line is lighter than the floor.
///
/// \return value between 0 - 7000.
///  - 9999 no line detected
///  - ...
///  - 0000 line is directly on the left most sensor
///  - ...
///  - 3500 line directly over two middle sensors.
///  - ...
///  - 7000 is under right most line sensor
///
///  Using calibrated line sensor value this function provides a numerical value indicating
///  where the robot is detecting the line. This function can be overridden.
uint32_t getLinePosition(uint16_t *calVal, uint8_t mode);

/// \brief Provide default values for the sensor's Min and Max arrays.
///
/// \param[out] sensorMin stores sensor's min values. Must pass an array with 8 elements.
///  All elements will by default be given a large value.
///
/// \param[out] sensorMax stores sensor's max values. Must pass an array with 8 elements.
///  All elements will by default be given a value of 0.
///
///  Initializes arrays to be used to store line sensor's min and max values.
void clearMinMax(uint16_t *sensorMin, uint16_t *sensorMax);

/// \brief Update line sensor's min and max values array based on current data.
///
/// \param[in] sensor is an array filled with line sensor values previously filled by readLineSensor.
///
/// \param[out] sensorMin stores sensor's min values.
///
/// \param[out] sensorMax stores sensor's max values.
///
///  Take the current line sensor values and update min and max values. This function along with the
///  min and max arrays are useful when performing calibration.
void setSensorMinMax(uint16_t *sensor, uint16_t *sensorMin, uint16_t *sensorMax);

/// \brief Logging using Serial print and println with adaptable log levels.
///
/// \param[in] s the value to print.
///
/// \param[in] scriptLevel the current logging level set by the script.
///
/// \param[in] printLevel the logging level for this print statement.
///
/// \param[in] ln whether to use println or print (true will use println)
///
void customLog(const String &s, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false);

/// \brief Logging using Serial print and println with adaptable log levels.
///
/// \param[in] str the value to print.
///
/// \param[in] scriptLevel the current logging level set by the script.
///
/// \param[in] printLevel the logging level for this print statement.
///
/// \param[in] ln whether to use println or print (true will use println)
///
void customLog(const char str[], logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false);

/// \brief Logging using Serial print and println with adaptable log levels.
///
/// \param[in] c the value to print.
///
/// \param[in] scriptLevel the current logging level set by the script.
///
/// \param[in] printLevel the logging level for this print statement.
///
/// \param[in] ln whether to use println or print (true will use println)
///
void customLog(char c, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false);

/// \brief Logging using Serial print and println with adaptable log levels.
///
/// \param[in] b the value to print.
///
/// \param[in] base the base of the value to print.
///
/// \param[in] scriptLevel the current logging level set by the script.
///
/// \param[in] printLevel the logging level for this print statement.
///
/// \param[in] ln whether to use println or print (true will use println)
///
void customLog(unsigned char b, int base, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false);

/// \brief Logging using Serial print and println with adaptable log levels.
///
/// \param[in] n the value to print.
///
/// \param[in] base the base of the value to print.
///
/// \param[in] scriptLevel the current logging level set by the script.
///
/// \param[in] printLevel the logging level for this print statement.
///
/// \param[in] ln whether to use println or print (true will use println)
///
void customLog(int n, int base, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false);

/// \brief Logging using Serial print and println with adaptable log levels.
///
/// \param[in] n the value to print.
///
/// \param[in] base the base of the value to print.
///
/// \param[in] scriptLevel the current logging level set by the script.
///
/// \param[in] printLevel the logging level for this print statement.
///
/// \param[in] ln whether to use println or print (true will use println)
///
void customLog(unsigned int n, int base, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false);

/// \brief Logging using Serial print and println with adaptable log levels.
///
/// \param[in] n the value to print.
///
/// \param[in] base the base of the value to print.
///
/// \param[in] scriptLevel the current logging level set by the script.
///
/// \param[in] printLevel the logging level for this print statement.
///
/// \param[in] ln whether to use println or print (true will use println)
///
void customLog(long n, int base, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false);

/// \brief Logging using Serial print and println with adaptable log levels.
///
/// \param[in] n the value to print.
///
/// \param[in] base the base of the value to print.
///
/// \param[in] scriptLevel the current logging level set by the script.
///
/// \param[in] printLevel the logging level for this print statement.
///
/// \param[in] ln whether to use println or print (true will use println)
///
void customLog(unsigned long n, int base, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false);

/// \brief Logging using Serial print and println with adaptable log levels.
///
/// \param[in] n the value to print.
///
/// \param[in] digits the number of digits to print from n.
///
/// \param[in] scriptLevel the current logging level set by the script.
///
/// \param[in] printLevel the logging level for this print statement.
///
/// \param[in] ln whether to use println or print (true will use println)
///
void customLog(float n, int digits, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false);

/// \brief Logging using Serial print and println with adaptable log levels.
///
/// \param[in] x the value to print.
///
/// \param[in] scriptLevel the current logging level set by the script.
///
/// \param[in] printLevel the logging level for this print statement.
///
/// \param[in] ln whether to use println or print (true will use println)
///
void customLog(const Printable& x, logLevels scriptLevel = DEBUG, logLevels printLevel = INFO, bool ln = false);
#endif