#include <QTRSensors.h>

// pins controlling the motors
const int g_m11Pin = 7;
const int g_m12Pin = 6;
const int g_m21Pin = 5;
const int g_m22Pin = 4;
const int g_m1Enable = 11;
const int g_m2Enable = 10;

// motor constraints
const int g_maxSpeed = 255;
const int g_minSpeed = -255;
#define MAX_BASE_SPEED 180

#define MIN_BASE_SPEED_PERCENT 60
#define MAX_BASE_SPEED_PERCENT 100
#define MAX_DERIVATIVE_TWEAK_VAL 10

int g_baseSpeed = MAX_BASE_SPEED;

// PID controller constants
float g_kp = 3.8;
float g_ki = 0;
float g_kd = 4.5;
int g_lastError = 0;

// sensor related
QTRSensors g_qtr;
const int g_sensorCount = 6;
uint16_t g_sensorValues[g_sensorCount];

// integrative functions related
#define INTEGRAL_SIZE 100
int g_recordedErrors[INTEGRAL_SIZE];
int g_recordedErrIdx = 0;

// updates the integrative rolling window array
void addRecordedError(int p_error)
{
    g_recordedErrors[g_recordedErrIdx] = p_error;

    g_recordedErrIdx ++;
    // loop around, rolling average
    if(g_recordedErrIdx >= INTEGRAL_SIZE)
    {
        g_recordedErrIdx = 0;
    }
}

// sets the integrative rolling window error array to 0
void initRecordedError()
{
    for(int i = 0; i < INTEGRAL_SIZE; i++)
    {
        g_recordedErrors[i] = 0;
    }
}

// returns the sum of the rolling window for the Integrative control
int getIntegral()
{
    int integral = 0;
    for(int i = 0; i < INTEGRAL_SIZE; i++)
    {
        integral += g_recordedErrors[i];
    }

    return integral/INTEGRAL_SIZE;
}

// calibration defines 
#define CALIBRATION_TURN_TIME 600
#define NUMBER_OF_TURNS 6
#define TURN_MOTOR_SPEED 120

#define MAX_CALIBRATION_SENSITIVITY 100
#define MIN_CALIBRATION_SENSITIVITY -100

#define LEFT 1
#define RIGHT 0

#define MAX_CALIBRATION_ERROR 70
#define MIN_CALIBRATION_ERROR -MAX_CALIBRATION_ERROR

#define MAX_QTR_SENSITIVITY 5000
#define MIN_QTR_SENSITIVITY 0

// basically makes the two wheels spin in the other direction for the self calibration
void switchTurnMotorSpeeds(int &p_turnMotorSpeed1, int& p_turnMotorSpeed2)
{
    if(p_turnMotorSpeed1 == -TURN_MOTOR_SPEED)
    {
        p_turnMotorSpeed1 = TURN_MOTOR_SPEED;
        p_turnMotorSpeed2 = -TURN_MOTOR_SPEED;
    }
    else
    {
        p_turnMotorSpeed2 = TURN_MOTOR_SPEED;
        p_turnMotorSpeed1 = -TURN_MOTOR_SPEED;   
    }
}

// self calibration done using data from the sensor.
// at first not perfect but as it does the calibration it gets better
void smartCalibrateSensor()
{
    Serial.print("in smart calibration\n");
    digitalWrite(LED_BUILTIN, HIGH);	// turn on Arduino's LED to indicate we are in calibration mode

    bool finishedCalibration = false;
    int currentTurnIdx = 0;
    int turnMotorSpeed1, turnMotorSpeed2;

	bool turningDirection = RIGHT;
    turnMotorSpeed1 = TURN_MOTOR_SPEED;
    turnMotorSpeed2 = -TURN_MOTOR_SPEED;
	g_qtr.calibrate();

	int error = 0;
	setMotorSpeed(turnMotorSpeed1, turnMotorSpeed2);

    while(!finishedCalibration)
    {
		g_qtr.calibrate();
		error = map(g_qtr.readLineBlack(g_sensorValues), MIN_QTR_SENSITIVITY, MAX_QTR_SENSITIVITY, MIN_CALIBRATION_SENSITIVITY, MAX_CALIBRATION_SENSITIVITY);
		
        if(error > MAX_CALIBRATION_ERROR && turningDirection == LEFT)
        {
            if(currentTurnIdx > NUMBER_OF_TURNS)
            {
                break; 
            }

            currentTurnIdx ++;
            switchTurnMotorSpeeds(turnMotorSpeed1, turnMotorSpeed2);

            setMotorSpeed(turnMotorSpeed1, turnMotorSpeed2);
			turningDirection = RIGHT;
        }
		if(error < MIN_CALIBRATION_ERROR && turningDirection == RIGHT)
        {
            if(currentTurnIdx > NUMBER_OF_TURNS)
            {
                break; 
            }

            currentTurnIdx ++;
            switchTurnMotorSpeeds(turnMotorSpeed1, turnMotorSpeed2);

            setMotorSpeed(turnMotorSpeed1, turnMotorSpeed2);
			turningDirection = LEFT;
        }
    }

	digitalWrite(LED_BUILTIN, LOW);
}

// how it was before for refference if not working
// g_qtr.setSensorPins((const uint8_t[])
// 	{
// 		A0, A1, A2, A3, A4, A5
// 	}, g_sensorCount);

void tweakBaseSpeed(int p_derivative)
{
	p_derivative = abs(p_derivative);

	if(p_derivative > 10)
	{
		p_derivative = 10;
	}

	float basePercentage = map(p_derivative, 0, MAX_DERIVATIVE_TWEAK_VAL, MAX_BASE_SPEED_PERCENT, MIN_BASE_SPEED_PERCENT);
	g_baseSpeed = basePercentage * MAX_BASE_SPEED / 100;
}

const uint8_t g_sensorPins[] = { A0, A1, A2, A3, A4, A5 };

void setup()
{
    Serial.begin(115200);

	// pinMode setup
	pinMode(g_m11Pin, OUTPUT);
	pinMode(g_m12Pin, OUTPUT);
	pinMode(g_m21Pin, OUTPUT);
	pinMode(g_m22Pin, OUTPUT);
	pinMode(g_m1Enable, OUTPUT);
	pinMode(g_m2Enable, OUTPUT);
	pinMode(LED_BUILTIN, OUTPUT);

	// sensor setup and calibration
	g_qtr.setTypeAnalog();
	g_qtr.setSensorPins(g_sensorPins, g_sensorCount);
    smartCalibrateSensor();

	// integrative array init
    initRecordedError();
}

// error range for the sensor 
#define MIN_ERROR -50
#define MAX_ERROR 50

#define DERIVATIVE_ERROR_TRACEBACK 10

// calculate PID value based on error, kp, kd, ki
int pidControl(float p_kp, float p_ki, float p_kd, int &p_lastError, int p_error)
{
    int p = p_error;
	
	int lastErrorIdx = g_recordedErrIdx - DERIVATIVE_ERROR_TRACEBACK;
	if(lastErrorIdx < 0)
	{
		lastErrorIdx = INTEGRAL_SIZE + lastErrorIdx; 
	}
	int d = p_error - g_recordedErrors[lastErrorIdx];
    
    addRecordedError(p_error);
	int i = getIntegral();
	if(-1 <= p_error && p_error <= 1) {
        i = 0;
    }
    p_lastError = p_error;

	tweakBaseSpeed(d);
	return p_kp *p + p_ki *i + p_kd * d;
}

void constrainMotorSpeeds(int &p_m1Speed, int &p_m2Speed, int p_speedCorrection, int p_error)
{
	// a bit counter intuitive because of the signs
	// basically in the first if, you substract the error from m1Speed (you add the negative)
	// in the 2nd if you add the error to m2Speed (you substract the negative)
	// it's just the way the values of the sensors and/or motors lined up
	if (p_error < 0)
	{
		p_m1Speed += p_speedCorrection;
	}
	else if (p_error > 0)
	{
		p_m2Speed -= p_speedCorrection;
	}

	// make sure it doesn't go past limits. You can use -255 instead of 0 if calibrated programmed
	// properly.
	// making sure we don't go out of bounds
	// maybe the lower bound should be negative, instead of 0? This of what happens when making a
	// steep turn
	p_m1Speed = constrain(p_m1Speed, 0, g_maxSpeed);
	p_m2Speed = constrain(p_m2Speed, 0, g_maxSpeed);

	// WARNING! SPEED CORRECTION OFTEN GOES BEYOND BASE SPEED!!
}

void loop()
{
    int error = map(g_qtr.readLineBlack(g_sensorValues), MIN_QTR_SENSITIVITY, MAX_QTR_SENSITIVITY, MIN_ERROR, MAX_ERROR);
	int speedCorrection = pidControl(g_kp, g_ki, g_kd ,g_lastError, error);
	int m1Speed = g_baseSpeed;
	Serial.println(g_baseSpeed);
	int m2Speed = g_baseSpeed;

	constrainMotorSpeeds(m1Speed, m2Speed, speedCorrection, error);
	setMotorSpeed(m1Speed, m2Speed);
}

// each arguments takes values between -255 and 255. The negative values represent the motor speed
// in reverse.
void setMotorSpeed(int p_motor1Speed, int p_motor2Speed)
{
	// remove comment if any of the motors are going in reverse
	// motor1Speed = -motor1Speed;
	// motor2Speed = -motor2Speed;
	if (p_motor1Speed == 0)
	{
		digitalWrite(g_m11Pin, LOW);
		digitalWrite(g_m12Pin, LOW);
		analogWrite(g_m1Enable, p_motor1Speed);
	}
	else
	{
		if (p_motor1Speed > 0)
		{
			digitalWrite(g_m11Pin, HIGH);
			digitalWrite(g_m12Pin, LOW);
			analogWrite(g_m1Enable, p_motor1Speed);
		}

		if (p_motor1Speed < 0)
		{
			digitalWrite(g_m11Pin, LOW);
			digitalWrite(g_m12Pin, HIGH);
			analogWrite(g_m1Enable, -p_motor1Speed);
		}
	}

	if (p_motor2Speed == 0)
	{
		digitalWrite(g_m21Pin, LOW);
		digitalWrite(g_m22Pin, LOW);
		analogWrite(g_m2Enable, p_motor2Speed);
	}
	else
	{
		if (p_motor2Speed > 0)
		{
			digitalWrite(g_m21Pin, HIGH);
			digitalWrite(g_m22Pin, LOW);
			analogWrite(g_m2Enable, p_motor2Speed);
		}

		if (p_motor2Speed < 0)
		{
			digitalWrite(g_m21Pin, LOW);
			digitalWrite(g_m22Pin, HIGH);
			analogWrite(g_m2Enable, -p_motor2Speed);
		}
	}
}

// debugging once used, here for faster further debugs
// DEBUGGING
// if(m1Speed > m2Speed)
// {
//     Serial.print("go left\n");
// }
// else if (m1Speed != m2Speed)
// {
//     Serial.print("go right\n");
// }
// Serial.print("Error: ");
// Serial.println(error);
// Serial.print("M1 speed: ");
// Serial.println(m1Speed);
//
// Serial.print("M2 speed: ");
// Serial.println(m2Speed);
//
// delay(250);