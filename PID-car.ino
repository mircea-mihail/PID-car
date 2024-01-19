#include <QTRSensors.h>

const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;

// increase kp’s value and see what happens
float kp = 10;
float ki = 0.001;
float kd = 3;

int error = 0;
int lastError = 0;

const int maxSpeed = 255;
const int minSpeed = -255;
const int baseSpeed = 200;

QTRSensors qtr;
const int sensorCount = 6;
uint16_t sensorValues[sensorCount];
int sensors[sensorCount] = { 0, 0, 0, 0, 0, 0 };

#define INTEGRAL_SIZE 100
int recordedErrors[INTEGRAL_SIZE];
int recErrIdx = 0;
void addRecordedError(int p_error)
{
    recordedErrors[recErrIdx] = p_error;

    recErrIdx ++;
    // loop around, rolling average
    if(recErrIdx >= INTEGRAL_SIZE)
    {
        recErrIdx = 0;
    }
}
void initRecordedError()
{
    for(int i = 0; i < INTEGRAL_SIZE; i++)
    {
        recordedErrors[i] = 0;
    }
}
int getIntegral()
{
    int integral = 0;
    for(int i = 0; i < INTEGRAL_SIZE; i++)
    {
        integral += recordedErrors[i];
    }

    return integral/INTEGRAL_SIZE;
}

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

void switchTurnMotorSpeeds(int &turnMotorSpeed1, int& turnMotorSpeed2)
{
    if(turnMotorSpeed1 == -TURN_MOTOR_SPEED)
    {
        turnMotorSpeed1 = TURN_MOTOR_SPEED;
        turnMotorSpeed2 = -TURN_MOTOR_SPEED;
    }
    else
    {
        turnMotorSpeed2 = TURN_MOTOR_SPEED;
        turnMotorSpeed1 = -TURN_MOTOR_SPEED;   
    }
}

void smartCalibrateSensor()
{
    Serial.print("in smart calibration\n");
    digitalWrite(LED_BUILTIN, HIGH);	// turn on Arduino's LED to indicate we are in calibration mode
	// calibrate the sensor. For maximum grade the line follower should do the movement itself, without
	// human interaction.digitalWrite(LED_BUILTIN, HIGH);	// turn on Arduino's LED to indicate we are in calibration mode
	// calibrate the sensor. For maximum grade the line follower should do the movement itself, without
	// human interaction.
	
    bool finishedCalibration = false;
    int currentTurnIdx = 0;
    int turnMotorSpeed1, turnMotorSpeed2;

	bool turningDirection = RIGHT;
    turnMotorSpeed1 = TURN_MOTOR_SPEED;
    turnMotorSpeed2 = -TURN_MOTOR_SPEED;
	qtr.calibrate();

	int error = 0;
	setMotorSpeed(turnMotorSpeed1, turnMotorSpeed2);

    while(!finishedCalibration)
    {
		qtr.calibrate();
		error = map(qtr.readLineBlack(sensorValues), MIN_QTR_SENSITIVITY, MAX_QTR_SENSITIVITY, MIN_CALIBRATION_SENSITIVITY, MAX_CALIBRATION_SENSITIVITY);
		
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
			Serial.print("Went Left. Turning Right");
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
			Serial.print("Went Right. Turning Left");
        }
    }

	digitalWrite(LED_BUILTIN, LOW);
}

void setup()
{
	// pinMode setup
	pinMode(m11Pin, OUTPUT);
	pinMode(m12Pin, OUTPUT);
	pinMode(m21Pin, OUTPUT);
	pinMode(m22Pin, OUTPUT);
	pinMode(m1Enable, OUTPUT);
	pinMode(m2Enable, OUTPUT);
	qtr.setTypeAnalog();
	qtr.setSensorPins((const uint8_t[])
	{
		A0, A1, A2, A3, A4, A5
	}, sensorCount);

	pinMode(LED_BUILTIN, OUTPUT);
	
    smartCalibrateSensor();
    
    Serial.begin(115200);
    initRecordedError();
}

#define MIN_ERROR -50
#define MAX_ERROR 50

// calculate PID value based on error, kp, kd, ki
int pidControl(float kp, float ki, float kd, int &lastError, int error)
{
    int p = error;
	int d = error - lastError;
    
    addRecordedError(error);
	int i = getIntegral();
	if(-1 <= error && error <= 1) {
        i = 0;
    }

    lastError = error;

	return kp *p + ki *i + kd * d;
}

void constrainMotorSpeeds(int &m1Speed, int &m2Speed, int motorSpeed, int error)
{
	// a bit counter intuitive because of the signs
	// basically in the first if, you substract the error from m1Speed (you add the negative)
	// in the 2nd if you add the error to m2Speed (you substract the negative)
	// it's just the way the values of the sensors and/or motors lined up
	if (error < 0)
	{
		m1Speed += motorSpeed;
	}
	else if (error > 0)
	{
		m2Speed -= motorSpeed;
	}

	// make sure it doesn't go past limits. You can use -255 instead of 0 if calibrated programmed
	// properly.
	// making sure we don't go out of bounds
	// maybe the lower bound should be negative, instead of 0? This of what happens when making a
	// steep turn
	m1Speed = constrain(m1Speed, 0, maxSpeed);
	m2Speed = constrain(m2Speed, 0, maxSpeed);
}

void loop()
{
    int error = map(qtr.readLineBlack(sensorValues), MIN_QTR_SENSITIVITY, MAX_QTR_SENSITIVITY, MIN_ERROR, MAX_ERROR);
	int motorSpeed = pidControl(kp, ki, kd ,lastError, error);	// = error in this case
	int m1Speed = baseSpeed;
	int m2Speed = baseSpeed;
	Serial.println(motorSpeed);
	constrainMotorSpeeds(m1Speed, m2Speed, motorSpeed, error);

	setMotorSpeed(m1Speed, m2Speed);
   
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
}

// each arguments takes values between -255 and 255. The negative values represent the motor speed
// in reverse.
void setMotorSpeed(int motor1Speed, int motor2Speed)
{
	// remove comment if any of the motors are going in reverse
	// motor1Speed = -motor1Speed;
	// motor2Speed = -motor2Speed;
	if (motor1Speed == 0)
	{
		digitalWrite(m11Pin, LOW);
		digitalWrite(m12Pin, LOW);
		analogWrite(m1Enable, motor1Speed);
	}
	else
	{
		if (motor1Speed > 0)
		{
			digitalWrite(m11Pin, HIGH);
			digitalWrite(m12Pin, LOW);
			analogWrite(m1Enable, motor1Speed);
		}

		if (motor1Speed < 0)
		{
			digitalWrite(m11Pin, LOW);
			digitalWrite(m12Pin, HIGH);
			analogWrite(m1Enable, -motor1Speed);
		}
	}

	if (motor2Speed == 0)
	{
		digitalWrite(m21Pin, LOW);
		digitalWrite(m22Pin, LOW);
		analogWrite(m2Enable, motor2Speed);
	}
	else
	{
		if (motor2Speed > 0)
		{
			digitalWrite(m21Pin, HIGH);
			digitalWrite(m22Pin, LOW);
			analogWrite(m2Enable, motor2Speed);
		}

		if (motor2Speed < 0)
		{
			digitalWrite(m21Pin, LOW);
			digitalWrite(m22Pin, HIGH);
			analogWrite(m2Enable, -motor2Speed);
		}
	}
}