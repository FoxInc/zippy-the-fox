// Included Libraries
#include <Motor.h>
#include <QTRSensors_teensy3.h>
#include <Encoder.h>
#include "config.h"

/*
	Inline Functions
*/

// -- Status LEDs --
#define GREEN_LED_LEFT_ON	digitalWrite(GREEN_LED_LEFT, HIGH)
#define GREEN_LED_LEFT_OFF	digitalWrite(GREEN_LED_LEFT, LOW)
#define GREEN_LED_RIGHT_ON	digitalWrite(GREEN_LED_RIGHT, HIGH)
#define GREEN_LED_RIGHT_OFF	digitalWrite(GREEN_LED_RIGHT, LOW)
#define RED_LED_LEFT_ON		digitalWrite(RED_LED_LEFT, HIGH)
#define RED_LED_LEFT_OFF	digitalWrite(RED_LED_LEFT, LOW)
#define RED_LED_RIGHT_ON	digitalWrite(RED_LED_RIGHT, HIGH)
#define RED_LED_RIGHT_OFF	digitalWrite(RED_LED_RIGHT, LOW)

// -- Buttons --
#define LEFT_BUTTON_ON()	digitalRead(LEFT_BUTTON)
#define RIGHT_BUTTON_ON()	digitalRead(RIGHT_BUTTON)

// -- Motor Driver --
#define ENABLE_STANDBY	digitalWrite(MOTOR_STANDBY_PIN, HIGH)
#define DISABLE_STANDBY digitalWrite(MOTOR_STANDBY_PIN, LOW)

// -- Sensors --
#define ON_LINE(sensor)		(sensor<COMPARE)
#define FOUND_LEFT()		(ON_LINE(lineSensorValues[0]) && ON_LINE(lineSensorValues[1]) && ON_LINE(lineSensorValues[2]) && ON_LINE(lineSensorValues[3]) && ON_LINE(lineSensorValues[4]))
#define FOUND_RIGHT()		(ON_LINE(lineSensorValues[7]) && ON_LINE(lineSensorValues[6]) && ON_LINE(lineSensorValues[5]) && ON_LINE(lineSensorValues[4]) && ON_LINE(lineSensorValues[3]))
#define FOUND_STRAIGHT()	((ON_LINE(lineSensorValues[2]) || ON_LINE(lineSensorValues[3])) && (ON_LINE(lineSensorValues[4]) || ON_LINE(lineSensorValues[5])))
#define FOUND_FRONT()		(ON_LINE(frontSensorValues[0]))

/* 
	Object Declarations
*/

// -- Motors --
motor motorLeft, motorRight;

// -- Encoders --
Encoder motorEncoder(11, 12);

// -- Line Sensor --
unsigned char lineSensorPins[] = { 33, 32, 31, 30, 29, 28, 27, 26 };
unsigned short lineSensorValues[LINE_NUMBER_OF_SENSORS];

QTRSensorsRC lineSensor(lineSensorPins, LINE_NUMBER_OF_SENSORS, TIMEOUT, LINE_EMITTER_PIN);

// -- Front Sensor --
unsigned char frontSensorPins[] = { 24 };
unsigned short frontSensorValues[FRONT_NUMBER_OF_SENSORS];

QTRSensorsRC frontSensor(frontSensorPins, FRONT_NUMBER_OF_SENSORS, TIMEOUT);

/*
	Variables
*/

// -- PID --
float position = 0, proportional = 0, derivative = 0, integral = 0, lastProportional = 0;
float control = 0;

// -- Path Record --
char path[30], simplifiedPath[30];
unsigned int pathCounter = 0;
unsigned int stringCounter = 0;
unsigned char foundLeft = 0, foundStraight = 0, foundRight = 0;

// -- Robot Status --
char robotState = IDLE;

// -- Encoder --
long entryTurnDistance[30], exitTurnDistance[30], nextTurnDistance = -999;

// decides which direction should the root turn
char selectTurn(unsigned char _foundLeft, unsigned char _foundRight, unsigned char _foundStraight)
{
	if (_foundLeft)
		return 'L';
	else if (_foundRight)
		return 'R';
	else if (_foundStraight)
		return 'S';
}

void turn(char _direction, unsigned int _speed, unsigned short _delayTime, unsigned char _automatic = 0)
{
	if (_automatic)
	{

	}

	else
	{
		switch (_direction)
		{
		case 'L':
			GREEN_LED_LEFT_ON;
			GREEN_LED_RIGHT_OFF;
			motorLeft.write(-_speed);
			motorRight.write(_speed);
			delay(_delayTime);
			GREEN_LED_LEFT_OFF;
			GREEN_LED_RIGHT_OFF;
			break;

		case 'R':
			GREEN_LED_RIGHT_ON;
			GREEN_LED_LEFT_OFF;
			motorLeft.write(_speed);
			motorRight.write(-_speed);
			delay(_delayTime);
			GREEN_LED_LEFT_OFF;
			GREEN_LED_RIGHT_OFF;
			break;

		case 'S':
			motorLeft.write(_speed);
			motorRight.write(_speed);
			delay(_delayTime);
			break;
		}
	}
}

unsigned int readLineSensor()
{
	if (WHITE_ON_BLACK)
		return lineSensor.readLine(lineSensorValues, QTR_EMITTERS_ON, true);

	else
		return lineSensor.readLine(lineSensorValues);
}

unsigned int readFrontSensor()
{
	if (WHITE_ON_BLACK)
		return frontSensor.readLine(frontSensorValues, QTR_EMITTERS_ON, true);

	else
		return frontSensor.readLine(frontSensorValues);
}

void setup()
{
	analogWriteResolution(12);

	if (DEBUG_CHANNEL == 1 || DEBUG_CHANNEL == 3)
		Serial.begin(115200);

	if (DEBUG_CHANNEL == 2 || DEBUG_CHANNEL == 3)
		BLUETOOTH.begin(9600);

	// pinMode Declarations
	pinMode(MOTOR_STANDBY_PIN, OUTPUT);

	pinMode(GREEN_LED_LEFT, OUTPUT);
	pinMode(GREEN_LED_RIGHT, OUTPUT);
	pinMode(RED_LED_LEFT, OUTPUT);
	pinMode(RED_LED_RIGHT, OUTPUT);

	pinMode(LEFT_BUTTON, INPUT);
	pinMode(RIGHT_BUTTON, INPUT);

	attachInterrupt(LEFT_BUTTON, leftButtonControl, RISING);
	attachInterrupt(RIGHT_BUTTON, rightButtonControl, RISING);
	
	// motor initialisation
	motorLeft.setPins(LEFT_MOTOR_A, LEFT_MOTOR_B, LEFT_MOTOR_PWM);
	motorRight.setPins(RIGHT_MOTOR_A, RIGHT_MOTOR_B, RIGHT_MOTOR_PWM);

	motorLeft.setMaxSpeed(SPEED_MAX);
	motorRight.setMaxSpeed(SPEED_MAX);

	motorLeft.initialise();
	motorRight.initialise();

	ENABLE_STANDBY;

	initialiseBot();
	
	robotState = WORKING;
}

void leftButtonControl()
{
	unsigned int counter = 0;
	while (LEFT_BUTTON_ON())
	{
		delay(10);
		counter++;
	}

	if (counter > 5)
	{
		initialiseBot();
	}
}

void rightButtonControl()
{
	int counter = 0;

	while (RIGHT_BUTTON_ON())
	{
		delay(10);
		counter++;
	}

	if (counter > 5)
	{
		if (robotState == IDLE)
		{
			ENABLE_STANDBY;
			robotState = WORKING;
		}

		else if (robotState == WORKING)
		{
			DISABLE_STANDBY;
			robotState = IDLE;
		}
	}	
}

void initialiseBot()
{
	RED_LED_LEFT_ON;
	RED_LED_RIGHT_ON;

	// Calibrate sensors 
	for (unsigned int i = 0; i < 67; i++)
	{
		if (i == 0 || i == 50)
			turn('L', SPEED_CALIBRATE, 10);
		if (i == 18)
			turn('R', SPEED_CALIBRATE, 10);

		frontSensor.calibrate();
		lineSensor.calibrate();
	}

	RED_LED_LEFT_OFF;
	RED_LED_RIGHT_OFF;
}

void showSensorValues()
{
	if (DEBUG_CHANNEL == 1 || DEBUG_CHANNEL == 3)
	{
		Serial.print("Front : ");
		Serial.print(frontSensorValues[0]);
		Serial.print(" ");

		for (unsigned char i = 0; i < LINE_NUMBER_OF_SENSORS; i++)
		{
			Serial.print(lineSensorValues[i]);
			Serial.print(' ');
		}

		Serial.println();
	}
	
	if (DEBUG_CHANNEL == 2 || DEBUG_CHANNEL == 3)
	{
		BLUETOOTH.print("Front : ");
		BLUETOOTH.print(frontSensorValues[0]);

		for (unsigned char i = 0; i < LINE_NUMBER_OF_SENSORS; i++)
		{
			BLUETOOTH.print(lineSensorValues[i]);
			BLUETOOTH.print(' ');
		}

		BLUETOOTH.println();
	}
}

void runPID(int _maxSpeed, unsigned char _adaptiveSpeed = 0)
{
	int _runSpeed = _maxSpeed;

	while (true)
	{
		if (_adaptiveSpeed)
		{
			if (stringCounter == 0)
			{
				_runSpeed = TURN_APPROACH_SPEED;
			}

			else
			{
				if (exitTurnDistance[stringCounter - 1] - motorEncoder.read() < DISTANCE_TO_TURN)
				{
					_runSpeed = TURN_APPROACH_SPEED;
				}

				else
				{
					_runSpeed = _maxSpeed;
				}
			}
		}

		position = readLineSensor();

		if (FOUND_LEFT() || FOUND_RIGHT())
			break;

		if (DEBUG_MODE == 1)
			showSensorValues();

		proportional = position - 3500;
	
		derivative = proportional - lastProportional;
		lastProportional = proportional;
		integral += proportional;

		if (_adaptiveSpeed )
			control = KP_OUT * proportional + KI_OUT * integral + KD_OUT * derivative;

		else
			control = KP_IN * proportional + KI_IN * integral + KD_IN * derivative;

		if (DEBUG_MODE == 2)
		{
			Serial.print("Control  ");
			Serial.print(control);
			Serial.print(" Position  ");
			Serial.print(proportional);
			Serial.println("  ");
		}

		if (control > _runSpeed)
			control = _runSpeed;
		if (control < -_runSpeed)
			control = -_runSpeed;

		if (control < 0)
		{
			motorLeft.write(_runSpeed + control);
			motorRight.write(_runSpeed);
		}
		else
		{
			motorLeft.write(_runSpeed);
			motorRight.write(_runSpeed - control);
		}
	}
}

void loop()
{
	enterMaze();

	RED_LED_LEFT_ON;
	RED_LED_RIGHT_ON;

	reducePath();

	exitMaze();
}


void enterMaze()
{
	motorEncoder.write(0);

	while (1)
	{
		char junctionFlag = 0;
		foundLeft = 0, foundStraight = 0, foundRight = 0;

		runPID(SPEED_MAX_ENTRY);

		entryTurnDistance[pathCounter] = motorEncoder.read();

		turn('S', 512, 0);

		readFrontSensor();
		readLineSensor();

		do
		{
			if (!ON_LINE(lineSensorValues[0]) && !ON_LINE(lineSensorValues[7]))
			{
				junctionFlag = 1;
				break;
			}

			readFrontSensor();
			readLineSensor();

			if (FOUND_LEFT())
				foundLeft = 1;
			if (FOUND_RIGHT())
				foundRight = 1;
		} while (FOUND_FRONT());

		if (junctionFlag && foundLeft && foundRight)
		{
			turn('S', 1024, 100);

			turn('S', 1024, 20);
			path[pathCounter++] = 'J';
			path[pathCounter] = '\0';

			break;
		}

		char direction = selectTurn(foundLeft, foundRight, foundStraight);
		turn(direction, SPEED_TURN, DELAY_TURN);

		motorEncoder.write(0);

		turn('S', 1024, 50);

		path[pathCounter++] = direction;
	}
}

void reducePath()
{
	stringCounter = 0;
	int i;

	// Add the last square's turns as it is
	for (i = 1; i <= 3; i++)
	{
		exitTurnDistance[stringCounter] = entryTurnDistance[pathCounter - 1 - i];
		simplifiedPath[stringCounter++] = path[pathCounter - 1 - i];
	}

	for (i = pathCounter - 5; i >= 0; i--)
	{
		exitTurnDistance[stringCounter] = entryTurnDistance[i];

		switch (path[i])
		{
		case 'L':
			simplifiedPath[stringCounter++] = 'R';
			break;
		case 'R':
			simplifiedPath[stringCounter++] = 'L';
			break;
		}
	}

	simplifiedPath[stringCounter++] = 'J';
	simplifiedPath[stringCounter] = '\0';

	for (i = 3; i < stringCounter; i++)
	{
		if (simplifiedPath[i] == 'L' && simplifiedPath[i + 1] == 'L' && simplifiedPath[i + 2] == 'L' && simplifiedPath[i + 3] == 'L' && simplifiedPath[i + 4] == 'R')
		{
			simplifiedPath[i] = 'R';
			simplifiedPath[i + 1] = 'R';
			simplifiedPath[i + 2] = 'L';
			simplifiedPath[i + 3] = 'O';
			simplifiedPath[i + 4] = 'O';

			exitTurnDistance[i + 1] /= 2;
			exitTurnDistance[i + 2] = exitTurnDistance[i + 4];
			exitTurnDistance[i + 3] = 0;
			exitTurnDistance[i + 4] = 0;
		}

		if (simplifiedPath[i] == 'R' && simplifiedPath[i + 1] == 'R' && simplifiedPath[i + 2] == 'R' && simplifiedPath[i + 3] == 'R' && simplifiedPath[i + 4] == 'L')
		{
			simplifiedPath[i] = 'L';
			simplifiedPath[i + 1] = 'L';
			simplifiedPath[i + 2] = 'R';
			simplifiedPath[i + 3] = 'O';
			simplifiedPath[i + 4] = 'O';

			exitTurnDistance[i + 1] /= 2;
			exitTurnDistance[i + 2] = exitTurnDistance[i + 4];
			exitTurnDistance[i + 3] = 0;
			exitTurnDistance[i + 4] = 0;
		}
	}

	stringCounter = 0;
}

void exitMaze()
{
	while (true)
	{
		foundLeft = 0, foundStraight = 0, foundRight = 0;

		motorEncoder.write(0);

		runPID(SPEED_MAX_EXIT, 1);

		for (unsigned int i = 0; i < 3; i++)
		{
			position = readLineSensor();

			if (FOUND_LEFT())
				foundLeft = 1;
			if (FOUND_RIGHT())
				foundRight = 1;
		}

		if (foundLeft || foundRight)
		{
			while (simplifiedPath[stringCounter] == 'O')
				stringCounter++;

			if (simplifiedPath[stringCounter] == 'J')
			{
				turn('S', 1024, 50);

				DISABLE_STANDBY;

				while (1)
				{
					RED_LED_LEFT_OFF;
					RED_LED_RIGHT_OFF;
					GREEN_LED_LEFT_ON;
					GREEN_LED_RIGHT_ON;
				}
			}

			else
				turn(simplifiedPath[stringCounter++], SPEED_TURN, DELAY_TURN);

			turn('S', 1024, 50);
		}
	}
}