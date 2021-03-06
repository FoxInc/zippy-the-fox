/*
	Pin Definintions
*/

// -- Bluetooth Serial Port --
#define BLUETOOTH Serial1		// Port on which bluetooth is connected

// -- Status LEDs --
#define RED_LED_LEFT	20
#define RED_LED_RIGHT	19

#define GREEN_LED_LEFT	16
#define GREEN_LED_RIGHT 15

// -- Buttons --
#define LEFT_BUTTON		17
#define RIGHT_BUTTON	18

// -- Buzzer --
#define BUZZER 22

// -- Sensors --
// Common
#define TIMEOUT 2500

// Line Sensor - QTR-8RC
#define LINE_NUMBER_OF_SENSORS	8
#define LINE_EMITTER_PIN		25

// Front Sensor - QTR-1RC
#define FRONT_NUMBER_OF_SENSORS 1


// -- Motor Driver --
#define LEFT_MOTOR_A	5
#define LEFT_MOTOR_B	6
#define LEFT_MOTOR_PWM	4

#define RIGHT_MOTOR_A	8
#define RIGHT_MOTOR_B	9
#define RIGHT_MOTOR_PWM	10

#define MOTOR_STANDBY_PIN 7

/*
	Environment Dependent Variables
*/

// -- Debugging --
// 0 for no debugging
// 1 for usb debugging
// 2 for bluetooth debugging
// 3 for both usb and bluetooth
#define DEBUG_CHANNEL 1

// 0 for sensor values
// 
#define DEBUG_MODE 0

// -- Encoder --
#define DISTANCE_TO_TURN 600
#define TURN_APPROACH_SPEED 2048

// -- Robot State Variables --
#define WORKING 1
#define IDLE	0

// -- Sensors --
#define BLACK_ON_WHITE	0
#define WHITE_ON_BLACK	1
#define COMPARE			250

// -- Speed Limits --
#define SPEED_MAX		2048
#define SPEED_MAX_ENTRY	2048
#define SPEED_MAX_EXIT	3192
#define SPEED_CALIBRATE	650
#define SPEED_TURN		1200

// -- PID constants --
#define KP_IN	2
#define KI_IN	0
#define KD_IN	80

#define KP_OUT	2
#define KI_OUT	0
#define KD_OUT	150

// -- Delays --
#define DELAY_TURN 190
