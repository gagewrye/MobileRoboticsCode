#include "../include/kinematics.h"
#include "../include/wscommunicator.h"
#include "../include/motorcontrol.h"
#include "../include/display.h"
#include "../include/intervaltimer.h"
#include "positioncontrol.h"
#include <math.h>

// Globals
const float WHEEL_DIAMETER = 0.062;
const float WHEEL_DISTANCE = 0.1; // distance between wheels
IntervalTimer timer(10000);
IntervalTimer messageTimer(250);
char message[100];

// Network Config
const char *SSID = "Pomona";
const uint16_t PORT = 8181;
const unsigned long HEARTBEAT_INTERVAL = 1000;
WsCommunicator wsCommunicator(SSID, PORT, HEARTBEAT_INTERVAL);

// Motor Config
const float LEFT_MOTOR_GAIN = 1.0;
const float RIGHT_MOTOR_GAIN = 1.0;
const float MAX_VELOCITY_STEP = 0.1;
const float MAX_VELOCITY = 1.0;
const long MIN_PWM_PERCENT = 5;
const unsigned long MOTOR_INTERVAL = 100;
MotorControl motors(
    WHEEL_DIAMETER * PI, // Wheel circumference
    LEFT_MOTOR_GAIN,
    RIGHT_MOTOR_GAIN,
    MAX_VELOCITY_STEP,
    MAX_VELOCITY,
    MIN_PWM_PERCENT,
    MOTOR_INTERVAL
);

// Postion Control Config
const float GOAL_X = 1.0;
const float GOAL_Y = 1.0;
const float GOAL_THRESHOLD = 0.1;
const float MAX_ANGULAR_VELOCITY = 0.5;
const float K_POSITION = 1.0;
const float K_ORIENTATION = 2.0;
PositionControl positionControl(
    GOAL_X, 
    GOAL_Y, 
    GOAL_THRESHOLD, 
    MAX_VELOCITY, 
    MAX_ANGULAR_VELOCITY, 
    K_POSITION, 
    K_ORIENTATION,
    MOTOR_INTERVAL,
    WHEEL_DISTANCE
);

// Position Tracking
Kinematics kinematics(WHEEL_DISTANCE, 50);

// Display Config
Display display;

void setup()
{
    // Start Communication
    Serial.begin(115200);
    wsCommunicator.setup();

    // Motor Config
    motors.setup();
    motors.setTargetVelocity(0.2, 0.25); // set left wheel to 0.2 m/s and right wheel to 0.25 m/s
    kinematics.setup();

    // Position Control Config
    positionControl.setup();

    // Display Config
    display.setup();
    display.drawString(0, 0, wsCommunicator.getIpAddress());
}

void loop()
{
    float leftVelocity = motors.getLeftVelocity();
    float rightVelocity = motors.getRightVelocity();
    
    // Updates
    wsCommunicator.loopStep();
    motors.loopStep(wsCommunicator.isEnabled());
    kinematics.loopStep(leftVelocity, rightVelocity);
    positionControl.loopStep(kinematics.getX(), kinematics.getY(), kinematics.getTheta(), leftVelocity, rightVelocity);

    // output the x, y, and theta values to the serial monitor
    if (messageTimer){
        snprintf(message, sizeof(message), "x=%f, y=%f, theta=%f vl=%f vr=%f", kinematics.getX(), kinematics.getY(), kinematics.getTheta(), leftVelocity, rightVelocity);
        wsCommunicator.sendText(message, strlen(message));
    }
}