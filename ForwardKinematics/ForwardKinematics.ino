#include "../include/kinematics.h"
#include "../include/wscommunicator.h"
#include "../include/motorcontrol.h"
#include "../include/display.h"
#include "../include/intervaltimer.h"
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

Kinematics kinematics(WHEEL_DISTANCE, 50);
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

    // Display Config
    display.setup();
    display.drawString(0, 0, wsCommunicator.getIpAddress());
}

void loop()
{
    // Updates
    wsCommunicator.loopStep();
    motors.loopStep(wsCommunicator.isEnabled());
    kinematics.loopStep(motors.getLeftVelocity(), motors.getRightVelocity());

    // output the x, y, and theta values to the serial monitor every 250 ms
    if (messageTimer){
        snprintf(message, sizeof(message), "x=%f, y=%f, theta=%f vl=%f vr=%f", kinematics.getX(), kinematics.getY(), kinematics.getTheta(), motors.getLeftVelocity(), motors.getRightVelocity());
        wsCommunicator.sendText(message, strlen(message))
    }

    if (timer) // End the loop after 10 seconds
    {
        motors.stop();
        return;
    }
    // 4. plot the x and y values on a 2D scatter plot
    // 5. plot the theta values on a line plot
}