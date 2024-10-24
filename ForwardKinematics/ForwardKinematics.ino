#include "../include/kinematics.h"
#include "../include/wscommunicator.h"
#include "../include/motorcontrol.h"
#include "../include/display.h"
#include "../include/intervaltimer.h"
#include <math.h>

// Globals
const float WHEEL_DIAMETER = 0.062;
const float WHEEL_DISTANCE = 0.1; // distance between wheels

// Network configuration
const char *SSID = "Pomona";
const uint16_t PORT = 8181;
const unsigned long HEARTBEAT_INTERVAL = 1000;

Kinematics kinematics(WHEEL_DISTANCE, 50);
Display display;
IntervalTimer timer(10000);
WSCommunicator wsCommunicator(SSID, PORT, HEARTBEAT_INTERVAL);

MotorControl motors(
    WHEEL_DIAMETER * PI, // Wheel circumference
    1.0,                // Left motor gain
    1.0,                // Right motor gain
    0.1,                // Maximum velocity step
    1.0,                // Maximum velocity
    5,                  // Minimum PWM percent
    100                 // Interval for updates (ms)
);

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
    if (timer.getLastDelta() % 250 == 0)
    {
        Serial.printf("x: %f, y: %f, theta: %f\n", kinematics.getX(), kinematics.getY(), kinematics.getTheta());
    }

    if (timer) // End the loop after 10 seconds
    {
        motors.stop();
        return;
    }
    // 4. plot the x and y values on a 2D scatter plot
    // 5. plot the theta values on a line plot
}