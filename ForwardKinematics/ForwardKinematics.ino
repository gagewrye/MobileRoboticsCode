#include <kinematics.h>
#include <wscommunicator.h>
#include <motorcontrol.h>
#include <display.h>

// Globals
const float r = 0.1; // wheel radius
const float d = 0.1; // distance between wheels

Kinematics kinematics(r, d);
Display display;
MotorControl motorControl;
WSCommunicator wsCommunicator;

// For the exercise, I want you to

void setup()
{
    //     Start serial
    Serial.begin(115200);
    //     Start the wsCommunicator
    wsCommunicator.setup();
    //     Start the motorControl
    motorControl.setup();
    //     Set the motor target velocity
    motorControl.setTargetVelocity(0.2, 0.25);
    //     Start the display
    display.setup();
    //     Display the IP address
    display.drawString(0, 0, wsCommunicator.getIpAddress());
    //     Start the kinematics
    kinematics.setup();
}

void loop()
{
    //     Update the wsCommunicator
    wsCommunicator.loopStep();
    //     Update the motorControl
    motorControl.loopStep(wsCommunicator.isEnabled());
    //     Update the kinematics
    kinematics.loopStep(motorControl.leftEncoder.getRotationsPerSecond(), motorControl.rightEncoder.getRotationsPerSecond(), 0.25);
    //     Output the current pose
    Serial.printf("x: %f, y: %f, theta: %f\n", kinematics.getX(), kinematics.getY(), kinematics.getTheta());

    // 1. set your left wheel to 0.2 m/s and your right wheel to 0.25 m/s
    // 2. have your robot move for 10 s
    // 3. output the x, y, and theta values to the serial monitor every 250 ms
    // 4. plot the x and y values on a 2D scatter plot
    // 5. plot the theta values on a line plot
}