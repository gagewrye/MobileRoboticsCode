#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <cmath>
#include "../include/intervaltimer.h"

class Kinematics
{
    IntervalTimer timer;

private:
    float x, y, theta; // position and orientation
    const float d; // distance between wheels

public:
    // Constructor
    Kinematics(float distance, unsigned long interval) : d(distance), x(0), y(0), theta(0), timer(interval) {}

    // Setup function
    void setup() {}

    void update_position(float leftVelocity, float rightVelocity, float dt)
    {
        float R_x_dot = (leftVelocity + rightVelocity ) / 2;
        float R_theta_dot = (leftVelocity - rightVelocity) / d;

        float G_x_dot = R_x_dot * cos(theta);
        float G_y_dot = R_x_dot * sin(theta);

        x += G_x_dot * dt;
        y += G_y_dot * dt;
        theta += R_theta_dot * dt;
    }

    void loopStep(float leftVelocity, float rightVelocity)
    {
        update_position(leftVelocity, rightVelocity, timer.getLastDelta() / 1000.0);
    }

    float getX() { return x; }
    float getY() { return y; }
    float getTheta() { return theta; }

    void setPose(float newX = 0, float newY = 0, float newTheta = 0) { 
        x = newX;
        y = newY;
        theta = newTheta;
    }
}
#endif