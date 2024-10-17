#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <cmath>

class Kinematics
{
private:
    float x, y, theta;
    const float r; // wheel radius
    const float d; // distance between wheels

public:
    // Constructor
    Kinematics(float radius, float distance) : r(radius), d(distance), x(0), y(0), theta(0) {}

    // Setup function
    void setup() {}

    void update_position(float phi_L_dot, float phi_R_dot, float dt)
    {
        float R_x_dot = (phi_L_dot * r + phi_R_dot * r) / 2;
        float R_theta_dot = (phi_L_dot * r - phi_R_dot * r) / (2 * d);

        float G_x_dot = R_x_dot * cos(theta);
        float G_y_dot = R_x_dot * sin(theta);

        x += G_x_dot * dt;
        y += G_y_dot * dt;
        theta += R_theta_dot * dt;
    }

    void loopStep(float phi_L_dot, float phi_R_dot, float dt)
    {
        update_position(phi_L_dot, phi_R_dot, dt);
    }

    float getX() { return x; }
    float getY() { return y; }
    float getTheta() { return theta; }
};

#endif
