#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include "../include/intervaltimer.h"

class PositionControl{
    float goalX;
    float goalY;
    float goalThreshold;

    float maxLinearVelocity;
    float maxAngularVelocity;

    float K_position;
    float K_orientation;
    float WHEEL_DISTANCE;

    IntervalTimer timer;

    public:
        PositionControl(
            float goalX, float goalY, float goalThreshold, float maxLinearVelocity, float maxAngularVelocity,
            float K_position, float K_orientation, unsigned long updateInterval, float WHEEL_DISTANCE):
            goalX(),
            goalY(), 
            goalThreshold(),
            maxLinearVelocity(), 
            maxAngularVelocity(), 
            K_position(), 
            K_orientation(),
            timer(updateInterval),
            WHEEL_DISTANCE(WHEEL_DISTANCE)
        {};

        void setup(){}

        bool loopStep(float x, float y, float theta, float& leftVelocity, float& rightVelocity)
        {   
            if (timer)
            {
                // Calculate the error
                float dx = goalX - x;
                float dy = goalY - y;
                float distance = sqrt(dx * dx + dy * dy);
                float angle = atan2(dy, dx) - theta;
                angle = atan2(sin(angle), cos(angle));

                // Calculate the linear and angular velocity
                float linearVelocity = K_position * distance;
                float angularVelocity = K_orientation * angle;

                // Limit the linear and angular velocity
                if (linearVelocity > maxLinearVelocity)
                {
                    linearVelocity = maxLinearVelocity;
                }
                if (angularVelocity > maxAngularVelocity)
                {
                    angularVelocity = maxAngularVelocity;
                }

                // Calculate the left and right wheel velocities
                leftVelocity = linearVelocity - angularVelocity * WHEEL_DISTANCE / 2;
                rightVelocity = linearVelocity + angularVelocity * WHEEL_DISTANCE / 2;

                // Check if the goal has been reached
                if (distance < goalThreshold)
                {
                    leftVelocity = 0;
                    rightVelocity = 0;
                }
                return true;
            }
            return false;
        }
};

#endif