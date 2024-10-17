#include <kinematics.h>
#include <wscommunicator.h>

// Globals
float wheel_radius = 0.01;
float wheel_distance = 0.1;
Kinematics kinematics = Kinematics(wheel_radius, wheel_distance);

// Network configuration
const char *SSID = "Pomona";
const uint16_t PORT = 8181;
const unsigned long HEARTBEAT_INTERVAL = 1000;
WSCommunicator wsCommunicator = WSCommunicator(SSID, PORT, HEARTBEAT_INTERVAL);

void setup()
{
    wsCommunicator.setup();
}

void loop()
{
    kinematics.loopStep(1.0, 1.0, 1.0);
    wsCommunicator.loopStep();
    delay(1000);
}
