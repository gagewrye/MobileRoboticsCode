#include <wscommunicator.h>
#include <Display.h>

// Network configuration (these would be provided in reality)
const char *SSID = "Pomona";
const uint16_t port = 8181;

// Update interval for refreshing the display (in milliseconds)
const unsigned long DISPLAY_UPDATE_INTERVAL = 1000; // 1 second
unsigned long lastDisplayUpdate = 0;

// Display configuration
Display display(SSID, port, DISPLAY_UPDATE_INTERVAL); // Correct initialization with arguments
void setup()
{
    Serial.begin(115200);
    display.setup();
}
void loop()
{
    unsigned long currentMillis = millis();

    // Update the display at the defined interval
    if (currentMillis - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL)
    {
        display.loopStep();
        lastDisplayUpdate = currentMillis;
    }
}