#ifndef DISPLAY_H
#define DISPLAY_H

#include <wscommunicator.h>
#include <Arduino.h>
#include <U8g2lib.h>

class Display
{
    WSCommunicator wsCommunicator;

public:
    // Constructor with member initialization for wsCommunicator and u8g2
    Display(const char *ssid, uint16_t port, unsigned long interval)
        : wsCommunicator(ssid, port, interval), u8g2(U8G2_R0, U8X8_PIN_NONE), ssid(ssid), port(port), interval(interval) {}

    void setup()
    {
        wsCommunicator.setup();
        u8g2.begin();                       // Initialize the display
        u8g2.setFont(u8g2_font_ncenB08_tr); // Set the font for the display
    }

    void loopStep()
    {
        const char *IP = getIpAddress(); // Get IP address as string
        uint16_t port = getPort();       // Get port number

        u8g2.clearBuffer(); // Clear the buffer
        u8g2.setCursor(0, 20);
        u8g2.print("IP: ");
        String fullIP = String(IP) + ":" + String(port); // Concatenate IP and port
        u8g2.print(fullIP.c_str());                      // Display the IP and port
        u8g2.sendBuffer();                               // Send the buffer to the display
    }

    const char *getIpAddress() { return wsCommunicator.getIpAddress(); } // Added missing semicolon
    uint16_t getPort() { return port; }

private:
    const char *ssid;
    uint16_t port;
    unsigned long interval;
    U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2; // U8g2 display object
};

#endif // DISPLAY_H
