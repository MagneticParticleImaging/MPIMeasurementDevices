#ifndef RP_WATCHDOG_H
#define RP_WATCHDOG_H

#include "Arduino.h"

class RedPitayaWatchdog 
{
    public:
    RedPitayaWatchdog(int input, int output, unsigned long timeout) : input(input), output(output), timeout(timeout) {}
    void setup();
    void reset();
    bool check();
    bool hasFailed();
    
    private:
    void send();

    const int input;
    const int output;
    const unsigned long timeout;
    unsigned long time;
    bool value;
    bool fail;
};

#include "Arduino.h"
#include "rp_watchdog.h"

void RedPitayaWatchdog::setup()
{
    pinMode(output, OUTPUT);
    pinMode(input, INPUT);
}

void RedPitayaWatchdog::reset()
{
    value = true;
    fail = false;
    send();
}


bool RedPitayaWatchdog::hasFailed()
{
    return fail;
}

void RedPitayaWatchdog::send()
{
    digitalWrite(output, value);
    time = micros();
}

bool RedPitayaWatchdog::check()
{
    if (digitalRead(input) == value)
    {
        fail = false;
        value = !value;
        send();
    }
    else if ((micros() - time) > timeout)
    {
        fail = true;
    }
    return fail;
}

#endif