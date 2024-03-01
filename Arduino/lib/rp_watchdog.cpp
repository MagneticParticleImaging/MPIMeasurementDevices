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

void RedPitayaWatchdog::check()
{
    if (digitalRead(input) == value)
    {
        fail = false;
        value = !value();
        send();
    }
    else if ((micros() - time) > timeout)
    {
        fail = true;
    }
    return fail;
}