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

#endif