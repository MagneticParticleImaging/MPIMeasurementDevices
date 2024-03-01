#ifndef SERIAL_COMMUNICATION_H
#define SERIAL_COMMUNICATION_H

#include "Arduino.h"


#define BEGIN_DELIM '!'
#define END_DELIM '*'
#define CMD_DELIM '#'

typedef struct
{
    const char *id;
    int (*callback)(char *);
} commandCallback_t;


template<int N = 256>
class SerialHandler {
    
    public:
    SerialHandler(commandCallback_t cmds[], int numCmds);
    int read();

    private:
    bool updateBufferUntilDelim();
    static const int buffer_size = N;
    int buffer_position;
    char buffer[N];
    
    int cmd_size;
    commandCallback_t *cmds;

};

#endif