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


class SerialHandler {
    
    public:
    SerialHandler(int size, commandCallback_t cmds[], int numCmds) : buffer_size(size), cmds(cmds), cmd_size(numCmds) {
        buffer_position = 0;
        buffer = new char [buffer_size];
    };
    ~SerialHandler() {
        delete buffer;
    };
    int read();

    private:
    bool updateBufferUntilDelim();
    const int buffer_size;
    int buffer_position;
    char *buffer;
    
    int cmd_size;
    commandCallback_t *cmds;

};

bool SerialHandler::updateBufferUntilDelim()
{
    char nextChar;
    while (Serial.available() > 0)
    {
        nextChar = Serial.read();
        if (nextChar == CMD_DELIM)
        {
            buffer[buffer_position] = '\0';
            buffer_position = 0;
            return true;
        }
        else if (nextChar != '\n')
        {
            buffer[buffer_position % (buffer_size - 1)] = nextChar; // Size - 1 to always leave room for \0
            buffer_position++;
        }
    }
    return false;
}

int SerialHandler::read() // Serial communication with PC in particular julia communication
{
    int s = -1;
    int e = -1;
    char beginDelim = BEGIN_DELIM;
    char *beginCmd;
    char endDelim = END_DELIM;
    char *endCmd;
    char *command;
    char *paramStart;
    bool delimFound = false;
    bool unknown = false;

    // determin substring
    if (Serial.available() > 0)
    {
        delimFound = updateBufferUntilDelim();
    }

    if (!delimFound)
    {
        return 0;
    }

    s = -1;
    if ((beginCmd = strchr(buffer, beginDelim)) != NULL)
    {
        s = beginCmd - buffer;
    }

    e = -1;
    if ((endCmd = strchr(buffer, endDelim)) != NULL)
    {
        e = endCmd - buffer;
    }

    unknown = true;
    // check if valid command
    if (e != -1 && s != -1)
    {
        command = beginCmd + 1;
        *endCmd = '\0';
        // check for known commands
        /// Getting rid of parameters at the end of String
        int commandOnly = 0;
        paramStart = strchr(command, '<');
        if (paramStart != NULL)
        {
            commandOnly = command - paramStart;
        }
        else
        {
            commandOnly = endCmd - command - 1;
        }
        for (int i = 0; i < cmd_size; i++)
        {
            if (strncmp(cmds[i].id, command, commandOnly) == 0)
            {
                cmds[i].callback(command);
                unknown = false;
                buffer[0] = '\0'; // "Empty" input buffer
                return 0;
            }
        }
    }

    return -1;
}

#endif