#include "Arduino.h"
#include "communication.h"

template<int N = 256>
SerialHandler::SerialHandler(commandCallback_t *callbacks, int numCmds)
{
    buffer_position = 0;
    cmd_size = numCmds;
    cmds = callbacks;
}

template<int N = 256>
bool SerialHandler::updateBufferUntilDelim()
{
    char nextChar;
    while (Serial.available() > 0)
    {
        nextChar = Serial.read();
        if (nextChar == CMD_DELIM)
        {
            buffer[buffer_posiiton] = '\0';
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

template<int N = 256>
void SerialHandler::read() // Serial communication with PC in particular julia communication
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
        delimFound = updateBufferUntilDelim('#');
    }

    if !(delimFound)
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
                input_buffer[0] = '\0'; // "Empty" input buffer
                return 0;
            }
        }
    }

    return -1;
}
