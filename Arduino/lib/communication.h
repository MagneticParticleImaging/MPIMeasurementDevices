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
    /*
        Constructs a SerialHandler which reads the Serial input and tries to find a fitting callback to invoke.

        @param size: Number of characters in the input buffer, must be longer than the longest command
        @param cmds: Array of callbacks, first fitting ID is invoked per found command
        @param numCmds: Size of cmds array
    */
    SerialHandler(int size, commandCallback_t cmds[], int numCmds) : buffer_size(size), cmds(cmds), cmd_size(numCmds) {
        buffer_position = 0;
        buffer = new char [buffer_size];
    };
    ~SerialHandler() {
        delete buffer;
    };
    /*
        Reads characters from the Serial input until no more are available or a command delimiter has been found.
        If a command delimiter has been found it invokes the first fitting callback (case-sensitive).
    */
    int read();
    /*
        Count the number of occurences in a given null-terminated string.
    */
    int countChars(char *s, char c);
    /*
        Reads a comma-sepearted list of numbers from a null-termianted string. Excpects to find size elements, otherwise returns an error.

        @return 0: Successfully parsed size numbers
        @return -1: Found an invalid digit
        @return -2: String ends before all numbers could be parsed
    
    */
    int readNumbers(char *s, int* array, int size);
    int readNumbers(char *s, double *array, int size);

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
            int length = commandOnly < strlen(cmds[i].id) ? commandOnly : strlen(cmds[i].id);
            if (strncmp(cmds[i].id, command, length) == 0)
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

int SerialHandler::countChars(char* string, char search)
{
    int count = 0;
    for (int i = 0; i < buffer_size; i++) {
        if (string[i] == search) {
            count++;
        }
        if (string[i] == '\0') {
            break; 
        }
    }
    return count;
}

int SerialHandler::readNumbers(char *base, int* array, int size)
{
    int result = 0;
    char * end;
    for (int i = 0; i < size; i++) {
        array[i] = strtol(base, &end, 0);

        if (base == end) { // No (valid) digits found
            return -1;
        } else if (*end == '\0' && i != size - 1) { // Premature end of string
            return -2;
        }

        base = strchr(end, ',');
        if (base == NULL && i != size - 1) { // Premature end of string
            return -3;
        }
        base++; // Advance past ','
    }

    return result;
}

int SerialHandler::readNumbers(char *base, double* array, int size)
{
    int result = 0;
    char * end;
    for (int i = 0; i < size; i++) {
        array[i] = strtod(base, &end);

        if (base == end) { // No (valid) digit found
            return -1;
        } else if (*end == '\0' && i != size - 1) { // Premature end of string
            return -2;
        }

        base = strchr(end, ',');
        if (base == NULL && i != size - 1) { // Premature end of string
            return -3;
        }
        base++; // Advance past ','
    }

    return result;
}

#endif