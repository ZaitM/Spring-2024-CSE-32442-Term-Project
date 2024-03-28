#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "lab_4_zait.h"

void getsUart0(USER_DATA *dataStruct)
{
    uint8_t count = 0;
    char c;
    do
    {
        c = getcUart0();

        // Blocking function
        while (c == 127 && (count == 0 | count == 1))
        {
            if (count > 0)
                count--;
            c = getcUart0();
        }

        // Delete or Backspace
        count = ((c == 8) | (c == 127)) ? (count > 0 ? --count : count) : ++count;

        // LF or CR i.e (Enter or Max char reached) add null terminator
        if ((c == 10 | c == 13) | count == MAX_CHARS)
        {
            dataStruct->buffer[count - 1] = c;
            dataStruct->buffer[count++] = 0;
            // Only need when reached max chars
            c = 0;
        }
        // Printable character
        if (c >= 32 && c < 127)
            dataStruct->buffer[count - 1] = c;
    } while (dataStruct->buffer[count - 1] != 0);
}

bool isAlpha(char c)
{
    return (c >= 65 && c <= 90) | (c >= 97 && c <= 122) ? true : false;
}

bool isNumeric(char c)
{
    return (c >= 48 && c <= 57) ? true : false;
}

void parseFields(USER_DATA *dataStruct)
{
    /*
     * 
     * Alphabetic upper case: 65-90
     * Alphabetic lower case: 97-122
     * Numeric: 48-57
     * Comma, Hyphen, Period : 44, 45, 46
     */
    uint8_t count = 0;

    // Can only parse 5 fields
    while ((dataStruct->buffer[count] != 0) && (dataStruct->fieldCount < MAX_FIELDS))
    {
        // 97 = 'a'
        if (isAlpha(dataStruct->buffer[count]))
        {
            dataStruct->fieldType[dataStruct->fieldCount] = 97;
            dataStruct->fieldPosition[dataStruct->fieldCount++] = count;
        }
        // 110 = 'n'
        else if (isNumeric(dataStruct->buffer[count]))
        {
            dataStruct->fieldType[dataStruct->fieldCount] = 110;
            dataStruct->fieldPosition[dataStruct->fieldCount++] = count;
        }
        else
            dataStruct->buffer[count] = 0;

        // count will be at the delimeter position
        while (isAlpha(dataStruct->buffer[count]) | isNumeric(dataStruct->buffer[count++]))
            ;
        dataStruct->buffer[count - 1] = 0;
    }
}

void clearStruct(USER_DATA *dataStruct)
{
    uint8_t i = 0;
    for (i = 0; i < MAX_CHARS; i++)
        dataStruct->buffer[i] = 0;
    for (i = 0; i < MAX_FIELDS; i++)
    {
        dataStruct->fieldPosition[i] = 0;
        dataStruct->fieldType[i] = 0;
    }
    dataStruct->fieldCount = 0;
}

char *getFieldString(USER_DATA *dataStruct, uint8_t fieldNumber)
{
    return fieldNumber < dataStruct->fieldCount ? &dataStruct->buffer[dataStruct->fieldPosition[fieldNumber]] : NULL;
}

int32_t atoi(const char *str)
{
    uint8_t i = str[0] == 45 ? 1 : 0;
    int32_t value = 0;

    while (str[i] != 0)
    {
        value = value * 10 + (str[i++] - 48);
    }
    return value;
}

int32_t getFieldInteger(USER_DATA *dataStruct, uint8_t fieldNumber)
{
    return (fieldNumber < dataStruct->fieldCount) && (dataStruct->fieldType[fieldNumber] == 110) ? atoi(getFieldString(dataStruct, fieldNumber)) : 0;
}

bool isCommand(USER_DATA *dataStruct, const char command[], uint8_t minArgs)
{
    uint8_t i = 0;

    // Find the size of the command
    uint8_t size = 0;
    while (command[size] != 0)
        size++;

    for (i = 0; dataStruct->buffer[i] != 0; i++)
    {
        // If command field does not match command parameter return false
        if (dataStruct->buffer[i] != command[i])
            return false;
    }
    return ((dataStruct->fieldCount - 1) >= minArgs) && (size ==  i)? true : false;
}

bool strCmp(const char str1[], const char str2[])
{
    uint8_t i = 0;
    while (str1[i] != 0 || str2[i] != 0)
    {
        if (str1[i] != str2[i])
            return false;
        i++;
    }
    return true;
}
