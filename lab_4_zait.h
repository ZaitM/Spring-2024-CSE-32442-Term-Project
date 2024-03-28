#ifndef LAB_4_ZAIT_H
#define LAB_4_ZAIT_H
#include <stdint.h>

// Maximum number of chars that can be accepted from the user
// and the structure for holding UI info
#define MAX_CHARS 80
#define MAX_FIELDS 5

typedef struct _USER_DATA
{
    char buffer[MAX_CHARS + 1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    uint8_t fieldType[MAX_FIELDS];
} USER_DATA;

void getsUart0(USER_DATA *dataStruct);
/**
 * @brief Clear the struct members
 *
 * @return no return
 */
void clearStruct(USER_DATA *dataStruct);
void parseFields(USER_DATA *dataStruct);

/**
 * @brief Detects wether it is alphabetic and returns a boolean
 * @param c Input character
 * @return bool
 */
bool isAlpha(char c);

/**
 * @brief Detects wether char is numeric
 * @param c Input character
 * @return bool
 */
bool isNumeric(char c);

/**
 * @brief Determine if provide command with minimum number
 * of arguments satisfies contents in buffer
 *
 * @param dataStruct
 * @param command
 * @param minArgs
 * @return true If valid input from user
 * @return false
 */
bool isCommand(USER_DATA *dataStruct, const char command[], uint8_t minArgs);

/**
 * @brief Compare strings
 *
 * @return true
 * @return false
 */
bool strCmp(const char *str1, const char *str2);

/**
 * @brief Get the field value
 * @param dataStruct
 * @param fieldNumber
 *
 * @return The value of the field requested if it exists or NULL otherwise
 */
char *getFieldString(USER_DATA *dataStruct, uint8_t fieldNumber);

/**
 * @brief Function to return the integer value of the field if it exists and is numeric
 * otherwise it returns 0
 * @param dataStruct
 * @param fieldNumber
 * @return int
 */
int32_t getFieldInteger(USER_DATA *dataStruct, uint8_t fieldNumber);

/**
 * @brief ASCII to integer conversion
 * @param str
 * @return int32_t
 */
int32_t atoi(const char *str);
#endif
