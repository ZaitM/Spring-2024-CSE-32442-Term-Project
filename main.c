// Lab_05
// Zait Martinez

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1
/*
    March 13th, 2024
    Testing DRV8833
    M0PWM0 Port B Pin 6
    M0PWM1 Port B Pin 7
    (M0PWM0, M0PWM1) -> (x,y)

    March 17th, 2024
    M0PWM2 Port B Pin 4
    M0PWM3 Port B Pin 5
    (M0PWM2, M0PWM3) -> (x,y)

    March 18th, 2024
    Port C Pin 4 Photo-transistor
    Port C Pin 5 Photo-transistor
*/

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "wait.h"
#include "clock.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"

// Port B masks
#define M0PWM2_MASK 16
#define M0PWM3_MASK 32
#define M0PWM0_MASK 64
#define M0PWM1_MASK 128

// Port C masks
#define PHOTO_TR_MASK 16
/* March 20th, 2024 No longer needed */
// #define PHOTO_TR_MASK_2 32

// Port D masks
#define DRV_SLEEP_MASK 64

// Port D bitband aliases
#define DRV_SLEEP (*((volatile uint32_t *)(0x42000000 + (0x400243FC - 0x40000000) * 32 + 0 * 4)))

// Port E masks
#define PHOTO_TR_MASK_2 1

// Port F bitband aliases
#define RED_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC - 0x40000000) * 32 + 1 * 4)))
#define BLUE_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC - 0x40000000) * 32 + 2 * 4)))
#define GREEN_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC - 0x40000000) * 32 + 3 * 4)))

// Port F masks
#define RED_LED_MASK 2
#define BLUE_LED_MASK 4
#define GREEN_LED_MASK 8

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

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initPWM(void);
void initDRVSleep(void);
void initLEDs(void);
void initEdgeTrigInputs(void);

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();
    /*
        Summary of ports
        Port B: M0PWM0, M0PWM1, M0PWM2, M0PWM3
        Port C: Photo-transistor
        Port D: DRV8833 Sleep
        Port E: Photo-transistor
        Port F: Red, Blue, Green LED
    */
    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5 | SYSCTL_RCGCGPIO_R4 | SYSCTL_RCGCGPIO_R3 | SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R1;
    // Enable clock for Module 0 PWM
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    // Enable clock for timer
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R5 | SYSCTL_RCGCTIMER_R4;
    _delay_cycles(3);

    initPWM();
    initDRVSleep();
    initLEDs();
    initEdgeTrigInputs();
}

void initPWM(void)
{
    // Configure GPIO for (M0PWM0, M0PWM1) and (M0PWM2, M0PWM3)
    GPIO_PORTB_DEN_R |= M0PWM0_MASK | M0PWM1_MASK | M0PWM2_MASK | M0PWM3_MASK;
    GPIO_PORTB_AFSEL_R |= M0PWM0_MASK | M0PWM1_MASK | M0PWM2_MASK | M0PWM3_MASK;
    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB4_M | GPIO_PCTL_PB5_M | GPIO_PCTL_PB6_M | GPIO_PCTL_PB7_M);
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB4_M0PWM2 | GPIO_PCTL_PB5_M0PWM3 | GPIO_PCTL_PB6_M0PWM0 | GPIO_PCTL_PB7_M0PWM1;

    // Configure SYSCTL for M0PWM0 and M0PWM1
    SYSCTL_SRPWM_R |= SYSCTL_SRPWM_R0;    // PWM Module 0 is reset
    SYSCTL_SRPWM_R &= ~(SYSCTL_SRPWM_R0); // Exit reset state

    /* Configure generator 0 pwmA and pwmB signals */
    PWM0_0_CTL_R = 0; // Turn-off module 0 generator 0 (driver 0 & 1)

    /* TODO 3/14/2024 Change to 0x8C*/
    // 'A' signal uses cmpA in gen 0 'A' signal
    PWM0_0_GENA_R = PWM_0_GENA_ACTCMPAD_ONE; // Drive pwmA high when counter matches cmpa while counting down
    PWM0_0_GENA_R = PWM_0_GENA_ACTLOAD_ZERO; // Drive pwmA low when counter matches LOAD value in reg

    PWM0_0_GENB_R = PWM_0_GENB_ACTCMPBD_ONE; // Drive pwmB high when counter matches cmpb while counting down
    PWM0_0_GENB_R = PWM_0_GENB_ACTLOAD_ZERO; // Drive pwmB low when counter matches LOAD value in reg

    PWM0_0_LOAD_R = 0;

    PWM0_0_CMPA_R = 0;
    PWM0_0_CMPB_R = 0;

    PWM0_0_CTL_R = PWM_0_CTL_ENABLE; // Turn on generator 0

    PWM0_ENABLE_R = PWM_ENABLE_PWM0EN | PWM_ENABLE_PWM1EN;

    /* Configure generator 1 pwmA and pwmB signals */
    // 3/17
    PWM0_1_CTL_R = 0; // Turn-off module 0 generator 1 (drives 2 & 3)

    PWM0_1_GENA_R = PWM_1_GENA_ACTCMPAD_ONE;
    PWM0_1_GENA_R = PWM_1_GENA_ACTLOAD_ZERO;

    PWM0_1_GENB_R = PWM_1_GENB_ACTCMPBD_ONE;
    PWM0_1_GENB_R = PWM_1_GENB_ACTLOAD_ZERO;

    PWM0_1_LOAD_R = 0;

    PWM0_1_CMPA_R = 0;
    PWM0_1_CMPB_R = 0;

    PWM0_1_CTL_R = PWM_1_CTL_ENABLE; // Turn on generator 1
    PWM0_ENABLE_R = PWM_ENABLE_PWM2EN | PWM_ENABLE_PWM3EN;
}

void initDRVSleep(void)
{
    // Configure DRV8833 Sleep
    GPIO_PORTD_DIR_R |= DRV_SLEEP_MASK;  // bits 6 is an output
    GPIO_PORTD_DR2R_R |= DRV_SLEEP_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTD_DEN_R |= DRV_SLEEP_MASK;  // enable LEDs
    DRV_SLEEP = 1;                       // Turn off DRV8833 by default. Active low. 1 = off, 0 = on
}

void initLEDs(void)
{
    // Configure LED pins
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK | RED_LED_MASK | BLUE_LED_MASK;  // bits 1 and 3 are outputs
    GPIO_PORTF_DR2R_R |= GREEN_LED_MASK | RED_LED_MASK | BLUE_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= GREEN_LED_MASK | RED_LED_MASK | BLUE_LED_MASK;  // enable LEDs
}

void initEdgeTrigInputs(void)
{
    /* For edge-triggered interrupts, software must clear the interrupt to enable any further
       interrupts.
    */
    // Configure PC4 to be an input
    GPIO_PORTC_DIR_R &= ~(PHOTO_TR_MASK);
    GPIO_PORTC_DEN_R |= (PHOTO_TR_MASK);
    // Configure edge-triggered interrupt input for PC4
    GPIO_PORTC_IM_R &= ~(PHOTO_TR_MASK); // Mask by clearing IME field. Interrupt Disabled
    GPIO_PORTC_IS_R &= ~(PHOTO_TR_MASK); // Clear bit/pin 4 in order to detect edges
    GPIO_PORTC_IBE_R &= ~(PHOTO_TR_MASK);
    GPIO_PORTC_IEV_R &= ~(PHOTO_TR_MASK); // Clear bit/pin 4 to detect falling edges
    GPIO_PORTC_ICR_R |= (PHOTO_TR_MASK);  // Setting clears bit in RIS and MIS
    GPIO_PORTC_IM_R |= (PHOTO_TR_MASK);   // Interrupt Enabled. Setting a bit int passes to controller

    NVIC_EN0_R = 1 << (INT_GPIOC - 16);

    // Configure PE0 to be an input
    GPIO_PORTE_DIR_R &= ~(PHOTO_TR_MASK_2);
    GPIO_PORTE_DEN_R |= (PHOTO_TR_MASK_2);

    // Configure edge-triggered interrupt input for PE0
    GPIO_PORTE_IM_R &= ~PHOTO_TR_MASK_2; // Mask by clearing IME field. Interrupt Disabled
    GPIO_PORTE_IS_R &= ~PHOTO_TR_MASK_2; // Clear bit/pin 0 in order to detect edges
    GPIO_PORTE_IBE_R &= ~PHOTO_TR_MASK_2;
    GPIO_PORTE_IEV_R &= ~PHOTO_TR_MASK_2; // Clear bit/pin 0 to detect falling edges
    GPIO_PORTE_ICR_R |= PHOTO_TR_MASK_2;  // Setting clears bit in RIS and MIS
    GPIO_PORTE_IM_R |= PHOTO_TR_MASK_2;   // Interrupt Enabled. Setting a bit int passes to controller

    NVIC_EN0_R = 1 << (INT_GPIOE - 16);

    // 25 ms one-shot timer configuration for PC4
    TIMER5_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER5_CFG_R |= TIMER_CFG_32_BIT_TIMER;
    TIMER5_TAMR_R |= TIMER_TAMR_TACDIR | TIMER_TAMR_TAMR_1_SHOT;
    TIMER5_TAILR_R = 1000000;
    TIMER5_IMR_R = TIMER_IMR_TATOIM;
    NVIC_EN2_R = 1 << (INT_TIMER5A - 16 - 64);

    // 25 ms one-shot timer configuration for PE0
    TIMER4_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER4_CFG_R |= TIMER_CFG_32_BIT_TIMER;
    TIMER4_TAMR_R |= TIMER_TAMR_TACDIR | TIMER_TAMR_TAMR_1_SHOT;
    TIMER4_TAILR_R = 1000000;
    TIMER4_IMR_R = TIMER_IMR_TATOIM;
    NVIC_EN2_R = 1 << (INT_TIMER4A - 16 - 64);
}

void photoISR(void)
{
    if (GPIO_PORTC_RIS_R & PHOTO_TR_MASK)
    {
        GREEN_LED ^= 1;
        GPIO_PORTC_IM_R &= ~PHOTO_TR_MASK; // Interrupt Disabled Mask by clearing IME field.
        /* 3/17/24 Have to turn off edge trigger input by the IM register */

        TIMER5_CTL_R |= TIMER_CTL_TAEN;    // Enable timer
        GPIO_PORTC_ICR_R |= PHOTO_TR_MASK; // Clearing interrupt
    }
}

void photoISR2(void)
{
    if (GPIO_PORTE_RIS_R & PHOTO_TR_MASK_2)
    {
        BLUE_LED ^= 1;
        GPIO_PORTE_IM_R &= ~PHOTO_TR_MASK_2; // Mask by clearing IME field. Interrupt Disabled
        /* 3/17/24 Have to turn off edge trigger input by the IM register */

        TIMER4_CTL_R |= TIMER_CTL_TAEN;      // Enable timer
        GPIO_PORTE_ICR_R |= PHOTO_TR_MASK_2; // Clearing interrupt
    }
}

void oneShotISR(void)
{
    GPIO_PORTC_ICR_R |= PHOTO_TR_MASK; // Clearing interrupt
    GPIO_PORTC_IM_R |= PHOTO_TR_MASK;  // Interrupt Enabled. Setting a bit int passes to controller
    TIMER5_ICR_R = TIMER_ICR_TATOCINT; // Clearing one-shot timer interrupt

    /* 3/17/24 Have to turn on (reenable) edge trigger input by the IM register */
}

void oneShotISR2(void)
{
    GPIO_PORTE_ICR_R |= PHOTO_TR_MASK_2; // Clearing interrupt
    GPIO_PORTE_IM_R |= PHOTO_TR_MASK_2;  // Interrupt Enabled. Setting a bit int passes to controller
    TIMER4_ICR_R = TIMER_ICR_TATOCINT;   // Clearing one-shot timer interrupt
}

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

/**
 * @brief Detects wether it is alphabetic and returns a boolean
 * @param c Input character
 * @return bool
 */
bool isAlpha(char c)
{
    return (c >= 65 && c <= 90) | (c >= 97 && c <= 122) ? true : false;
}

/**
 * @brief Detects wether char is numeric
 * @param c Input character
 * @return bool
 */
bool isNumeric(char c)
{
    return (c >= 48 && c <= 57) | (c >= 44 && c <= 46) ? true : false;
}

void parseFields(USER_DATA *dataStruct)
{
    /**
     * @brief
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
/**
 * @brief Clear the struct members
 *
 * @return no return
 */
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
/**
 * @brief Get the field value
 * @param dataStruct
 * @param fieldNumber
 *
 * @return The value of the field requested if it exists or NULL otherwise
 */
char *getFieldString(USER_DATA *dataStruct, uint8_t fieldNumber)
{
    return fieldNumber < dataStruct->fieldCount ? &dataStruct->buffer[dataStruct->fieldPosition[fieldNumber]] : NULL;
}

/**
 * @brief ASCII to integer conversion
 * @param str
 * @return int32_t
 */
int32_t atoi(char *str)
{
    uint8_t i = str[0] == 45 ? 1 : 0;
    int32_t value = 0;

    while (str[i] != 0)
    {
        value = value * 10 + (str[i++] - 48);
    }
    return value;
}

/**
 * @brief Function to return the integer value of the field if it exists and is numeric
 * otherwise it returns 0
 * @param dataStruct
 * @param fieldNumber
 * @return int
 */
int32_t getFieldInteger(USER_DATA *dataStruct, uint8_t fieldNumber)
{
    return (fieldNumber < dataStruct->fieldCount) && (dataStruct->fieldType[fieldNumber] == 110) ? atoi(getFieldString(dataStruct, fieldNumber)) : 0;
}
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

bool isCommand(USER_DATA *dataStruct, const char command[], uint8_t minArgs)
{
    uint8_t i = 0;
    for (i = 0; dataStruct->buffer[i] != 0; i++)
    {
        // If command field does not match command parameter return false
        if (dataStruct->buffer[i] != command[i])
            return false;
    }
    return ((dataStruct->fieldCount - 1) >= minArgs) ? true : false;
}

/**
 * @brief Compare strings
 *
 * @return true
 * @return false
 */
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
/**
 * @brief Manipulate the cmp values ?
 *
 * @return
 */
void setGen0Values(uint16_t frequency, uint16_t x, uint16_t y)
{
    PWM0_0_LOAD_R = frequency;
    PWM0_0_CMPA_R = x;
    PWM0_0_CMPB_R = y;
}

void setGen1Values(uint16_t frequency, uint16_t x, uint16_t y)
{
    PWM0_1_LOAD_R = frequency;
    PWM0_1_CMPA_R = x;
    PWM0_1_CMPB_R = y;
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    USER_DATA data;
    data.fieldCount = 0;

    // Initialize hardware
    initHw();

    // while (true)
    // {
    //     for (j = 0; j < 2048; j++)
    //     {
    //         RED_LED ^= 1;
    //         for (i = 0; i < j; i++)
    //         {
    //             setValues(j, j, i);

    //             // Wait 2 seconds
    //             waitMicrosecond(100000);
    //         }

    //         //            setValues(i, 1023);
    //         //
    //         //            waitMicrosecond(1000000);
    //     }
    // }

    initUart0();

    // Setup UART0 baud rate
    setUart0BaudRate(19200, 40e6);
    int32_t val1 = 0, val2 = 0;

    while (true)
    {
        bool foo = 0;
        putsUart0("********************\n\n");
        putsUart0("Input:\n");

        // Get the string from the user
        getsUart0(&data);

        // Echo back to the user of the TTY interface for testing

        putsUart0("Output:\n");
        putsUart0(data.buffer);
        putcUart0('\n');

        // Parse fields
        parseFields(&data);

#ifdef DEBUG
        uint8_t i = 0;
        for (i = 0; i < data.fieldCount; i++)
        {
            putsUart0("Field ");
            putcUart0(i + 48);
            putsUart0(" :");
            putcUart0(data.fieldType[i]);
            putcUart0('\t');
            putsUart0(&data.buffer[data.fieldPosition[i]]);
            putcUart0('\n');
        }
#endif DEBUG

        /* Validate command and minimum num of args provide by user */
        /* Add a command 'foward' 100% duty cycle */
        if (isCommand(&data, "foward", 1))
        {
            foo = true;
            val1 = getFieldInteger(&data, 1);
            setGen0Values(1024, , );
            setGen1Values(1024, , );
        }

        // Add a  command 'reverse' 100% duty cycle
        if (isCommand(&data, "reverse", 1))
        {
            foo = true;
            val1 = getFieldInteger(&data, 1);
            setGen0Values(1024, , );
            setGen1Values(1024, , );
        }

        // Add a command 'stop' 0% duty cycle
        if (isCommand(&data, "stop", 0))
        {
            foo = true;
            setGen0Values(1024, 0, 0);
            setGen1Values(1024, 0, 0);
        }

        // Add a command 'ccw' 100% duty cycle
        if (isCommand(&data, "ccw", 1))
        {
            foo = true;
            val1 = getFieldInteger(&data, 1);
            setGen0Values(1024, , );
            setGen1Values(1024, , );
        }

        // Add a command 'cw' 100% duty cycle
        if (isCommand(&data, "cw", 1))
        {
            foo = true;
            val1 = getFieldInteger(&data, 1);
            setGen0Values(1024, , );
            setGen1Values(1024, , );
        }
        // Command 'DRV' to turn off/on the DRV8833
        if (isCommand(&data, "DRV", 1))
        {
            foo = true;
            val1 = getFieldInteger(&data, 1);
            val1 == 0 ? putsUart0("Turning on DRV8833\n") : putsUart0("Turning off DRV8833\n");
            DRV_SLEEP = val1;
        }

        foo ? putsUart0("Valid command!\n\n") : putsUart0("Invalid command!\n\n");
        putsUart0("********************");

        // TODO: Ensure consistency with uppercase and lower case commands

        clearStruct(&data);
    }
}
