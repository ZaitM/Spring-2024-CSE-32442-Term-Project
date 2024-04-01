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
    M0PWM2 Port B Pin 4 M0PWM3 Port B Pin 5
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
#include "lab_4_zait.h"

// Global variables
volatile uint16_t leftWheelCount = 0, rightWheelCount = 0;
volatile uint16_t rightWheelGen0Val = 0, leftWheelGen1Val = 0, speed = 0;
char str[40];

// Constants
#define L_WHEEL_DIAMETER 90           // 90.20 mm
#define R_WHEEL_DIAMETER 91           // 90.75 mm
#define BASE_DIAMETER 138             // 5 and 7/16 inches or 138.1 mm
#define BASE_RADIUS 69                // 69.05 mm
#define DISTANCE_ADJUSTMENT_FACTOR 40 // Overshoots by 80 mm
#define PI_OVER_180 17453             // Approximation of pi/180 * 10e6
#define SCALING_FACTOR 1000           // Scaling factor for angle calculations
#define PMM_LOAD_VAL 1024
#define MAX_PWM 1023
#define MIN_PWM 0
#define MAX_SPEED 9988
#define MIN_SPEED 1624
#define ANGLE_ADJUSTMENT_FACTOR_CW 30  // In degrees
#define ANGLE_ADJUSTMENT_FACTOR_CCW 24 // In degrees

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
#define DRV_SLEEP (*((volatile uint32_t *)(0x42000000 + (0x400073FC - 0x40000000) * 32 + 6 * 4)))

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

// Function prototypes
uint16_t speedToPWM(uint16_t speed);
void initPWM(void);
void initDRVSleep(void);
void initLEDs(void);
void initEdgeTrigInputs(void);
void setGen0Values(uint16_t frequency, uint16_t x, uint16_t y);
void setGen1Values(uint16_t frequency, uint16_t x, uint16_t y);
void driveStraightFoward(void);
void driveStragihtReverse(void);
uint16_t absoluteValue(int16_t value);

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();
    /*
        Summary of ports
        Port B: M0PWM0, M0PWM1, M0PWM2, M0PWM3
                B6      B7      B4      B5
        Port C: Photo-transistor
        Port D: DRV8833 Sleep
        Port E: Photo-transistor
        Port F: Red, Blue, Green LED
    */

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5 | SYSCTL_RCGCGPIO_R4 | SYSCTL_RCGCGPIO_R3 | SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R1;
    // Enable clock for Module 0 PWM
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    // Enable clock for Timer
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R5 | SYSCTL_RCGCTIMER_R4 | SYSCTL_RCGCTIMER_R3 | SYSCTL_RCGCTIMER_R2;
    // Enable clock for Wide Timer 0
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R0;
    _delay_cycles(3);
}

void initPWM(void)
{
    /*
     *         Port B: M0PWM0, M0PWM1, M0PWM2, M0PWM3
     *                 B6      B7      B4      B5
     */
    // Configure GPIO for (M0PWM0, M0PWM1) and (M0PWM2, M0PWM3)
    GPIO_PORTB_DEN_R |= (M0PWM0_MASK | M0PWM1_MASK | M0PWM2_MASK | M0PWM3_MASK);
    GPIO_PORTB_AFSEL_R |= (M0PWM0_MASK | M0PWM1_MASK | M0PWM2_MASK | M0PWM3_MASK);
    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB4_M | GPIO_PCTL_PB5_M | GPIO_PCTL_PB6_M | GPIO_PCTL_PB7_M);
    GPIO_PORTB_PCTL_R |= (GPIO_PCTL_PB4_M0PWM2 | GPIO_PCTL_PB5_M0PWM3 | GPIO_PCTL_PB6_M0PWM0 | GPIO_PCTL_PB7_M0PWM1);

    // Configure SYSCTL for M0PWM0 and M0PWM1
    SYSCTL_SRPWM_R |= SYSCTL_SRPWM_R0;    // PWM Module 0 is reset
    SYSCTL_SRPWM_R &= ~(SYSCTL_SRPWM_R0); // Exit reset state

    /* Configure generator 0 pwmA and pwmB signals */
    PWM0_0_CTL_R = 0; // Turn-off module 0 generator 0 (driver 0 & 1)

    /* TODO 3/14/2024 Change to 0x8C*/
    // 'A' signal uses cmpA in gen 0 'A' signal
    PWM0_0_GENA_R |= PWM_0_GENA_ACTCMPAD_ONE; // Drive pwmA high when counter matches cmpa while counting down
    PWM0_0_GENA_R |= PWM_0_GENA_ACTLOAD_ZERO; // Drive pwmA low when counter matches LOAD value in reg

    PWM0_0_GENB_R |= PWM_0_GENB_ACTCMPBD_ONE; // Drive pwmB high when counter matches cmpb while counting down
    PWM0_0_GENB_R |= PWM_0_GENB_ACTLOAD_ZERO; // Drive pwmB low when counter matches LOAD value in reg

    PWM0_0_LOAD_R = 1024;

    PWM0_0_CMPA_R = 0;
    PWM0_0_CMPB_R = 0;

    PWM0_0_CTL_R = PWM_0_CTL_ENABLE; // Turn on generator 0

    PWM0_ENABLE_R |= (PWM_ENABLE_PWM0EN | PWM_ENABLE_PWM1EN);

    /* Configure generator 1 pwmA and pwmB signals */
    // 3/17
    PWM0_1_CTL_R = 0; // Turn-off module 0  generator 1 (drives 2 & 3)

    PWM0_1_GENA_R |= PWM_1_GENA_ACTCMPAD_ONE;
    PWM0_1_GENA_R |= PWM_1_GENA_ACTLOAD_ZERO;

    PWM0_1_GENB_R |= PWM_1_GENB_ACTCMPBD_ONE;
    PWM0_1_GENB_R |= PWM_1_GENB_ACTLOAD_ZERO;

    PWM0_1_LOAD_R = 1024;

    PWM0_1_CMPA_R = 0;
    PWM0_1_CMPB_R = 0;

    PWM0_1_CTL_R = PWM_1_CTL_ENABLE; // Turn on generator 1
    PWM0_ENABLE_R |= (PWM_ENABLE_PWM2EN | PWM_ENABLE_PWM3EN);
}

void initDRVSleep(void)
{
    // Configure DRV8833 Sleep
    GPIO_PORTD_DIR_R |= DRV_SLEEP_MASK; // bits 6 is an output
    GPIO_PORTD_DEN_R |= DRV_SLEEP_MASK; // enable LEDs
    DRV_SLEEP = 0;                      // Turn off DRV8833 by default. Active high
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
    GPIO_PORTC_IBE_R |= (PHOTO_TR_MASK); // Detect both edges
    // GPIO_PORTC_IEV_R &= ~(PHOTO_TR_MASK);  // Clear bit/pin 4 to detect falling edges
    GPIO_PORTC_ICR_R |= (PHOTO_TR_MASK);   // Setting clears bit in RIS and MIS
    GPIO_PORTC_IM_R |= (PHOTO_TR_MASK);    // Interrupt Enabled. Setting a bit int passes to controller
    GPIO_PORTC_AFSEL_R |= PHOTO_TR_MASK;   // Enable alternate function
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC4_M; // Clear bit/pin 4
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_WT0CCP0;

    NVIC_EN0_R = 1 << (INT_GPIOC - 16);

    // Configure PE0 to be an input
    GPIO_PORTE_DIR_R &= ~(PHOTO_TR_MASK_2);
    GPIO_PORTE_DEN_R |= (PHOTO_TR_MASK_2);

    // Configure edge-triggered interrupt input for PE0
    GPIO_PORTE_IM_R &= ~PHOTO_TR_MASK_2; // Mask by clearing IME field. Interrupt Disabled
    GPIO_PORTE_IS_R &= ~PHOTO_TR_MASK_2; // Clear bit/pin 0 in order to detect edges
    GPIO_PORTE_IBE_R |= PHOTO_TR_MASK_2;
    // GPIO_PORTE_IEV_R &= ~PHOTO_TR_MASK_2; // Clear bit/pin 0 to detect falling edges
    GPIO_PORTE_ICR_R |= PHOTO_TR_MASK_2; // Setting clears bit in RIS and MIS
    GPIO_PORTE_IM_R |= PHOTO_TR_MASK_2;  // Interrupt Enabled. Setting a bit int passes to controller

    NVIC_EN0_R = 1 << (INT_GPIOE - 16);

    // 25 ms one-shot timer configuration for PC4
    TIMER5_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER5_CFG_R |= TIMER_CFG_32_BIT_TIMER;
    TIMER5_TAMR_R |= TIMER_TAMR_TACDIR | TIMER_TAMR_TAMR_1_SHOT;
    TIMER5_TAILR_R = 1000000;
    TIMER5_IMR_R = TIMER_IMR_TATOIM;
    NVIC_EN2_R = 1 << (INT_TIMER5A - 16 - 64);

    // Input Edge-Count and Capture Mode Timer for PC4
    // There are 20 holes in wheel
    // Will use this to determine speed of wheel
    // Will determine the number of clock cycles for wheel to make a full rotation

    TIMER3_CTL_R &= ~TIMER_CTL_TAEN;       // Turn-off counter before reconfiguring
    TIMER3_CFG_R = TIMER_CFG_32_BIT_TIMER; // configure as 32-bit timer
    TIMER3_TAMR_R |= TIMER_TAMR_TACDIR | TIMER_TAMR_TAMR_1_SHOT;
    TIMER3_IMR_R = TIMER_IMR_TATOIM;
    NVIC_EN1_R = 1 << (INT_TIMER3A - 16 - 32);

    // 25 ms one-shot timer configuration for PE0
    TIMER4_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER4_CFG_R |= TIMER_CFG_32_BIT_TIMER;
    TIMER4_TAMR_R |= TIMER_TAMR_TACDIR | TIMER_TAMR_TAMR_1_SHOT;
    TIMER4_TAILR_R = 1000000;
    TIMER4_IMR_R = TIMER_IMR_TATOIM;
    NVIC_EN2_R = 1 << (INT_TIMER4A - 16 - 64);
}

// PC4 Right wheel ISR
void rightWheelEdgeISR(void)
{
    rightWheelCount++;
    if (GPIO_PORTC_RIS_R & PHOTO_TR_MASK)
    {
        GREEN_LED ^= 1;
        GPIO_PORTC_IM_R &= ~PHOTO_TR_MASK; // Interrupt Disabled Mask by clearing IME field.
        /* 3/17/24 Have to turn off edge trigger input by the IM register */

        TIMER5_CTL_R |= TIMER_CTL_TAEN;  // Enable 25 ms one-shot timer
        WTIMER0_CTL_R |= TIMER_CTL_TAEN; // turn-on edge counter PC4
        // TIMER3_CTL_R |= TIMER_CTL_TAEN;    // Enable timer 3 for one revolution
        GPIO_PORTC_ICR_R |= PHOTO_TR_MASK; // Clearing interrupt
    }
}

void timer3ISR(void)
{
    TIMER3_CTL_R &= ~TIMER_CTL_TAEN;   // Disable timer
    TIMER3_ICR_R = TIMER_ICR_TATOCINT; // Clearing one-shot timer interrupt
    setGen0Values(1024, 0, 0);
    setGen1Values(1024, 0, 0);
    RED_LED ^= 1;
}

// PE0 Left wheel ISR
void leftWheelEdgeISR(void)
{
    leftWheelCount++;
    if (GPIO_PORTE_RIS_R & PHOTO_TR_MASK_2)
    {
        BLUE_LED ^= 1;
        GPIO_PORTE_IM_R &= ~PHOTO_TR_MASK_2; // Mask by clearing IME field. Interrupt Disabled
        /* 3/17/24 Have to turn off edge trigger input by the IM register */

        TIMER4_CTL_R |= TIMER_CTL_TAEN; // Enable timer

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

uint16_t speedToPWM(uint16_t speed)
{
    /*
        The max speed is 9988 mm/min
        The lowest speed is 1624 mm/min
        Don't worry for no speed val provided
    */

    if (speed == 0)
        return 0;
    else if (speed < 1624)
        return 675;
    else if (speed >= 1624 && speed < 1946)
        return 690;
    else if (speed >= 1946 && speed < 2271)
        return 705;
    else if (speed >= 2271 && speed < 2618)
        return 720;
    else if (speed >= 2618 && speed < 2926)
        return 735;
    else if (speed >= 2923 && speed < 3729)
        return 770;
    else if (speed >= 3729 && speed < 4453)
        return 805;
    else if (speed >= 4453 && speed < 5258)
        return 840;
    else if (speed >= 5258 && speed < 6105)
        return 875;
    else if (speed >= 6105 && speed < 6722)
        return 900;
    else if (speed >= 6722 && speed < 7318)
        return 925;
    else if (speed >= 7318 && speed < 7976)
        return 950;
    else if (speed >= 7976 && speed < 8648)
        return 975;
    else if (speed >= 8648 && speed < 9321)
        return 1000;
    else if (speed >= 9321 && speed < 9988)
        return 1023;
    else
        return 1023;
}

void driveStraightFoward(void)
{
    while (absoluteValue(leftWheelCount - rightWheelCount) > 0)
    {
        if (rightWheelCount > leftWheelCount)
        {
            leftWheelGen1Val = leftWheelGen1Val >= 1023 ? speedToPWM(speed) : leftWheelGen1Val + 1;
            rightWheelGen0Val--;
        }
        else if (leftWheelCount > rightWheelCount)
        {
            rightWheelGen0Val = rightWheelGen0Val >= 1023 ? speedToPWM(speed) : rightWheelGen0Val + 1;
            leftWheelGen1Val--;
        }
        setGen0Values(1024, 0, rightWheelGen0Val);
        setGen1Values(1024, leftWheelGen1Val, 0);

        snprintf(str, sizeof(str), "Left: %d, Right: %d\n", leftWheelCount, rightWheelCount);

        putsUart0(str);

        waitMicrosecond(100000);
    }
}

void driveStraightReverse(void)
{
    while (absoluteValue(leftWheelCount - rightWheelCount) > 0)
    {
        if (rightWheelCount > leftWheelCount)
        {
            leftWheelGen1Val = leftWheelGen1Val >= 1023 ? speedToPWM(speed) : leftWheelGen1Val + 1;
            rightWheelGen0Val--;
        }
        else if (leftWheelCount > rightWheelCount)
        {
            rightWheelGen0Val = rightWheelGen0Val >= 1023 ? speedToPWM(speed) : rightWheelGen0Val + 1;
            leftWheelGen1Val--;
        }
        setGen0Values(1024, rightWheelGen0Val, 0);
        setGen1Values(1024, 0, leftWheelGen1Val);

        snprintf(str, sizeof(str), "Left: %d, Right: %d\n", leftWheelCount, rightWheelCount);

        putsUart0(str);

        waitMicrosecond(50000);
    }
}

uint16_t absoluteValue(int16_t value)
{
    return value < 0 ? -value : value;
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
    initPWM();
    initDRVSleep();
    initLEDs();
    initEdgeTrigInputs();
    initUart0();

    // Setup UART0 baud rate
    setUart0BaudRate(19200, 40e6);
    uint16_t distance = 0, mmArcLength = 0;
    uint8_t DRV_Val = 0, angle = 0;

    while (true)
    {
        bool validCommand = 0;
        uint32_t timeForDistanceDesired = 0;
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

        // Validate command and minimum num of args provide by user
        // Add a command 'forward' 100% duty cycle
        // if no 'speed' is provided, then the default values are used
        if (isCommand(&data, "forward", 0) | isCommand(&data, "forward", 1) | isCommand(&data, "forward", 2))
        {
            leftWheelCount = rightWheelCount = 0;

            validCommand = true;

            speed = (data.fieldCount > 1) ? getFieldInteger(&data, 1) : 9988;
            distance = (data.fieldCount == 3) ? getFieldInteger(&data, 2) : 0;

            leftWheelGen1Val = rightWheelGen0Val = speedToPWM(speed);
            if (data.fieldCount == 3)
            {
                timeForDistanceDesired = ((distance * 60 * 40e6) / speed) - ((DISTANCE_ADJUSTMENT_FACTOR * 40e6 * 60) / speed);

                snprintf(str, sizeof(str), "Time for distance desired: %d\n", timeForDistanceDesired);
                putsUart0(str);

                TIMER3_TAILR_R = timeForDistanceDesired;
                TIMER3_CTL_R |= TIMER_CTL_TAEN; // Enable timer 3 for desired distance

                setGen0Values(1024, 0, rightWheelGen0Val);
                setGen1Values(1024, leftWheelGen1Val, 0);

                waitMicrosecond(200000); // 200 ms Let the motors start running and adjust speed  if needed

                driveStraightFoward();
            }
            else
            {
                setGen0Values(1024, 0, rightWheelGen0Val);
                setGen1Values(1024, leftWheelGen1Val, 0);

                waitMicrosecond(200000); // 200 ms Let the motors start running and adjust speed  if needed

                driveStraightFoward();
            }

            snprintf(str, sizeof(str), "In \"forward\" Left: %d, Right: %d\n", leftWheelCount, rightWheelCount);
            putsUart0(str);
        }

        // Add a  command 'reverse' 100% duty cycle
        else if (isCommand(&data, "reverse", 0) | isCommand(&data, "reverse", 1) | isCommand(&data, "reverse", 2))
        {
            leftWheelCount = rightWheelCount = 0;

            validCommand = true;

            speed = (data.fieldCount > 1) ? getFieldInteger(&data, 1) : 9988;
            distance = (data.fieldCount == 3) ? getFieldInteger(&data, 2) : 0;

            leftWheelGen1Val = rightWheelGen0Val = speedToPWM(speed);
            if (data.fieldCount == 3)
            {
                timeForDistanceDesired = ((distance * 60 * 40e6) / speed) - ((DISTANCE_ADJUSTMENT_FACTOR * 40e6 * 60) / speed);

                snprintf(str, sizeof(str), "Time for distance desired: %d\n", timeForDistanceDesired);
                putsUart0(str);

                TIMER3_TAILR_R = timeForDistanceDesired;
                TIMER3_CTL_R |= TIMER_CTL_TAEN; // Enable timer 3 for desired distance

                setGen0Values(1024, rightWheelGen0Val, 0);
                setGen1Values(1024, 0, leftWheelGen1Val);

                waitMicrosecond(200000);

                driveStraightReverse();
            }
            else
            {
                setGen0Values(1024, rightWheelGen0Val, 0);
                setGen1Values(1024, 0, leftWheelGen1Val);

                waitMicrosecond(200000);

                driveStraightReverse();
            }

            snprintf(str, sizeof(str), "In \"reverse\" Left: %d, Right: %d\n", leftWheelCount, rightWheelCount);
            putsUart0(str);
        }

        // Add a command 'stop' 0% duty cycle
        else if (isCommand(&data, "s", 0))
        {
            validCommand = true;
            setGen0Values(1024, 0, 0);
            setGen1Values(1024, 0, 0);
        }

        // Add a command 'ccw' 100% duty cycle
        else if (isCommand(&data, "cw", 0) | isCommand(&data, "cw", 1))
        {
            validCommand = true;
            angle = (data.fieldCount > 1) ? getFieldInteger(&data, 1) : 0;
            if (data.fieldCount == 2)
            {
                mmArcLength = (absoluteValue(angle - ANGLE_ADJUSTMENT_FACTOR_CW) * PI_OVER_180 * BASE_RADIUS) / (SCALING_FACTOR * SCALING_FACTOR);
                timeForDistanceDesired = ((mmArcLength * 60 * 40e6) / MAX_SPEED);
                TIMER3_TAILR_R = timeForDistanceDesired;
                TIMER3_CTL_R |= TIMER_CTL_TAEN; // Enable timer 3 for desired distance
                setGen0Values(1024, 1023, 0);
                setGen1Values(1024, 1023, 0);
            }
            else
            {
                setGen0Values(1024, 1023, 0);
                setGen1Values(1024, 1023, 0);
            }
        }

        // Add a command 'cw' 100% duty cycle
        else if (isCommand(&data, "ccw", 0) | isCommand(&data, "ccw", 1))
        {
            validCommand = true;
            angle = (data.fieldCount > 1) ? getFieldInteger(&data, 1) : 0;

            if (data.fieldCount == 2)
            {
                mmArcLength = (absoluteValue(angle - ANGLE_ADJUSTMENT_FACTOR_CCW) * PI_OVER_180 * BASE_RADIUS) / (SCALING_FACTOR * SCALING_FACTOR);

                timeForDistanceDesired = ((mmArcLength * 60 * 40e6) / MAX_SPEED);

                TIMER3_TAILR_R = timeForDistanceDesired;
                TIMER3_CTL_R |= TIMER_CTL_TAEN; // Enable timer 3 for desired distance

                setGen0Values(1024, 0, 1023);
                setGen1Values(1024, 0, 1023);
            }
            else
            {
                setGen0Values(1024, 0, 1023);
                setGen1Values(1024, 0, 1023);
            }
        }

        // Command 'DRV' to turn off/on the DRV8833
        else if (isCommand(&data, "drv", 1))
        {
            validCommand = true;

            DRV_Val = getFieldInteger(&data, 1);

            DRV_Val == 1 ? putsUart0("Turning on DRV8833\n") : putsUart0("Turning off DRV8833\n");

            DRV_SLEEP = DRV_Val;
        }

        validCommand ? putsUart0("Valid command!\n\n") : putsUart0("Invalid command!\n\n");
        putsUart0("********************");

        // TODO: Ensure consistency with uppercase and lower case commands

        clearStruct(&data);
    }
}
