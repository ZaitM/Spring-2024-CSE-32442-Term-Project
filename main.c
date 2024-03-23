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
#include "lab_4_zait.h"

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

// Maximum number of chars that can be accepted from the user
// and the structure for holding UI info
#define MAX_CHARS 80
#define MAX_FIELDS 5



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
    // Enable clock for timer
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R5 | SYSCTL_RCGCTIMER_R4 | SYSCTL_RCGCTIMER_R3 | SYSCTL_RCGCTIMER_R2;
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

    // Input Edge-Count and Capture Mode Timer for PC4
    // There are 20 holes in wheel
    // Will use this to determine speed of wheel
    // Will determine the number of clock cycles for wheel to make a full rotation
    TIMER3_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER3_CTL_R |= TIMER_CTL_TAEVENT_NEG; // Capture on falling edge
    TIMER3_CFG_R |= TIMER_CFG_32_BIT_TIMER;
    TIMER3_TAMR_R |= TIMER_TAMR_TAMR_CAP; // Set to capture mode
    TIMER3_TAMR_R &= ~TIMER_TAMR_TACMR;   // Edge-Count mode
    TIMER3_TAMR_R |= TIMER_TAMR_TACDIR;   // Count up
    // TIMER3_TAILR_R = 20;                  // Sets the upper bound for the timeout event  
    // GPTMTnMATCHR interrupt is generated when the timer value is equal to the value in this register
    TIMER3_TAMATCHR_R = 20;
    TIMER3_IMR_R = TIMER_IMR_CAMIM; // Enable timeout interrupt
    NVIC_EN1_R = 1 << (INT_TIMER3A - 16 - 32);

    // 25 ms one-shot timer configuration for PE0
    TIMER4_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER4_CFG_R |= TIMER_CFG_32_BIT_TIMER;
    TIMER4_TAMR_R |= TIMER_TAMR_TACDIR | TIMER_TAMR_TAMR_1_SHOT;
    TIMER4_TAILR_R = 1000000;
    TIMER4_IMR_R = TIMER_IMR_TATOIM;
    NVIC_EN2_R = 1 << (INT_TIMER4A - 16 - 64);
}

volatile uint32_t startTime = 0, endTime = 0;
volatile bool rotationComplete = false;

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

void timer3ISR(void)
{
    if(     )
    RED_LED ^= 1;
    if(startTime == 0)
    {
        startTime = TIMER3_TAV_R;
    }
    else
    {
        endTime = TIMER3_TAV_R;
        rotationComplete = true;
        TIMER3_ICR_R = TIMER_ICR_CAMCINT;
        TIMER3_CTL_R &= ~TIMER_CTL_TAEN;
    }
}

void photoISR2(void)
{
    if (GPIO_PORTE_RIS_R & PHOTO_TR_MASK_2)
    {
        // BLUE_LED ^= 1;
        GPIO_PORTE_IM_R &= ~PHOTO_TR_MASK_2; // Mask by clearing IME field. Interrupt Disabled
        /* 3/17/24 Have to turn off edge trigger input by the IM register */

        TIMER4_CTL_R |= TIMER_CTL_TAEN;      // Enable timer
        TIMER3_CTL_R |= TIMER_CTL_TAEN;    // Enable timer 3 for edge count and capture mode

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
    int32_t val1 = 0;

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

        // Validate command and minimum num of args provide by user
        // Add a command 'foward' 100% duty cycle
        if (isCommand(&data, "foward", 0))
        {
            foo = true;
            // setGen0Values(1024, 1023, 0);
            setGen1Values(1024, 0, 1023);
        }

        // Add a  command 'reverse' 100% duty cycle
        if (isCommand(&data, "reverse", 0))
        {
            foo = true;
            setGen0Values(1024, 0, 1023);
            setGen1Values(1024, 1023, 0);
        }

        // Add a command 'stop' 0% duty cycle
        if (isCommand(&data, "stop", 0))
        {
            foo = true;
            setGen0Values(1024, 0, 0);
            setGen1Values(1024, 0, 0);
        }

        // Add a command 'ccw' 100% duty cycle
        if (isCommand(&data, "ccw", 0))
        {
            foo = true;
            setGen0Values(1024, 0, 1023);
            setGen1Values(1024, 0, 1023);
        }

        // Add a command 'cw' 100% duty cycle
        if (isCommand(&data, "cw", 0))
        {
            foo = true;
            setGen0Values(1024, 1023, 0);
            setGen1Values(1024, 1023, 0);
        }

        // Command 'DRV' to turn off/on the DRV8833
        if (isCommand(&data, "drv", 1))
        {
            foo = true;
            val1 = getFieldInteger(&data, 1);
            val1 == 1 ? putsUart0("Turning on DRV8833\n") : putsUart0("Turning off DRV8833\n");
            DRV_SLEEP = val1;
        }

        if (isCommand(&data, "freq", 1))
        {
            foo = true;
            val1 = getFieldInteger(&data, 1);
            // Currently only adjusts for 'forward' command
            setGen0Values(val1, val1 - 1, 0);
            setGen1Values(val1, 0, val1 - 1);
        }
        if(rotationComplete)
        {
            rotationComplete = false;
            uint32_t time = endTime - startTime;
            snprintf(data.buffer, MAX_CHARS, "Time: %d\n", time);
            putsUart0(data.buffer);
            startTime = 0;
            endTime = 0;
        }
        foo ? putsUart0("Valid command!\n\n") : putsUart0("Invalid command!\n\n");
        putsUart0("********************");

        // TODO: Ensure consistency with uppercase and lower case commands

        clearStruct(&data);
    }
}
