/**
 * -------------------------------------------------
 * BOREALIS Valve Control - Debug Version
 * -------------------------------------------------
 */
#include <msp430.h>
#define ON_LED BIT2
#define RESISTOR BIT5
#define XBEE3  BIT6
#define SERVO  BIT7
#define CLOSED 1
#define OPEN 0
#define FALSE 0
#define TRUE 1
#define SERVO_ANGLE_BIG 650
#define SERVO_ANGLE_SMALL 500

// Function declarations
void initTimer_A(void);
void toggleLED(void);

// Global Variables
unsigned char servo_status;
unsigned int temp;
volatile unsigned long overflow_counter = 0;
volatile unsigned char auto_close_enabled = FALSE;

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    servo_status = OPEN;

    // Configure output pins
    P1DIR |= ON_LED;
    P1OUT |= ON_LED;  // Start with LED on
    P1DIR |= SERVO;
    P1OUT &= ~(SERVO);
    P1DIR |= RESISTOR;
    P1OUT |= (RESISTOR);

    // Configure input pin
    P1DIR &= ~(XBEE3);
    P1IES &= ~(XBEE3);
    P1IFG &= ~(XBEE3);
    P1IE |= (XBEE3);

    // Clock setup
    DCOCTL = 0;
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;

    initTimer_A();
    __enable_interrupt();
    while(TRUE) {}
}

void initTimer_A(void) {
    TACCTL0 |= CCIE;
    TACCTL1 |= CCIE;
    TACCR1 = 20000;  // 20ms period
    TACCR0 = TACCR1 + SERVO_ANGLE_SMALL + (SERVO_ANGLE_BIG * servo_status);
    TACTL = TASSEL_2 + ID_0 + MC_2;  // SMCLK, no divider, continuous mode
}

void toggleLED(void) {
    P1OUT ^= ON_LED;
}

//Timer ISR
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A_CCR0_ISR(void) {
    P1OUT &= ~SERVO;        // set SERVO low

    if (auto_close_enabled) {
        overflow_counter++;

        // Every ~50 overflows should be roughly 1 second
        if (overflow_counter % 50 == 0) {
            toggleLED();  // Toggle LED every "second"

            // After ~10 seconds (500 overflows), close the valve
            if (overflow_counter >= 500) {
                servo_status = CLOSED;
                auto_close_enabled = FALSE;
                overflow_counter = 0;
                P1OUT |= ON_LED;  // LED solid ON when closed
            }
        }
    }

    TACCR0 = TACCR1 + SERVO_ANGLE_SMALL + (SERVO_ANGLE_BIG * servo_status);
}

//Servo ISR
#pragma vector = TIMER0_A1_VECTOR
__interrupt void Timer_A_CCR1_ISR(void) {
    temp = TAIV;
    if (temp == TAIV_TACCR1) {
        P1OUT |= (SERVO);    // set SERVO High
        TACCR1 += 20000;     // add one Period
    }
}

//PWM pin to SERVO pin ISR
#pragma vector = PORT1_VECTOR
__interrupt void Servo_change(void) {
    if ((P1IES & XBEE3) == 0) {  // rising edge trigger
        P1IES |= (XBEE3);        // start looking for falling edge
        servo_status = OPEN;
        P1OUT &= ~ON_LED;        // LED off initially
        overflow_counter = 0;     // Reset counter
        auto_close_enabled = TRUE;
    }
    else {                      // falling edge trigger
        P1IES &= ~(XBEE3);     // start looking for rising edge
        servo_status = CLOSED;
        P1OUT |= ON_LED;        // LED solid on
        auto_close_enabled = FALSE;
    }
    P1IFG &= ~(XBEE3);
}


//the only change that occured was that this code implements a slower
//open and close vent