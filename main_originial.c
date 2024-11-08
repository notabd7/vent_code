/**
 * -------------------------------------------------
 * BOREALIS Valve Control
 * -------------------------------------------------
 * TARGET DEVICE: MSP430G2230
 *
 * unblocking approach for a timed open
 */


#include <msp430.h>
//// Configure pins (all in P1)
//#define ON_LED BIT2 - Assigns the LED to Pin 1.2 (P1.2) of the MSP430
//#define RESISTOR BIT5 - Assigns the heater resistor control to Pin 1.5 (P1.5)
//#define XBEE3 BIT6 - Assigns the XBee communication pin to Pin 1.6 (P1.6)
//#define SERVO BIT7 - Assigns the servo control to Pin 1.7 (P1.7)
#define ON_LED BIT2
#define RESISTOR BIT5
#define XBEE3  BIT6
#define SERVO  BIT7
//servo
#define CLOSED  1 //0 //initial states for each
#define OPEN  0 //1
//const unsigned char OPEN = 1;
#define FALSE 0
#define TRUE 1
#define SERVO_ANGLE_BIG 650  //input desired pulse width here from roughly 1700-400 to change angle
#define SERVO_ANGLE_SMALL 500 //input desired pulse width here from roughly 1700-400 to change angle
// ---------------------------------------------------------
// =========================================================


// Function declarations
void initTimer_A(void);        // Initializes Timer A for servo PWM control
//PWM (Pulse Width Modulation) is a technique used to control servos by sending pulses of specific widths. Think of it like a digital signal that's either ON or OFF, where the duration of the "ON" time (pulse width) tells the servo what position to move to.
//Here's how it works in this vent system:
//
//The basics:
//
//   _____          _____          _____
//  |     |        |     |        |     |
//  |     |________|     |________|     |________
//
//  <-pw->
//  <----period---->
//
//The signal alternates between HIGH (ON) and LOW (OFF)
//pw = pulse width (controlled by SERVO_ANGLE_SMALL + SERVO_ANGLE_BIG)
//period = total time of one cycle (20000 in this code)


void delay_overflows(int n);   // Creates delays based on timer overflows



//Global Variables
unsigned char servo_status;    // Tracks if servo is OPEN (1) or CLOSED (0)
unsigned int temp;            // Temporary variable used in timer interrupts to check interrupt source
unsigned long overflow_counter; // Used by delay62MS function & timer
unsigned long open_start_time = 0;
#define VENT_OPEN_DURATION 150  // 10 seconds = 15 overflows/sec * 10
volatile unsigned char auto_close = FALSE;  // Flag to track if we're in auto-close mode

/**
 * main.c
 */
int main(void){
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    servo_status= CLOSED;  // sets initial servo position to closed


    // Configure output pins
    P1DIR |= ON_LED; //P1.2 LED  enable output
    P1OUT |= ON_LED; //LED on
    P1DIR |= SERVO; // P1.SERVO enable output
    P1OUT &=~(SERVO);//Start off
    P1DIR |= RESISTOR; //P1.5 RESISTOR enable output
    P1OUT |=(RESISTOR);//Start on


    // Configure input pin
    P1DIR &= ~(XBEE3);//P1.6(XBEE3) enable input
    P1IES &=~(XBEE3); //Start looking for low to high edge
    P1IFG &=~ (XBEE3); // clear interrupt flag
    P1IE |=(XBEE3);  // Interrupt Enable

    //MCLK = SMCLK = 1MHz
    DCOCTL =0;
    BCSCTL1 = CALBC1_1MHZ; //setting up main clock
    DCOCTL = CALDCO_1MHZ; //sets up clock to 1MHz

    initTimer_A(); //timer for pwm
    __enable_interrupt();
    while(TRUE){}; //infinite loop everything else happens in interrupts

}
//------------------Functions-----------------------------------
void initTimer_A(void){
    //Timer0_A3 Config
    TACCTL0 |=CCIE; //Enable CCRO interrupt
    TACCTL1 |=CCIE; //Enable CCR1 interrupt
    TACCR1 = 20000; //CCR1 intial value (Period)
    TACCR0 = TACCR1+SERVO_ANGLE_SMALL+(SERVO_ANGLE_BIG*servo_status); // CCR0 initial value (Pulse Width)
    TACTL = TASSEL_2 + ID_0 + MC_2; //Use SMCLK, SMLK/1, Counting Continous Mode
}
//Delays the inputted overflow
void delay_overflows(int n){
    unsigned long desired_overflows= overflow_counter+n; //cool maths for desired delay in 65 ms intervals
    while (overflow_counter < desired_overflows);//do nothing until desired delay in 65 ms intervals
}
void delay_SEC(int sec){
    unsigned long desired_overflows = overflow_counter+15*sec; //cool maths for desired delay in minutes
    while (overflow_counter < desired_overflows);//do nothing until desired delay in minutes
}

void Resist(void){

}

//------------------------Interrupt Service Routines---------------------
//Timer ISR
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A_CCR0_ISR(void) {
   P1OUT &=~(SERVO);        //set SERVO low

   // Check if we need to auto-close
      if(auto_close && (overflow_counter - open_start_time) >= VENT_OPEN_DURATION) {
          servo_status = CLOSED;
          auto_close = FALSE;
      }

      //

   TACCR0 = TACCR1+SERVO_ANGLE_SMALL+(SERVO_ANGLE_BIG*servo_status);    //Add one Period
}
//Servo ISR
#pragma vector = TIMER0_A1_VECTOR
__interrupt void Timer_A_CCR1_ISR(void){
    temp = TAIV;
    if (temp == TAIV_TACCR1){
        P1OUT |=(SERVO);    //set SERVO High
        TACCR1 +=20000;     //add one Period
    }
}

//PWM pin to SERVO pin ISR
#pragma vector = PORT1_VECTOR
__interrupt void Servo_change(void){
    if ((P1IES & XBEE3) == 0){  //rising edge trigger
        P1IES |=(XBEE3);        //start looking for falling edge
        servo_status = OPEN;

        //
        open_start_time = overflow_counter; //record when we open the vent
        auto_close = TRUE; //enable autoclose

    }
    else{                   //falling edge trigger
        P1IES &=~(XBEE3);   //start looking for rising edge
        servo_status = CLOSED;
        auto_close = FALSE;
    }
    P1IFG &=~(XBEE3);
}




