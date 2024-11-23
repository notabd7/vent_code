// this code starts the open close cycle once it receives an open command from xbee
// stops the cycle when receives a close command, this is for the green vent but the servo position doesnt seal the vent very well


#include <msp430.h>
// Configure pins (all in P1)

//pin2 will control the LED
#define ON_LED BIT2
//pin 5
#define RESISTOR BIT5
//pin 6
#define XBEE3  BIT6
//pin7
#define SERVO  BIT7

//servo
#define CLOSED  0
#define OPEN 1
//const unsigned char OPEN = 1;
#define FALSE 0
#define TRUE 1
#define SERVO_ANGLE_BIG 1000  //input desired pulse width here from roughly 1700-400 to change angle
#define SERVO_ANGLE_SMALL 420 //input desired pulse width here from roughly 1700-400 to change angle
#define CLOSE_TIME_DELAY 500  // 10 seconds = 500 periods (each period is 20ms)
#define OPEN_TIME_DELAY 250 // 5 SEC
// ---------------------------------------------------------
// =========================================================


// Function declarations
void initTimer_A(void);
void delay_overflows(int n);
void delay_SEC(int sec);



//Global Variables
unsigned char servo_status;
unsigned int temp;
unsigned long overflow_counter; // Used by delay62MS function & timer
unsigned long pulse_count = 0;
unsigned char cycle_enabled = FALSE; // controls the initiation of the 10 sec cycle

/**
 * main.c
 */
int main(void){
  WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    servo_status = CLOSED;


    // Configure output pins
    P1DIR |= ON_LED; //P1.2 LED  enable output
    P1OUT |= ON_LED; //LED on
    P1DIR |= SERVO; // P1.SERVO enable output
    P1OUT &=~(SERVO);//Start off
    P1DIR |= RESISTOR; //P1.5 RESISTOR enable output
    P1OUT |=(RESISTOR);//Start on


    // Configure input pin


// Watch this pin for changes (like watching for the doorbell)
// When the signal changes from low to high (doorbell pressed), stop what you're doing and run this code:
    P1DIR &= ~(XBEE3);//P1.6(XBEE3) enable input
    P1IES &=~(XBEE3); //Start looking for low to high edge
    P1IFG &=~ (XBEE3); //Clear any previous interrupts
    P1IE |=(XBEE3);  // Interrupt Enable
// calls Servo_change
    //MCLK = SMCLK = 1MHz
    DCOCTL =0;
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;

    initTimer_A();
    __enable_interrupt();

    while(TRUE){

    };

}
//------------------Functions-----------------------------------
void initTimer_A(void){
    //Timer0_A3 Config
    TACCTL0 |=CCIE; //Enable CCRO interrupt
    TACCTL1 |=CCIE; //Enable CCR1 interrupt
    TACCR1 = 20000; //CCR1 intial value (Period) Sets the period to 20 milliseconds
// Servo motors need a signal every 20ms to work properly
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
//#pragma vector = TIMER0_A0_VECTOR
//__interrupt void Timer_A_CCR0_ISR(void) {
//   P1OUT &=~(SERVO);        //set SERVO low
//   TACCR0 = TACCR1+SERVO_ANGLE_SMALL+(SERVO_ANGLE_BIG*servo_status);    //Adds one period // responsible for when to turn off ( in terms of pwm signalling)
//}


#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A_CCR0_ISR(void) {
   P1OUT &=~(SERVO);        //set SERVO low

   // Count pulses when in OPEN state
   if (cycle_enabled) {
       pulse_count++;

       // Flash LED every second (50 pulses = 1 second)
       if(pulse_count % 50 == 0) {
           P1OUT ^= ON_LED;  // Toggle LED
       }
       if (servo_status == OPEN) {
           if(pulse_count >= OPEN_TIME_DELAY) {
                      servo_status = CLOSED;
                      pulse_count = 0;
           }
       } else {
            if(pulse_count >= CLOSE_TIME_DELAY) {
                       servo_status = OPEN;
                       pulse_count = 0;
                  }
              }

       }
//       // After 500 pulses (10 seconds), change PWM to CLOSED position
//       if(pulse_count >= FIVE_SEC_DELAY) {
//           if(servo_status == OPEN) {
//                       servo_status = CLOSED;
//                   } else {
//                       servo_status = OPEN;
//                   }
//           pulse_count = 0;
//           //P1OUT |= ON_LED;  // Turn LED solid ON when closing
//       }
//   }


   TACCR0 = TACCR1+SERVO_ANGLE_SMALL+(SERVO_ANGLE_BIG*servo_status);
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
        cycle_enabled = TRUE; // enable cycle on first open command
        pulse_count = 0;
    }
    else{                   //falling edge trigger
        P1IES &=~(XBEE3);   //start looking for rising edge
        servo_status = CLOSED;
        cycle_enabled =  FALSE;
    }
    P1IFG &=~(XBEE3);
}





