/*
 * File:   motor.c
 * Author: team-4a
 *
 * Created on March 31, 2025, 6:45 PM
 */


#include "motor.h"
#include <stdlib.h>
int prev_x = 0;
int prev_y = 0;


void motor_init(uint8_t chan){
    /* Disable Timer 2 */
    CLEARBIT(T2CONbits.TON);
    /* Setup Timer 2 for no interrupts, 20ms period */
    CLEARBIT(T2CONbits.TCS); 
    CLEARBIT(T2CONbits.TGATE);
    TMR2 = 0x00;
    T2CONbits.TCKPS = 0b10;
    CLEARBIT(IEC0bits.T2IE);
    CLEARBIT(IFS0bits.T2IF);
    PR2 = 4000;
    if(chan == 0){ // for X axis
        /*Configure Output Compare 8 (X-axis motor) */
        CLEARBIT(TRISDbits.TRISD7); // Set OC8 = RD7 as output
    }else if(chan == 1){
        CLEARBIT(TRISDbits.TRISD6); // Set OC7 = RD6 as output !!! not sure
    }
    /* Enable Timer 2 */
    SETBIT(T2CONbits.TON);
    OC8CONbits.OCM = 0b110; // Set PWM, no fault mode
    OC7CONbits.OCM = 0b110; // Set PWM, no fault mode
}


double mapValue(double x, double in_min, double in_max, double out_min, double out_max){
    if(in_max == in_min){
        return out_min;
    }
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    
}
/*
Set the duty cycle of the specified motor channel using 
the provided duty in microseconds. Use Timer2 for the duty cycle.
 */
void motor_set_duty(uint8_t chan, uint16_t duty_us){
    double ratio_us = ((double)duty_us)/20000.0;
    double duty_ratio = ratio_us * PR2;
    //duty_ratio = mapValue(duty_ratio, 200, 400, 2000, 3000);
    lcd_locate(1,6);
    lcd_printf("%f", duty_ratio);
    double inverted = (PR2 - duty_ratio);
    //double val = (12.8*1000000) / (64/duty_us);
    //double inverted = PR2 - val;
    // safety check according to lab manual
    //if(duty_us <= 210000 && duty_us >= 90000){
        if(chan == 0){ // for X axis       
            lcd_locate(1,7);
            lcd_printf("%f", inverted);
            if(abs(prev_x - inverted) < 10){
                return;
            }
            OC8R = inverted; // First duty cycle of x ms
            OC8RS = inverted; // Next duty cycle of x ms
            //OC8CONbits.OCM = 0b110; // Set PWM, no fault mode
            prev_x = inverted;
        }else if (chan == 1){
            if(abs(prev_y - inverted) < 10){
                return;
            }
            OC7R = inverted; // First duty cycle of y ms
            OC7RS = inverted; // Next duty cycle of y ms
            //OC7CONbits.OCM = 0b110; // Set PWM, no fault mode
            prev_y = inverted;
        }
    //}
}
