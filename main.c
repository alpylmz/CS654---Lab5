/****************************************************/
/*                                                  */
/*   CS-454/654 Embedded Systems Development        */
/*   Instructor: Renato Mancuso <rmancuso@bu.edu>   */
/*   Boston University                              */
/*                                                  */
/*   Description: simple HelloWorld application     */
/*                for Amazing Ball platform         */
/*                                                  */
/****************************************************/

#include <p33Fxxxx.h>
//do not change the order of the following 3 definitions
#define FCY 12800000UL 
#include <stdio.h>
#include <libpic30.h>
#include <uart.h>

#include "lcd.h"
#include "led.h"
#include "joystick.h"
#include "types.h"
#include "motor.h"
#include "timer.h"



//Initial configuration by EE #include "types.h"
// Primary (XT, HS, EC) Oscillator with PLL
_FOSCSEL(FNOSC_PRIPLL);

// OSC2 Pin Function: OSC2 is Clock Output - Primary Oscillator Mode: XT Crystal
_FOSC(OSCIOFNC_OFF & POSCMD_XT); 

// Watchdog Timer Enabled/disabled by user software
_FWDT(FWDTEN_OFF);

// Disable Code Protection
_FGS(GCP_OFF);  

int curr_duty_us_x = 0;
int curr_duty_us_y = 0;


void __attribute__((__interrupt__)) _T2Interrupt(void){
    motor_set_duty(0, curr_duty_us_x);
    
    lcd_locate(6, 6);
    lcd_printf("tt");
    __delay_ms(1);
    
    IFS0bits.T2IF = 0;
}


int main(){
	// LCD Initialization Sequence 
	__C30_UART=1;	
	lcd_initialize();
	lcd_clear();
    
    motor_init(0);
    
    unsigned char counter = 0;
    
    
    CLEARBIT(LED4_TRIS);    
    CLEARBIT(LED1_TRIS);    
    CLEARBIT(LED2_TRIS);
    CLEARBIT(LED3_TRIS);



    
    
    SETBIT(LED4_PORT);
    __delay_ms(100);
    CLEARBIT(LED4_PORT);
    __delay_ms(1000);
    SETBIT(LED4_PORT);
    __delay_ms(100);
    CLEARBIT(LED4_PORT);
    __delay_ms(1000);
    SETBIT(LED4_PORT);
    __delay_ms(100);
    CLEARBIT(LED4_PORT);
    __delay_ms(1000);
    
    SETBIT(TRISEbits.TRISE8);
    SETBIT(TRISDbits.TRISD10);
    // for button 0 digital mode
    SETBIT(AD1PCFGHbits.PCFG20);

    unsigned char prev_counter = 0;
    unsigned char is_pressed = 0;
    unsigned char last_time = 0;
    unsigned char pressed_count = 0;
    unsigned char released_count = 0;
    
    // disable ADC
    CLEARBIT(AD1CON1bits.ADON);    
    CLEARBIT(AD2CON1bits.ADON);

    
    // initialize PIN for joystick X&Y
    SETBIT(TRISBbits.TRISB4);
    SETBIT(TRISBbits.TRISB5); // joystick Y

    
    //CLEARBIT(AD1PCFGHbits.   ); //set AD1 AN4 input pin as analog
    //Configure AD1CON1
    CLEARBIT(AD1CON1bits.AD12B); //set 10b Operation Mode    
    CLEARBIT(AD2CON1bits.AD12B); //set 10b Operation Mode for ADC2

    AD1CON1bits.FORM = 0; //set integer output    
    AD2CON1bits.FORM = 0; //set integer output

    AD1CON1bits.SSRC = 0x7; //set automatic conversion    
    AD2CON1bits.SSRC = 0x7; //set automatic conversion

    //Configure AD1CON2
    AD1CON2 = 0; //not using scanning sampling    
    AD2CON2 = 0; //not using scanning sampling

    //Configure AD1CON3
    CLEARBIT(AD1CON3bits.ADRC); //internal clock source
    AD1CON3bits.SAMC = 0x1F; //sample-to-conversion clock = 31Tad
    AD1CON3bits.ADCS = 0x2; //Tad = 3Tcy (Time cycles)
    CLEARBIT(AD2CON3bits.ADRC); //internal clock source
    AD2CON3bits.SAMC = 0x1F; //sample-to-conversion clock = 31Tad
    AD2CON3bits.ADCS = 0x2; //Tad = 3Tcy (Time cycles)
    //Leave AD1CON4 at its default value
    //enable ADC
    SETBIT(AD1CON1bits.ADON);    
    SETBIT(AD2CON1bits.ADON);

    
    AD1CHS0bits.CH0SA = 0x04;    
    AD2CHS0bits.CH0SA = 0x05;

    
    char curr_text[500] = "";
    int curr_state = 0;
    
    int min_x, max_x, min_y, max_y;
    double X, Y;
    min_x = max_x = min_y = max_y = X = 0;
    
    lcd_locate(0, 0);
    lcd_printf("x min: ? ");
    
    lcd_locate(0, 1);
    lcd_printf("x max: ? ");
    
    lcd_locate(0, 2);
    lcd_printf("y min: ? ");
    
    lcd_locate(0, 3);
    lcd_printf("y max: ? ");
    
    
    int duty_us = 150000;
    motor_set_duty(0, duty_us);
    
    
	while(1){

        if(prev_counter != counter){
            //duty_us = ADC1BUF0 % 1024;
            //curr_duty_us_x = duty_us;
            //motor_set_duty(0, duty_us);
            //lcd_locate(0,3);
            //lcd_printf("Counter: ");
            //lcd_printf("%d, 0x%x", counter,counter);
            
            if(curr_state == 0){
                SETBIT(AD1CON1bits.SAMP);
                while(!AD1CON1bits.DONE);
                CLEARBIT(AD1CON1bits.DONE);
                //ADC1BUFO includes the sample
                
                min_x = ADC1BUF0 % 1024;
                
                lcd_locate(10,0);
                lcd_printf(" %d", min_x);
                __delay_ms(1);
            }else if(curr_state == 1){
                SETBIT(AD1CON1bits.SAMP);
                while(!AD1CON1bits.DONE);
                CLEARBIT(AD1CON1bits.DONE);
                //ADC1BUFO includes the sample
                
                max_x = ADC1BUF0 % 1024;
                
                lcd_locate(10,1);
                lcd_printf(" %d", max_x);
                __delay_ms(1);
            }else if(curr_state == 2){
                SETBIT(AD2CON1bits.SAMP);
                while(!AD2CON1bits.DONE);
                CLEARBIT(AD2CON1bits.DONE);
                //ADC1BUFO includes the sample
                
                min_y = ADC2BUF0 % 1024;
                
                lcd_locate(10,2);
                lcd_printf(" %d", min_y);
                __delay_ms(1);
            }else if(curr_state == 3){
                SETBIT(AD2CON1bits.SAMP);
                while(!AD2CON1bits.DONE);
                CLEARBIT(AD2CON1bits.DONE);
                //ADC1BUFO includes the sample
                
                max_y = ADC2BUF0 % 1024;
                
                lcd_locate(10,3);
                lcd_printf(" %d", max_y);
                __delay_ms(1);
            }

            curr_state++;
            prev_counter = counter;
        }
        
        
		if (JOYSTICK1 == 0)
        {
            pressed_count++;
            
            if(pressed_count > 30 && is_pressed == 0){
                counter++;
                is_pressed = 1;
                pressed_count = 0;
                curr_duty_us_x += 1000;

            }
          
            //__delay_ms(30);
           
            SETBIT(LED1_PORT);
        }
        else{
            released_count++;
            if(released_count > 30){
                is_pressed = 0;
                released_count = 0;
                CLEARBIT(LED1_PORT);
            }
        }
        if (JOYSTICK2 == 0)
        {
            SETBIT(LED2_PORT);
        }
        else{
            CLEARBIT(LED2_PORT);
        }
        if(JOYSTICK1 != JOYSTICK2){
            SETBIT(LED3_PORT);
        }
        else{
            CLEARBIT(LED3_PORT);
        }
        
        
        if(curr_state == 0){
            // sampling min x
            SETBIT(AD1CON1bits.SAMP);
            while(!AD1CON1bits.DONE);
            CLEARBIT(AD1CON1bits.DONE);
            //ADC1BUFO includes the sample

            unsigned short adc1res = ADC1BUF0;

            lcd_locate(10,0);
            if(adc1res % 1024 < 1000){
                lcd_printf(" %d", adc1res % 1024);    
            }
            else{
                lcd_printf("%d", adc1res % 1024);    
            }
            __delay_ms(1);
        }
        
        if(curr_state == 1){
            // sampling min x
            SETBIT(AD1CON1bits.SAMP);
            while(!AD1CON1bits.DONE);
            CLEARBIT(AD1CON1bits.DONE);
            //ADC1BUFO includes the sample

            unsigned short adc1res = ADC1BUF0;

            lcd_locate(10,1);
            if(adc1res % 1024 < 1000){
                lcd_printf(" %d", adc1res % 1024);    
            }
            else{
                lcd_printf("%d", adc1res % 1024);    
            }
            __delay_ms(1);
        }
        
        if(curr_state == 2){
            // sampling min y
            SETBIT(AD2CON1bits.SAMP);
            while(!AD2CON1bits.DONE);
            CLEARBIT(AD2CON1bits.DONE);
            //ADC1BUFO includes the sample

            unsigned short adc2res = ADC2BUF0;

            lcd_locate(10,2);
            if(adc2res % 1024 < 1000){
                lcd_printf(" %d", adc2res % 1024);    
            }
            else{
                lcd_printf("%d", adc2res % 1024);    
            }
            __delay_ms(1);
        }
        
        if(curr_state == 3){
            // sampling max y
            SETBIT(AD2CON1bits.SAMP);
            while(!AD2CON1bits.DONE);
            CLEARBIT(AD2CON1bits.DONE);
            //ADC1BUFO includes the sample

            unsigned short adc2res = ADC2BUF0; 
                
            lcd_locate(10,3);
            if(adc2res % 1024 < 1000){
                lcd_printf(" %d", adc2res % 1024);    
            }
            else{
                lcd_printf("%d", adc2res % 1024);    
            }
            __delay_ms(1);
        }
        
        
        if(curr_state == 4){
            SETBIT(AD1CON1bits.SAMP);
            while(!AD1CON1bits.DONE);
            CLEARBIT(AD1CON1bits.DONE);
            
            int interval = max_x - min_x;
            X = ADC1BUF0 - min_x;
            X = 90000 + X / interval * (120000);
            
            lcd_locate(10,6);
            lcd_printf("%.2f", X);    
            motor_set_duty(0, X);
            
            __delay_ms(1);
        }
        
        
        
        
        
        /*
        SETBIT(AD1CON1bits.SAMP);
        while(!AD1CON1bits.DONE);
        CLEARBIT(AD1CON1bits.DONE);
        //ADC1BUFO includes the sample
        
        unsigned short adc1res = ADC1BUF0;

        lcd_locate(0,0);
        if(adc1res % 1024 < 1000){
            lcd_printf(" %d", adc1res % 1024);    
        }
        else{
            lcd_printf("%d", adc1res % 1024);    
        }
        __delay_us(1000);
        */

        
	}
    
    return 0;
}
    
