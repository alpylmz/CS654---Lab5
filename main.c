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



//Initial configuration by EE #include "types.h"
// Primary (XT, HS, EC) Oscillator with PLL
_FOSCSEL(FNOSC_PRIPLL);

// OSC2 Pin Function: OSC2 is Clock Output - Primary Oscillator Mode: XT Crystal
_FOSC(OSCIOFNC_OFF & POSCMD_XT); 

// Watchdog Timer Enabled/disabled by user software
_FWDT(FWDTEN_OFF);

// Disable Code Protection
_FGS(GCP_OFF);  


int main(){
	// LCD Initialization Sequence 
	__C30_UART=1;	
	lcd_initialize();
	lcd_clear();
    
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
    
    // initialize PIN for joystick X
    SETBIT(TRISBbits.TRISB4);
    
    //CLEARBIT(AD1PCFGHbits.   ); //set AD1 AN4 input pin as analog
    #include "types.h"
    //Configure AD1CON1
    CLEARBIT(AD1CON1bits.AD12B); //set 10b Operation Mode
    AD1CON1bits.FORM = 0; //set integer output
    AD1CON1bits.SSRC = 0x7; //set automatic conversion
    //Configure AD1CON2
    AD1CON2 = 0; //not using scanning sampling
    //Configure AD1CON3
    CLEARBIT(AD1CON3bits.ADRC); //internal clock source
    AD1CON3bits.SAMC = 0x1F; //sample-to-conversion clock = 31Tad
    AD1CON3bits.ADCS = 0x2; //Tad = 3Tcy (Time cycles)
    //Leave AD1CON4 at its default value
    //enable ADC
    SETBIT(AD1CON1bits.ADON);
    
    AD1CHS0bits.CH0SA = 0x04;
    
    char curr_text[500] = "";
    int curr_state = 0;
    
    int min_x, max_x, min_y, max_y;
    min_x = max_x = min_y = max_y = 0;
    
    lcd_locate(0, 0);
    lcd_printf("x min: ? ");
    
    lcd_locate(0, 1);
    lcd_printf("x max: ? ");
    
    lcd_locate(0, 2);
    lcd_printf("y min: ? ");
    
    lcd_locate(0, 3);
    lcd_printf("y max: ? ");
    
    
    
	while(1){
        
        if(prev_counter != counter){
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
                SETBIT(AD1CON1bits.SAMP);
                while(!AD1CON1bits.DONE);
                CLEARBIT(AD1CON1bits.DONE);
                //ADC1BUFO includes the sample
                
                min_y = ADC1BUF0 % 1024;
                
                lcd_locate(10,2);
                lcd_printf(" %d", min_y);
                __delay_ms(1);
            }else if(curr_state == 3){
                SETBIT(AD1CON1bits.SAMP);
                while(!AD1CON1bits.DONE);
                CLEARBIT(AD1CON1bits.DONE);
                //ADC1BUFO includes the sample
                
                max_y = ADC1BUF0 % 1024;
                
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
            SETBIT(AD1CON1bits.SAMP);
            while(!AD1CON1bits.DONE);
            CLEARBIT(AD1CON1bits.DONE);
            //ADC1BUFO includes the sample

            unsigned short adc1res = ADC1BUF0;

            lcd_locate(10,2);
            if(adc1res % 1024 < 1000){
                lcd_printf(" %d", adc1res % 1024);    
            }
            else{
                lcd_printf("%d", adc1res % 1024);    
            }
            __delay_ms(1);
        }
        
        if(curr_state == 3){
            // sampling max y
            SETBIT(AD1CON1bits.SAMP);
            while(!AD1CON1bits.DONE);
            CLEARBIT(AD1CON1bits.DONE);
            //ADC1BUFO includes the sample

            unsigned short adc1res = ADC1BUF0;

            lcd_locate(10,3);
            if(adc1res % 1024 < 1000){
                lcd_printf(" %d", adc1res % 1024);    
            }
            else{
                lcd_printf("%d", adc1res % 1024);    
            }
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
    
