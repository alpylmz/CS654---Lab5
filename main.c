/****************************************************/
/*                                                  */
/*   CS-454/654 Embedded Systems Development        */
/*   Instructor: Renato Mancuso <rmancuso@bu.edu>   */
/*   Boston University                              */
/*                                                  */
/*   Description: Lab5 Joystick ADC, PWM, and Servo   */
/*                Motors using helper functions     */
/*                                                  */
/****************************************************/

#include <p33Fxxxx.h>
// do not change the order of the following 3 definitions
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

// Initial configuration by EE
#include "types.h"
// Primary (XT, HS, EC) Oscillator with PLL
_FOSCSEL(FNOSC_PRIPLL);
// OSC2 Pin Function: OSC2 is Clock Output - Primary Oscillator Mode: XT Crystal
_FOSC(OSCIOFNC_OFF & POSCMD_XT); 
// Watchdog Timer Enabled/disabled by user software
_FWDT(FWDTEN_OFF);
// Disable Code Protection
_FGS(GCP_OFF);  

// --- Constants ---
#define DEBOUNCE_COUNT 30
#define ADC_CHANNEL_X 0x04 // AN4
#define ADC_CHANNEL_Y 0x05 // AN5
#define MOTOR_CHANNEL_X 1  // OC7 (RD6) - Verify pin mapping
#define MOTOR_CHANNEL_Y 0  // OC8 (RD7) - Verify pin mapping
#define MOTOR_MIN_DUTY_US      900
#define MOTOR_CENTER_DUTY_US   1500
#define MOTOR_MAX_DUTY_US      2100

// Calibration values (global so they can be used by helper functions)
int min_x = 0, max_x = 0, min_y = 0, max_y = 0;

// Timer2 Interrupt (if needed)
void __attribute__((__interrupt__)) _T2Interrupt(void){    
    IFS0bits.T2IF = 0; // clear timer 2 interrupt flag
}

// Initializes Joystick ADC peripheral
void init_joystick_adc(){
    CLEARBIT(AD1CON1bits.ADON);    // disable ADC

    // Configure AN4 (RB4) and AN5 (RB5) as analog inputs
    SETBIT(TRISBbits.TRISB4); // AN4 for X
    SETBIT(TRISBbits.TRISB5); // AN5 for Y
    AD1PCFGbits.PCFG4 = 0;    // Set AN4 to analog mode
    AD1PCFGbits.PCFG5 = 0;    // Set AN5 to analog mode

    // Configure AD1CON1
    SETBIT(AD1CON1bits.AD12B);   // Set 12-bit Operation Mode
    AD1CON1bits.FORM = 0;        // Set integer output    
    AD1CON1bits.SSRC = 0x7;      // Set automatic conversion (internal counter ends sampling)

    // Configure AD1CON2
    AD1CON2 = 0; // Not using scanning sampling    

    // Configure AD1CON3
    CLEARBIT(AD1CON3bits.ADRC);  // Internal clock source (Fcy)
    AD1CON3bits.SAMC = 0x1F;     // Auto-sample time = 31 Tad
    AD1CON3bits.ADCS = 0x2;      // Tad = 3 * Tcy 

    // Set initial channel to X
    AD1CHS0bits.CH0SA = ADC_CHANNEL_X; 

    SETBIT(AD1CON1bits.ADON);    // Enable ADC
}

// Sets the active ADC channel (X or Y)
void set_joystick_channel(int channel_an) {
    CLEARBIT(AD1CON1bits.ADON);  
    AD1CHS0bits.CH0SA = channel_an;  // Set the analog input channel (AN4 or AN5)
    __delay_us(20);
    SETBIT(AD1CON1bits.ADON);  
}

// Function wrap for displaying changes during calibration.
// The state determines the row to update:
// 0 = x min, 1 = x max, 2 = y min, 3 = y max.
void update_calibration_display(int state, int current_value) {
    lcd_locate(10, state);
    if (current_value < 1000) {
         lcd_printf(" %d ", current_value);
    } else {
         lcd_printf("%d ", current_value); 
    }
}

// Captures a calibration value for the given state.
// This function takes one sample and sets the corresponding calibration variable.
// It also updates the display to “freeze” the value.
void capture_calibration_value(int state) {
    int sample = sample_joystick(); // Capture the current sample
    switch (state) {
        case 0:
            min_x = sample;
            update_calibration_display(state, min_x);
            break;
        case 1: 
            max_x = sample;
            update_calibration_display(state, max_x);
            // Switch to Y channel for the next calibration state
            set_joystick_channel(ADC_CHANNEL_Y);
            break;
        case 2:
            min_y = sample;
            update_calibration_display(state, min_y);
            break;
        case 3: 
            max_y = sample;
            update_calibration_display(state, max_y);
            // Switch back to X channel for motor control
            set_joystick_channel(ADC_CHANNEL_X);
            break;
    }
    __delay_ms(10);
}

// Updates and controls the motor for the specified axis (0: x, 1: y)
void update_and_control_motor(int motor) {
    if(motor == 0){
        int raw_x = sample_joystick();  
        uint16_t duty_x_us = (uint16_t)mapValue(raw_x, min_x, max_x, 
                                            MOTOR_MIN_DUTY_US, MOTOR_MAX_DUTY_US);
        lcd_locate(0, 5); 
        lcd_printf("X Raw: %4d", raw_x); // Display raw value
        lcd_locate(0, 6); 
        lcd_printf("X Duty: %4u ", duty_x_us); // Display calculated duty
        motor_set_duty(MOTOR_CHANNEL_X, duty_x_us);
    } else if(motor == 1){
        int raw_y = sample_joystick(); // Assumes Y channel is selected
        uint16_t duty_y_us = (uint16_t)mapValue(raw_y, min_y, max_y, 
                                            MOTOR_MIN_DUTY_US, MOTOR_MAX_DUTY_US);                                 
        lcd_locate(9, 5); 
        lcd_printf("Y Raw: %4d", raw_y); // Display raw value
        lcd_locate(9, 6); 
        lcd_printf("Y Duty: %4u ", duty_y_us); // Display calculated duty
        motor_set_duty(MOTOR_CHANNEL_Y, duty_y_us); 
    }
}

int main(){
    // LCD Initialization Sequence 
    __C30_UART = 1;	
    lcd_initialize();
    lcd_clear();
    
    // Initialize both motors (motor_init expects 0 for one axis, 1 for the other)
    motor_init(0);
    motor_init(1);
    
    // Joystick Button Initialization (Set as Inputs)
    SETBIT(TRISEbits.TRISE8);
    SETBIT(TRISDbits.TRISD10);
    // For button 0 digital mode
    SETBIT(AD1PCFGHbits.PCFG20);
	
    init_joystick_adc();
    
    // Display prompts for calibration values
    lcd_locate(0, 0); lcd_printf("x min: ? ");
    lcd_locate(0, 1); lcd_printf("x max: ? ");
    lcd_locate(0, 2); lcd_printf("y min: ? ");
    lcd_locate(0, 3); lcd_printf("y max: ? ");
    
    // Set initial motor positions (center)
    motor_set_duty(MOTOR_CHANNEL_X, MOTOR_CENTER_DUTY_US);
    motor_set_duty(MOTOR_CHANNEL_Y, MOTOR_CENTER_DUTY_US);
    
    // Calibration state: 0 = x min, 1 = x max, 2 = y min, 3 = y max.
    // Once calibration_state reaches 4, the system enters motor control mode.
    int calibration_state = 0;
    
    // Variables for button debouncing
    unsigned char pressed_count = 0;
    unsigned char released_count = 0;
    int is_pressed = 0;
    
    while(1){
        // Calibration phase: continuously sample and display until button press freezes value.
        if(calibration_state < 4) {
            int current_sample = sample_joystick();
            update_calibration_display(calibration_state, current_sample);
            
            // Debounce the joystick trigger button (assumed active low: JOYSTICK1 == 0 means pressed)
            if(JOYSTICK1 == 0){
                pressed_count++;
                if(pressed_count > DEBOUNCE_COUNT && is_pressed == 0){
                    // Capture and freeze the current value for the current calibration state
                    capture_calibration_value(calibration_state);
                    calibration_state++;
                    pressed_count = 0;
                    released_count = 0;
                    is_pressed = 1;
                }
            } else {
                released_count++;
                if(released_count > DEBOUNCE_COUNT){
                    is_pressed = 0;
                    released_count = 0;
                }
            }
        }
        // Motor control phase: after all calibration values are set
        else {
            // Update and control motor for X-axis (motor 0) and Y-axis (motor 1)
            update_and_control_motor(0);
            update_and_control_motor(1);
        }
    }
    
    return 0;
}
