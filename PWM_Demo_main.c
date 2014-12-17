/******************************************************************************
 *
 * PIC16F88_PWM_Demo
 *
 * Author: Dan Milliken
 *
 * Date: 2014-12-16
 * 
 * Project: PIC16F88_PWM_Demo
 *
 * Description: Demonstrates using the PWM output on the PIC16F88
 * Microcontroller. PWM output on pin 6 (CCP1) will be used to drive an LED.
 * Pin 17 (AN0) will be connected to a potentiometer that will control the PWM
 * duty cycle. The A/D module will read the voltage on AN0, convert it to a
 * numeric value, and use it to set the duty cycle.
 *
 * License: Licensed under the Creative Commons Attribution-ShareAlike 4.0
 * International License (http://creativecommons.org/licenses/by-sa/4.0/)
 *
*******************************************************************************/

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#endif

#include <stdint.h>        /* For uint8_t definition */
#include <stdbool.h>       /* For true/false definition */
#include <stdio.h>         /* C standard IO */

#define _XTAL_FREQ  4000000

#pragma config LVP = OFF        // disable low voltage programming
#pragma config FCMEN = OFF      // disable fail safe clock monitor
#pragma config IESO = OFF       // disable internal/external oscillator switchover
#pragma config BOREN = OFF      // disable brown out reset
#pragma config PWRTE = ON       // enable power up timer
#pragma config WDTE = OFF       // disable watchdog timer
#pragma config FOSC = INTOSCIO  // Internal oscillator

// function prototypes
void mod_init_ad(void);
void mod_init_pwm(void);
void set_pwm(unsigned int value);
void interrupt isr(void);

int main(void)
{
    #assert _XTAL_FREQ == 4000000  // Make sure _XTAL_FREQ is set correctly
    OSCCONbits.IRCF = 0b110;       // Set internal RC oscillator to 4 MHz
    while(!OSCCONbits.IOFS);       // Wait for frequency to stabalize

    mod_init_ad();       // initialize the A/D
    mod_init_pwm();      // initialize the PWM (CCP1)

    INTCONbits.PEIE = 1; // enable peripheral interrupts
    INTCONbits.GIE = 1;  // enable interrupts

    while(1)
    {
        PIR1bits.ADIF = 0;      // Reset the A/D interrupt flag
        PIE1bits.ADIE = 1;      // enable the A/D interrupt
        ADCON0bits.GO_DONE = 1; // start the A/D
        __delay_ms(25);
    }
}

void mod_init_ad(void)
{
    TRISA = 0b00000001;    // Configure RA0 as input (for potentiometer)
    ANSELbits.ANS0 = 1;    // Set RA0 to analog
    ADCON0bits.ADON = 1;   // Turn on the A/D
    ADCON0bits.CHS = 0;    // Use channel AN0 for A/D
    ADCON1bits.ADFM = 1;   // Result right justified
    ADCON1bits.ADCS2 = 0;  // 8 * TOSC = 1.6us
    ADCON0bits.ADCS1 = 0;
    ADCON0bits.ADCS0 = 1;

    return;
}

void mod_init_pwm(void)
{
    PR2 = 255;               // Period register 255

    TRISBbits.TRISB0 = 0;    // Set RB0 (CCP1) as output for PWM

    T2CONbits.T2CKPS1 = 0;   // Prescalar = 1:4
    T2CONbits.T2CKPS0 = 1;

    T2CONbits.TMR2ON = 1;    // Timer2 ON

    CCP1CON |= 0b00001100;   // Set CCP1 mode to PWM

    return;
}

void set_pwm(unsigned int VALUE)
{
    // PWM duty cycle is stored in (CCPR1L:CCP1CON<5:4>)
    CCPR1L = VALUE >> 2;
    CCP1CONbits.CCP1X = VALUE & 0x0002;
    CCP1CONbits.CCP1Y = VALUE & 0x0001;
}

void process_ad(void)
{
    PIE1bits.ADIE = 0;  // disable the A/D interrupt
    PIR1bits.ADIF = 0;  // Reset the A/D interrupt flag
    unsigned int ad_result = ADRESH * 256 + ADRESL;
    // Set the PWM duty cycle to ad_result to give a duty cycle in the range
    // of 0 to 1024 us
    set_pwm(ad_result);
    return;
}

void interrupt isr(void)
{
    // A/D Complete Interrupt Flag bit
    if (1 == PIR1bits.ADIF)
        process_ad();

    return;
}
