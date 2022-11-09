#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "AD5933.h"
#include "serial.h"
#include <string.h>

/*
 * lib.c
 *
 *  Created on: Aug 14, 2021
 *      Author: tuan.dang@uta.edu
 */

#include "lib.h"
#include "config.h"

void init_system(){
   WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
   init_clock_16MHz();
   init_gpio();
   power(POWER_SENSING, OFF);
   power(POWER_COMM, OFF);
}

void init_gpio()
{
    // Configure GPIO
    P1OUT &= ~(BIT0); //MCU LED
    P1DIR |= (BIT0);

    //Off Serial Port as INPUT
    P1SEL0 &= (~BIT4); //GPIO function
    P1SEL0 &= (~BIT5);
    P1DIR &= ~(BIT4); //Input
    P1DIR &= ~(BIT5); //Input

    //Switch
    P2OUT &= ~(BIT2); //DVDD
    P2DIR |= (BIT2);   //Output

    P3OUT &= ~(BIT0); //AVDD
    P3DIR |= (BIT0) ;  //Output

    P1OUT &= ~(BIT1); //LORA POWER
    P1DIR |= (BIT1) ;  //Output

    P3OUT &= ~(BIT2); //LORA M0: P3.2
    P3DIR |= (BIT2) ;  //Output

    P2OUT &= ~(BIT7); //LORA M1: P2.7
    P2DIR |= (BIT7) ;  //Output

    P2IN &= ~(BIT3); //LORA AUX: P2.3
    P2DIR &= ~(BIT3) ;  //INPUT

    // I2C pins
    P1SEL0 |= BIT2 | BIT3;
    P1SEL1 &= ~(BIT2 | BIT3);

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;
}

void init_clock_16MHz()
{
    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    // Clock System Setup
    __bis_SR_register(SCG0);                           // disable FLL
    CSCTL3 |= SELREF__REFOCLK;               // Set REFO as FLL reference source
    CSCTL0 = 0;                                   // clear DCO and MOD registers
    CSCTL1 &= ~(DCORSEL_7);             // Clear DCO frequency select bits first
    CSCTL1 |= DCORSEL_5;                               // Set DCO = 16MHz
    CSCTL2 = FLLD_0 + 487;                             // DCOCLKDIV = 16MHz
    __delay_cycles(3);
    __bic_SR_register(SCG0);                           // enable FLL
    while (CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1))
        ;         // FLL locked
}


void  power(uint8_t source, uint8_t onOff){
    if (source == POWER_SENSING){
        if (onOff == 1){
            P2OUT |= BIT2;   //P2.2
            P3OUT |= BIT0;   //P3.0
        }
        else{
            P2OUT &= ~BIT2;
            P3OUT &= ~BIT0;
        }
    }else{
        if (onOff == 1)
            P1OUT |= BIT1;   //P1.1
        else
            P1OUT &= ~BIT1;
    }
}


void lora_config(uint8_t mode){
    switch(mode){
    case LORA_MODE_NORMAL:
            P2OUT &= ~BIT7; //M1 2.7
            P3OUT &= ~BIT2; //M0 3.2
        break;

    case LORA_MODE_WAKEUP:
            P2OUT &= ~(BIT7); //M1 2.7
            P3OUT |= BIT2; //M0 3.2
            break;
    case LORA_MODE_PW_SAVING:
            P2OUT |= BIT7; //M1 2.7
            P3OUT &= ~BIT2; //M0 3.2
            break;
    case LORA_MODE_SLEEP:
            P2OUT |= BIT7; //M1 2.7
            P3OUT |= BIT2; //M0 3.2
            break;
    default: break;
    }
}

void lora_read_config(){

   uint8_t txBuf[6];
   uint8_t rxBuf[6];
   Serial_init(UART_9600);
   __enable_interrupt();
   power(POWER_COMM, ON);
   lora_config(LORA_MODE_SLEEP);
   __delay_cycles(16000000);
   txBuf[0] = 0xC1;
   txBuf[1] = 0xC1;
   txBuf[2] = 0xC1;

   Serial_write(txBuf, 3);
   __delay_cycles(TIME_1SEC);
   uint8_t i = 0;
   for(i = 0; i < 6; i++)
       rxBuf[i] = Serial_readByte();
}

void lora_write_config(){

    uint8_t txBuf[6];
   Serial_init(UART_9600);
   __enable_interrupt();
   power(POWER_COMM, ON);
   lora_config(LORA_MODE_SLEEP);
   __delay_cycles(16000000);
   txBuf[0] = 0xC0;
   txBuf[1] = 0x00; //High Target Address
   txBuf[2] = 0x01; //Low  Target Address
   txBuf[3] = 0x1C;
   txBuf[4] = 0x0F;
   txBuf[5] = 0x44;
   Serial_write(txBuf, 6);
   __delay_cycles(16000000);
}

void led_power(uint8_t state){
    if (state == ON){
#if defined(BOARD_IOTREE)
        P1OUT |= BIT0;
#else
        P3OUT |= BIT0;
#endif
    }else {
#if defined(BOARD_IOTREE)
        P1OUT &= ~BIT0;
#else
        P3OUT &= ~BIT0;
#endif
    }
}

void test_io(){
    for(; ; ){
        led_power(ON);
        __delay_cycles(TIME_500MS); //500 ms
        led_power(OFF);
        __delay_cycles(TIME_500MS); //500 ms
    }
}




