/*
 * Serial.cpp
 *
 *  Created on: 2020. 7. 23.
 *      Author: tuandang
 *      Serial for MSP430FR2433
 */

#ifdef __cplusplus
extern "C" {
#endif


#include <serial.h>
#include <driverlib.h>
#include "QmathLib.h"
#include <string.h>
#include <stdio.h>

#define Delay(x)             __delay_cycles((unsigned long)(x*16000)); // 1k Ms -> 16M; 1 ms -> 16K

bool rxStringReady = false;
bool rxThreshold = false;
int rIndex = 0, wIndex = 0, rxCount = 0;
uint8_t rx[MAX_STRBUF_SIZE];

//Set uart speed here
//http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
void Serial_init()
{

    // Configure UCA1TXD and UCA1RXD
    P1SEL0 |= BIT4 | BIT5;
    P1SEL1 &= ~(BIT4 | BIT5);

    // Configure UART
    // http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
    EUSCI_A_UART_initParam param = {0};
    param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
#if 0
    //115200
    param.clockPrescalar = 8;
    param.firstModReg = 10;
    param.secondModReg = 247;
#else
    //9600
    param.clockPrescalar = 104;
    param.firstModReg = 2;
    param.secondModReg = 182;
#endif

    param.parity = EUSCI_A_UART_NO_PARITY;
    param.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    param.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    param.uartMode = EUSCI_A_UART_MODE;
    param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable USCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE,
                                 EUSCI_A_UART_RECEIVE_INTERRUPT);      // Enable interrupt
}



void Serial_receiveString(char data) {
#if 0
    static bool rxInProgress = false;
    static unsigned int charCnt = 0;

    if(!rxInProgress){
        if ((data != '\n') ){
            rxInProgress = true;
            charCnt = 0;
            rxString[charCnt] = data;
        }
    }else{ // in progress
        charCnt++;
        if((data != '\n')){
            if (charCnt >= MAX_STRBUF_SIZE){
                rxInProgress = false;
            }else{
                rxString[charCnt] = data;
            }
        }else{
            rxInProgress = false;
            rxString[charCnt] = '\0';
            // String receive complete
            rxStringReady = true;
        }
    }
#endif
}

// Transmits string buffer through EUSCI UART
void Serial_transmitString(char *str)
{
    int i = 0;
    for(i = 0; i < strlen(str); i++)
    {
        if (str[i] != 0)
        {
            // Transmit Character
            while (EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY));
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, str[i]);
        }
        __delay_cycles(16000*10);
    }
}

void Serial_write(uint8_t *buf, int size)
{
    int i = 0;
    for(i = 0; i < size; i++)
    {
        while (EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY));
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, buf[i]);
    }
}

#if 1

// EUSCI interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
            // Read buffer
            //Serial_receiveString(UCA0RXBUF);
            rx[wIndex++] = UCA0RXBUF;
            rxCount++;
            if (wIndex >= MAX_STRBUF_SIZE){
                wIndex = 0;
            }
            __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
            break;
        case USCI_UART_UCTXIFG:{

        }
            break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}

#endif

//Blocking implement
uint8_t Serial_readByte(){

    uint8_t ret = 0;
    //if (rxCount <= 0 ) return 0;
    while (rxCount <= 0){__delay_cycles(10);};
    ret = rx[rIndex];
    rIndex++;
    rxCount--;
    if (rIndex >= MAX_STRBUF_SIZE){
        rIndex = 0;
    }
    return ret;
}

void Serial_printStr(char *str){
    Serial_transmitString(str);
}

void Serial_println(){
    Serial_printStr("\r\n");
}
void Serial_printInt(int32_t val, bool dec){
    char buf[10] = {0};
    if (dec)
        sprintf(buf, "%d", val);
    else
        sprintf(buf, "0x%x", val);
    Serial_printStr(buf);
}

void Serial_printUint(uint32_t val, bool dec){
    char buf[10] = {0};
    uint16_t h = (val >> 16) & 0xFFFF;
    uint16_t l = val & 0xFFFF;
    if (dec){
        sprintf(buf, "%d", val);
        Serial_printStr(buf);
    }
    else{
        sprintf(buf, "0x%x", h);
        Serial_printStr(buf);
        memset(buf, 0, 10);
        sprintf(buf, "%x", l);
        Serial_printStr(buf);
    }

}

static float fval;
static int32_t i1, i2, dec, dec2;
void Serial_printFloat(float val){
    fval = val;
    //char buf[20] = {0};
    dec = (int32_t)fval;
    i1 = (int32_t)(fval*1000);
    i2 = dec*1000;
    dec2 =  i1 - i2;
    //sprintf(buf, "%f", val);
    //Serial_printStr(buf);
    Serial_printInt(dec, true);
    Serial_printStr(".");
    if (10 <= dec2 && dec2 <= 99)
        Serial_printStr("0");
    else if (0 <= dec2 && dec2  <= 9)
        Serial_printStr("00");

    Serial_printInt(dec2, true);
}


#ifdef __cplusplus
}
#endif


