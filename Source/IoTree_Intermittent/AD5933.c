/***************************************************************************//**
 *   @file   AD5933.c
 *   @brief  AD5933 Driver.
 *   @author ATofan (alexandru.tofan@analog.com)
 *   Porting to MSP430FR2433 by Tuan Dang (dangthanhtuanit@gmail.com)
 ********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ********************************************************************************
 *   SVN Revision: $WCREV$
 *******************************************************************************/

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/
#include <stdio.h>
#include <math.h>
#include "AD5933.h"
#include <string.h>
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "serial.h"

typedef enum I2C_ModeEnum
{
    IDLE_MODE,
    NACK_MODE,
    TX_REG_ADDRESS_MODE,
    RX_REG_ADDRESS_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    SWITCH_TO_RX_MODE,
    SWITHC_TO_TX_MODE,
    TIMEOUT_MODE
} I2C_Mode;

#define MAX_BUFFER_SIZE     20
#define SLAVE_ADDR         AD5933_I2C_ADDR


static uint8_t ReceiveBuffer[MAX_BUFFER_SIZE] = {0};
static uint8_t RXByteCtr = 0;
static uint8_t ReceiveIndex = 0;
static uint8_t TransmitBuffer[MAX_BUFFER_SIZE] = {0};
static uint8_t TXByteCtr = 0;
static uint8_t TransmitIndex = 0;

static I2C_Mode MasterMode = IDLE_MODE;
static uint8_t TransmitRegAddr = 0;


void initI2C()
{

    UCB0CTLW0 = UCSWRST;                      // Enable SW reset
    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSSEL__SMCLK | UCSYNC; // I2C master mode, SMCLK
    UCB0BRW = 160;                            // fSCL = SMCLK/160 = ~100kHz
    UCB0I2CSA = SLAVE_ADDR;                   // Slave Address
    UCB0CTLW0 &= ~UCSWRST;                   // Clear SW reset, resume operation
    UCB0IE |= UCNACKIE;
}

void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count)
{
    uint8_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++)
    {
        dest[copyIndex] = source[copyIndex];
    }
}

uint8_t I2C_Master_ReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t count)
{
    /* Initialize state machine */
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;
    RXByteCtr = count;
    TXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize slave address and interrupts */
    UCB0I2CSA = dev_addr;
    UCB0IFG &= ~(UCTXIFG + UCRXIFG);       // Clear any pending interrupts
    UCB0IE &= ~UCRXIE;                       // Disable RX interrupt
    UCB0IE |= UCTXIE;                        // Enable TX interrupt

    UCB0CTLW0 |= UCTR + UCTXSTT;             // I2C TX, start condition
    //__bis_SR_register(LPM0_bits + GIE);              // Enter LPM0 w/ interrupts
    __bis_SR_register(GIE);
    while (RXByteCtr > 0) {Delay(1);};
    return ReceiveBuffer[0];
    //return MasterMode;

}

I2C_Mode I2C_Master_WriteReg(uint8_t dev_addr, uint8_t reg_addr,  uint8_t *reg_data, uint8_t count)
{
    //P1OUT |= 0x1;
    /* Initialize state machine */
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;

    //Copy register data to TransmitBuffer
    CopyArray(reg_data, TransmitBuffer, count);

    TXByteCtr = count;
    RXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize slave address and interrupts */
    UCB0I2CSA = dev_addr;
    UCB0IFG &= ~(UCTXIFG + UCRXIFG);       // Clear any pending interrupts
    UCB0IE &= ~UCRXIE;                       // Disable RX interrupt
    UCB0IE |= UCTXIE;                        // Enable TX interrupt

    UCB0CTLW0 |= UCTR + UCTXSTT;             // I2C TX, start condition
   // __bis_SR_register(LPM0_bits + GIE);              // Enter LPM0 w/ interrupts
    __bis_SR_register(GIE);


    while (TXByteCtr > 0) {Delay(1);};
    //P1OUT &= 0xfe;
    return MasterMode;
}



//******************************************************************************
// I2C Interrupt ***************************************************************
//******************************************************************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B0_VECTOR))) USCI_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    //Must read from UCB0RXBUF
    uint8_t rx_val = 0;
    switch (__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG))
    {
    case USCI_NONE:
        break;         // Vector 0: No interrupts
    case USCI_I2C_UCALIFG:
        break;         // Vector 2: ALIFG
    case USCI_I2C_UCNACKIFG:                // Vector 4: NACKIFG
        break;
    case USCI_I2C_UCSTTIFG:
        break;         // Vector 6: STTIFG
    case USCI_I2C_UCSTPIFG:
        break;         // Vector 8: STPIFG
    case USCI_I2C_UCRXIFG3:
        break;         // Vector 10: RXIFG3
    case USCI_I2C_UCTXIFG3:
        break;         // Vector 12: TXIFG3
    case USCI_I2C_UCRXIFG2:
        break;         // Vector 14: RXIFG2
    case USCI_I2C_UCTXIFG2:
        break;         // Vector 16: TXIFG2
    case USCI_I2C_UCRXIFG1:
        break;         // Vector 18: RXIFG1
    case USCI_I2C_UCTXIFG1:
        break;         // Vector 20: TXIFG1
    case USCI_I2C_UCRXIFG0:                 // Vector 22: RXIFG0
        rx_val = UCB0RXBUF;
        if (RXByteCtr)
        {
            ReceiveBuffer[ReceiveIndex++] = rx_val;
            RXByteCtr--;
        }

        if (RXByteCtr == 1)
        {
            UCB0CTLW0 |= UCTXSTP;
        }
        else if (RXByteCtr == 0)
        {
            UCB0IE &= ~UCRXIE;
            MasterMode = IDLE_MODE;
            //__bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
        }
        break;
    case USCI_I2C_UCTXIFG0:                 // Vector 24: TXIFG0
        switch (MasterMode)
        {
        case TX_REG_ADDRESS_MODE:
            UCB0TXBUF = TransmitRegAddr;
            if (RXByteCtr)
                MasterMode = SWITCH_TO_RX_MODE;   // Need to start receiving now
            else
                MasterMode = TX_DATA_MODE; // Continue to transmision with the data in Transmit Buffer
            break;

        case SWITCH_TO_RX_MODE:
            UCB0IE |= UCRXIE;              // Enable RX interrupt
            UCB0IE &= ~UCTXIE;             // Disable TX interrupt
            UCB0CTLW0 &= ~UCTR;            // Switch to receiver
            MasterMode = RX_DATA_MODE;    // State state is to receive data
            UCB0CTLW0 |= UCTXSTT;          // Send repeated start
            if (RXByteCtr == 1)
            {
                //Must send stop since this is the N-1 byte
                while ((UCB0CTLW0 & UCTXSTT))
                    ;
                UCB0CTLW0 |= UCTXSTP;      // Send stop condition
            }
            break;

        case TX_DATA_MODE:
            if (TXByteCtr)
            {
                UCB0TXBUF = TransmitBuffer[TransmitIndex++];
                TXByteCtr--;
            }
            else
            {
                //Done with transmission
                UCB0CTLW0 |= UCTXSTP;     // Send stop condition
                MasterMode = IDLE_MODE;
                UCB0IE &= ~UCTXIE;                       // disable TX interrupt
                //__bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
            }
            break;

        default:
            __no_operation();
            break;
        }
        break;
    default:
        break;
    }
}

void AD5933_Init()
{
    initI2C();
}

#if 1
void AD5933_SetRegisterValue(uint8_t registerAddress, uint32_t registerValue,
                             uint8_t numberOfBytes)
{
    char byte = 0;
    //Low Address -> High Byte
    //High Address -> Low Byte
    for (byte = 0; byte < numberOfBytes; byte++)
    {
        unsigned char sendData = (registerValue >> 8 * (numberOfBytes - 1 - byte)) & 0xFF;
        I2C_Master_WriteReg(SLAVE_ADDR, registerAddress + byte, &sendData, 1);
    }
}

uint32_t AD5933_GetRegisterValue(uint8_t registerAddress, uint8_t numberOfBytes)
{
    uint32_t registerValue = 0;
    uint8_t byte = 0;
    uint8_t rByte = 0;

    //Low Address -> High Byte
    //High Address -> Low Byte
    for (byte = 0; byte < numberOfBytes; byte++)
    {
        unsigned char sendData = registerAddress + byte;
        I2C_Master_WriteReg(SLAVE_ADDR, AD5933_ADDR_POINTER, &sendData, 1);

        rByte = I2C_Master_ReadReg(SLAVE_ADDR, registerAddress + byte, 1);
        registerValue = (registerValue << 8) | rByte;
    }
    //printf("****Reg = 0x%06X \n", registerValue);
    return (registerValue);
}
#else

/******************************************************************************
* @brief Set an AD5933 internal register value.
*
* @param registerAddress - Address of AD5933 register.
*
* @param registerValue - Value of data to be written in the register.
*
* @param numberOfBytes - Number of bytes to be written in the register
*
* @return None.
******************************************************************************/
void AD5933_SetRegisterValue(int registerAddress,
							 int registerValue,
							 char numberOfBytes)
{
	char 			byte         = 0;
	int ret = 0;
	//Low Address -> High Byte
	//High Address -> Low Byte
	for(byte = 0; byte < numberOfBytes; byte++)
	{
		//printf("reg = 0x%02x, val = 0x%02x \n", registerAddress + byte, (registerValue >> 8 * byte) & 0xFF);
		//ret = wiringPiI2CWriteReg8(i2c_fd, registerAddress + byte, (registerValue >> 8 * (numberOfBytes - 1 - byte)) & 0xFF);
		//if (ret == -1)  perror("AD5933_SetRegisterValue: wiringPiI2CWriteReg8 1: \n");
	}
}


/******************************************************************************
* @brief Read an AD5933 internal register value.
*
* @param registerAddress - Address of AD5933 register.
*
* @param numberOfBytes - Number of bytes to be read from the register.
*
* @return Register value.
******************************************************************************/
int AD5933_GetRegisterValue(int registerAddress,
							char numberOfBytes)
{	
	int  registerValue 	= 0;
	char byte 			= 0;
	int ret = 0;
	unsigned char rByte = 0;

	//Low Address -> High Byte
	//High Address -> Low Byte
	for(byte = 0; byte < numberOfBytes; byte++)
	{
		
		//Set pointer register
		//ret = wiringPiI2CWriteReg8(i2c_fd, AD5933_ADDR_POINTER, AD5933_ADDR_POINTER);
		//if (ret == -1)  perror("AD5933_GetRegisterValue: wiringPiI2CWriteReg8 1: \n");
		//ret = wiringPiI2CWriteReg8(i2c_fd, AD5933_ADDR_POINTER, registerAddress + byte);
		//if (ret == -1)  perror("ERR: AD5933_GetRegisterValue: wiringPiI2CWriteReg8 2: ");

		//rByte = wiringPiI2CReadReg8(i2c_fd, registerAddress + byte);
		//printf("Read: reg = 0x%02x, val = 0x%02x \n", registerAddress + byte, rByte);
		//printf("rByte = 0x%02X \n", rByte);

		registerValue = (registerValue << 8) | rByte;
	}
	//printf("****Reg = 0x%06X \n", registerValue);
	return(registerValue);
}

#endif

/******************************************************************************
 * @brief Read the AD5933 temperature.
 *
 * @param None.
 *
 * @return Temperature value.
 ******************************************************************************/
int AD5933_GetTemperature(void)
{
    int temperature = 0;
    int status = 0;
    uint8_t reg = 0;

    // Enable temperature measurement
    reg = AD5933_GetRegisterValue(AD5933_CONTROL_REG_HB, 1);
    reg = reg & 0x0F; //Clear D15-D12
    reg |=  AD5933_CONTROL_FUNCTION(AD5933_MEAS_TEMP);
    AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB, reg, 1);
    // Wait for read temperature to be valid
    while ((status & 1) == 0)
    {
        status = AD5933_GetRegisterValue(AD5933_STATUS_REG, 1);
        //printf("Status = 0x%04x\n", status);
        //usleep(500000);
    }

    // Read correct temperature value
    temperature = AD5933_GetRegisterValue(AD5933_TEMP_REG_HB, 2);

    // Calculate temperature according to datasheet specifications
    if (temperature < 8192)
    {
        temperature /= 32;
    }
    else
    {
        temperature -= 16384;
        temperature /= 32;
    }

    return (temperature);
}

/******************************************************************************
 * @brief Configure the AD5933 frequency sweep parameters.
 *
 * @param startFreq - Starting frequency value.
 *
 * @param incSteps - Number of increment steps.
 *
 * @param incFreq - Frequency step value.
 *
 * @return None.
 ******************************************************************************/
void AD5933_ConfigSweep(uint32_t startFreq, uint32_t incSteps, uint32_t incFreq)
{
    // Configure starting frequency
    AD5933_SetRegisterValue(AD5933_START_FREQ_REG_HB, startFreq, 3);
    // Configure number of steps
    AD5933_SetRegisterValue(AD5933_NR_INCR_REG_HB, incSteps, 2);
    // Configure frequency increment step
    AD5933_SetRegisterValue(AD5933_FREQ_INCR_REG_HB, incFreq, 3);
}

/******************************************************************************
 * @brief Start AD5933 frequency sweep.
 *
 * @param None.
 *
 * @return None.
 ******************************************************************************/
void AD5933_StartSweep(void)
{
    int status = 0;

    // Place AD5933 in standby
    AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB,
                            AD5933_CONTROL_FUNCTION(AD5933_STANDBY) | 1, 1);

    Delay(10);

    // Select internal system clock
    AD5933_SetRegisterValue(AD5933_CONTROL_REG_LB, 0x00, 1);

    // Initialize starting frequency
    AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB,
                               AD5933_CONTROL_FUNCTION(AD5933_INIT_START_FREQ), 1);

    // Start frequency sweep
      AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB,
                              AD5933_CONTROL_FUNCTION(AD5933_START_FREQ_SWEEP),
                              1);
    // Wait for data to be valid
    //while ((status & AD5933_STATUS_DATA_VALID) == 0)
    //{
    //    status = AD5933_GetRegisterValue(AD5933_STATUS_REG, 1);
    //}
}

/******************************************************************************
 * @brief Calculate gain factor
 *
 * @param calibrationImpedance - Known value of connected impedance for calibration.
 *
 * @param freqFunction - Select Repeat Frequency Sweep.
 *
 * @return gainFactor.
 ******************************************************************************/
void AD5933_CalculateGainFactor(unsigned long calibrationImpedance, int16_t* real, int16_t* img)
{
    //double gainFactor = 0.0;
    //double magnitude = 0.0;
    int status = 0;
    //int16_t realData = 0;
    //int16_t imgData = 0;

    AD5933_StartSweep();

    // Repeat frequency sweep with last set parameters
    AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB,
                            AD5933_CONTROL_FUNCTION(AD5933_REPEAT_FREQ) | 1, 1);

    // Wait for data received to be valid
    while ((status & AD5933_STATUS_DATA_VALID) == 0)
    {
        status = AD5933_GetRegisterValue(AD5933_STATUS_REG, 1);
    }

    // Read real and imaginary data
    //realData = (int16_t) AD5933_GetRegisterValue(AD5933_REAL_REG_HB, 2);
    //imgData = (int16_t) AD5933_GetRegisterValue(AD5933_IMG_REG_HB, 2);
    *real = (int16_t) AD5933_GetRegisterValue(AD5933_REAL_REG_HB, 2);
    *img = (int16_t) AD5933_GetRegisterValue(AD5933_IMG_REG_HB, 2);

    // Calculate magnitude
    //magnitude = sqrtf((realData * realData) + (imgData * imgData));

    // Calculate gain factor
    //gainFactor = 1.0 / (magnitude * calibrationImpedance);

    //return (gainFactor);
}

/******************************************************************************
 * @brief Calculate impedance.
 *
 * @param gainFactor - Gain factor calculated using a known impedance.
 *
 * @param freqFunction - Select Repeat Frequency Sweep.
 *
 * @return impedance.
 ******************************************************************************/
void AD5933_CalculateImpedance(int16_t* real, int16_t* img)
{
    //int16_t realData = 0;
    //int16_t imgData = 0;
    //double magnitude = 0;
    //double impedance = 0;
    int status = 0;

    // Repeat frequency sweep with last set parameters
    AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB,
                            AD5933_CONTROL_FUNCTION(AD5933_REPEAT_FREQ), 1);

    // Wait for data received to be valid
    while ((status & AD5933_STATUS_DATA_VALID) == 0)
    {
        status = AD5933_GetRegisterValue(AD5933_STATUS_REG, 1);
    }

    // Read real and imaginary data
    //realData = (int16_t) AD5933_GetRegisterValue(AD5933_REAL_REG_HB, 2);
    //imgData = (int16_t)AD5933_GetRegisterValue(AD5933_IMG_REG_HB, 2);
    *real = (int16_t) AD5933_GetRegisterValue(AD5933_REAL_REG_HB, 2);
    *img = (int16_t) AD5933_GetRegisterValue(AD5933_IMG_REG_HB, 2);


    // Calculate magnitude
    //magnitude = sqrtf((realData * realData) + (imgData * imgData));

    //printf("realData = %d, imgData = %d, magnitude = %f \n", realData, imgData, magnitude);

    // Calculate impedance
    //impedance = 1 / (magnitude * gainFactor );

    //return (magnitude);
}

//========================================================

unsigned char GetRange(unsigned short reg)
{
    switch ((reg >> 9) & 0x3)
    {
    case 0:
        return 1;
    case 1:
        return 4;
    case 2:
        return 3;
    case 3:
        return 2;
    }
    return 0;
}

#if 0
void GetCtrFunctionName(unsigned char func, char *name)
{
    switch (func)
    {
    case 0:
    case 8:
    case 12:
    case 13:
        memcpy(name, "No operation", strlen("No operation"));
        break;
    case 1:
        memcpy(name, "Init with start freq", strlen("Init with start freq"));
        break;
    case 2:
        memcpy(name, "Start freq Sweep", strlen("Init with start freq"));
        break;
    case 3:
        memcpy(name, "Increment Sweep", strlen("Increment Sweep"));
        break;
    case 4:
        memcpy(name, "Repeat Freq", strlen("Repeat Freq"));
        break;
    case 9:
        memcpy(name, "Measurement Temperature",
               strlen("Measurement Temperature"));
        break;
    case 10:
        memcpy(name, "Power-down Mode", strlen("Power-down Mode"));
        break;
    case 11:
        memcpy(name, "Standby mode", strlen("Standby mode"));
        break;

    }
}

void GetSttFunctionName(unsigned char func, char *name)
{
    switch (func & 0x7)
    {
    case 1:
        memcpy(name, "Valid Temperature Measurement",
               strlen("Valid Temperature Measurement"));
        break;
    case 2:
        memcpy(name, "Valid real/image data", strlen("Valid real/image data"));
        break;
    case 4:
        memcpy(name, "Frequency Sweep Complete",
               strlen("Frequency Sweep Complete"));
        break;
    default:
        memcpy(name, "Invalid", strlen("Invalid"));
        break;
    }
}
#endif


void AD5933_ShowRegs()
{
    uint16_t ctrlReg = AD5933_GetRegisterValue(AD5933_CONTROL_REG_HB, 2);
    uint8_t statusReg = AD5933_GetRegisterValue(AD5933_STATUS_REG, 1);
    uint32_t startFreq = AD5933_GetRegisterValue(AD5933_START_FREQ_REG_HB, 3);
    uint32_t incFreq = AD5933_GetRegisterValue(AD5933_FREQ_INCR_REG_HB, 3);
    uint16_t numSamples = AD5933_GetRegisterValue(AD5933_NR_INCR_REG_HB,2);
    uint16_t settlingReg = AD5933_GetRegisterValue(AD5933_NR_SETTLE_REG_HB, 2);



    Serial_println();
    Serial_printStr("========= ADD5933 Regiser Information ===================");Serial_println();

    Serial_printStr("I2C ADDRESS = ");  Serial_printUint(AD5933_I2C_ADDR, false); Serial_println();
     //printf("*Control Regiser  (0x%02X) = 0x%04X\n", AD5933_CONTROL_REG_HB, ctrlReg);
     Serial_printStr("*Control Register ");  Serial_printUint(AD5933_CONTROL_REG_HB, false);
     Serial_printStr(" = ");  Serial_printUint(ctrlReg, false); Serial_println();

     //printf("Start Freq: Hex(0x%06X), Dec(%6d), in Hz = %d \n", startFreq, startFreq, REG2VAL_FREQ(startFreq));
     Serial_printStr("Start Freq: Hex = ");  Serial_printUint(startFreq, false);
     Serial_printStr(", Dec = ");  Serial_printUint(startFreq, true);
     Serial_printStr(", in Hz = ");  Serial_printUint(REG2VAL_FREQ(startFreq), true);
     /*
     GetCtrFunctionName((ctrlReg >> 12) & 0xF, buf);
     printf("\tFunction: %d = %s \n", (ctrlReg >> 12) & 0xF, buf);
     printf("\tRange:    %d \n", GetRange(ctrlReg));
     printf("\tPGA gain: %d \n", (ctrlReg >> 8) & 0x1 > 0 ? 1:5);
     printf("\tReset:    %d \n", (ctrlReg >> 4) & 0x01);
     printf("\tExt Clk:  %d \n", (ctrlReg >> 3) & 0x01);

     printf("*Status Regiser  (0x%02X) = 0x%02X\n", AD5933_STATUS_REG, statusReg);
     memset(buf, 0, 50);
     GetSttFunctionName(statusReg, buf);
     printf("\tFunction: %d = %s \n", statusReg & 0x7, buf);

     printf("Start Freq: Hex(0x%06X), Dec(%6d), in Hz = %d \n", startFreq, startFreq, REG2VAL_FREQ(startFreq));
     printf("Inc   Freq: Hex(0x%06X), Dec(%6d), in Hz = %d \n", incFreq, incFreq, REG2VAL_FREQ(incFreq));
     printf("Number of Sample: %d \n", numSamples);
     printf("NUMBER OF SETTLING TIME CYCLES: Hex(0x%06X) \n", settlingReg);
     printf("\tDecode:                           %d \n", (settlingReg >> 9) & 0x3);
     printf("\tMSB:                              %d \n", (settlingReg >> 8) & 0x1);
     printf("\tNumber of settling time cycles:   %d \n", (settlingReg) & 0xFF);
     */

}

void ADD59_Sweep(unsigned int startFreq, unsigned int incFreq,
                 unsigned short samples)
{
#if 0
    AD5933_SetRegisterValue(AD5933_START_FREQ_REG_HB, VAL2REG_FREQ(startFreq),
                            3);
    AD5933_SetRegisterValue(AD5933_FREQ_INCR_REG_HB, VAL2REG_FREQ(incFreq), 3);
    AD5933_SetRegisterValue(AD5933_NR_INCR_REG_HB, samples, 2);

    AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB,
                            AD5933_CONTROL_FUNCTION(AD5933_STANDBY), 1);

    // Select internal system clock
    AD5933_SetRegisterValue(AD5933_CONTROL_REG_LB, 0x00, 1);

    // Initialize starting frequency
    AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB,
                            AD5933_CONTROL_FUNCTION(AD5933_INIT_START_FREQ), 1);

    // Start frequency sweep
    Delay(50);
    AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB,
                            AD5933_CONTROL_FUNCTION(AD5933_START_FREQ_SWEEP),
                            1);
    Delay(10);

    unsigned char status = 0;
    signed short realData = 0, imgData = 0;
    double magnitude = 0.0;
    int counter = 0;
    unsigned int curFeq = startFreq;
    float gainFactor = 0.0f;
    double impedance = 0.0;
    while (1)
    {

        //While for real/image conversion
        while ((status & AD5933_STATUS_DATA_VALID) == 0)
        {
            status = AD5933_GetRegisterValue(AD5933_STATUS_REG, 1);
        }
        counter++;
        //printf("Conversion Complete: %d \n", counter);
        realData = AD5933_GetRegisterValue(AD5933_REAL_REG_HB, 2);
        imgData = AD5933_GetRegisterValue(AD5933_IMG_REG_HB, 2);
        magnitude = sqrtf((realData * realData) + (imgData * imgData));
        gainFactor = AD5933_CalculateGainFactor(100000, AD5933_REPEAT_FREQ); //100kOhm
        impedance = 1 / (magnitude * gainFactor / 1000000000);
        //printf("counter = %d, curFeq = %d, realData = %d, imgData = %d, magnitude = %2.2f, impedance = %3.2f \n",
        //	counter, curFeq, realData, imgData, magnitude, impedance);

        status = AD5933_GetRegisterValue(AD5933_STATUS_REG, 1);
        if ((status & AD5933_STATUS_SWEEP_DONE) != 0)
            break;
        else
        {
            curFeq += incFreq;
            AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB,
                                    AD5933_CONTROL_FUNCTION(AD5933_INCR_FREQ),
                                    1);

        }

        if (counter >= samples)
            break;
    }

    AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB,
                            AD5933_CONTROL_FUNCTION(AD5933_POWER_DOWN), 1);
#endif
}
