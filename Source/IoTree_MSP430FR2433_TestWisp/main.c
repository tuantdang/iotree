#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "AD5933.h"
#include "serial.h"
#include <string.h>

//******************************************************************************
// Pin Config ******************************************************************
//******************************************************************************

#define LED_OUT     P1OUT
#define LED_DIR     P1DIR
#define LED0_PIN    BIT0
#define LED1_PIN    BIT1
int i = 0;

//******************************************************************************
// Device Initialization *******************************************************
//******************************************************************************

void initGPIO()
{
    // Configure GPIO
    LED_OUT &= ~(LED0_PIN | LED1_PIN); // P1 setup for LED & reset output
    LED_DIR |= (LED0_PIN | LED1_PIN);

    // I2C pins
    P1SEL0 |= BIT2 | BIT3;
    P1SEL1 &= ~(BIT2 | BIT3);

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;
}

void initClockTo16MHz()
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

//******************************************************************************
// Main ************************************************************************
// Send and receive three messages containing the example commands *************
//******************************************************************************

#define SOF1    0xFE
#define SOF2    0xFF
#define MAX_TX_BUF  (256*5)
#define MAX_RX_BUF  15

#define CMD_GET_ALL_REGS        0x01
#define CMD_SWEEP               0x02
#define CMD_GET_TEMPERATURE     0x03
#define CMD_CALIBRATION         0x04
#define CMD_GET_ONESHOT_100TIMES 0x05
#define CMD_IMPEDANCE           0x06

#define CMD_SET_FUNC             0x11
#define CMD_SET_RANGE            0x12
#define CMD_SET_GAIN             0x13
#define CMD_SET_RESET            0x14
#define CMD_SET_START_FREQ       0x15
#define CMD_SET_INC              0x16
#define CMD_SET_NUM_SAMPLES      0x17
#define CMD_SET_SETTILING        0x18

uint8_t cmd = 0, byte1 = 0, byte2 = 0;
uint8_t txBuf[MAX_TX_BUF];
uint8_t rxBuf[MAX_RX_BUF];

#define DEFAULT_START_FREQ  5000
//#define DEFAULT_INC_FREQ    4990
#define DEFAULT_INC_FREQ    5000
#define SETTLE_TIME         100

uint16_t ctrlReg = 0;
uint8_t statusReg = 0;
static uint32_t startFreq = DEFAULT_START_FREQ;
static uint32_t incFreq = DEFAULT_INC_FREQ;
uint16_t numSamples = 3;
uint16_t settlingReg = 0;
int memIndex = 0;
int temperature = 0;


uint8_t status = 0;
int16_t realData = 0, imgData = 0;
double magnitude = 0.0;
int counter = 0;
static uint32_t curFeq = 0;
double gainFactor = 1.0f;
double impedance = 0.0;
uint32_t calibrateRes = 0;

//Data
uint8_t byte = 0;
uint16_t word = 0;
uint32_t dword = 0;

//Reg
static uint8_t RegCtlL = 0, RegCtlH = 0;

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    initClockTo16MHz();
    initGPIO();
    Serial_init();
    AD5933_Init();

    __enable_interrupt();

    for(i = 0; i < 20; i++){
           P1OUT ^= LED0_PIN;
           __delay_cycles(16000000/10);
    }

    //Serial_printStr("Hello! \r\n");

    RegCtlH = AD5933_GetRegisterValue(AD5933_CONTROL_REG_HB, 1);
    RegCtlL = AD5933_GetRegisterValue(AD5933_CONTROL_REG_LB, 1);

    AD5933_SetRegisterValue(AD5933_START_FREQ_REG_HB, VAL2REG_FREQ(DEFAULT_START_FREQ), 3);
    AD5933_SetRegisterValue(AD5933_FREQ_INCR_REG_HB, VAL2REG_FREQ(DEFAULT_INC_FREQ), 3);
    AD5933_SetRegisterValue(AD5933_NR_INCR_REG_HB, numSamples, 2);
    AD5933_SetRegisterValue(AD5933_NR_SETTLE_REG_HB, 0, 1);
    AD5933_SetRegisterValue(AD5933_NR_SETTLE_REG_LB, SETTLE_TIME, 1);
    //Set Gain -> 1
    RegCtlH = RegCtlH & 0xFE; //Clear D8s
    RegCtlH = RegCtlH | 1; //D8 = 1: Gain = 1; If D8=0: Gain = 5;
    AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB, RegCtlH, 1);

    while (1)
    {
        cmd = Serial_readByte();
        //cmd=2;
        P1OUT |= 0x1;
        switch (cmd)
        {
        case CMD_GET_ALL_REGS:
            memset(txBuf, 0, MAX_TX_BUF);
            memIndex = 0;

            ctrlReg = AD5933_GetRegisterValue(AD5933_CONTROL_REG_HB, 2);
            statusReg = AD5933_GetRegisterValue(AD5933_STATUS_REG, 1);
            startFreq = AD5933_GetRegisterValue(AD5933_START_FREQ_REG_HB, 3);
            incFreq = AD5933_GetRegisterValue(AD5933_FREQ_INCR_REG_HB, 3);
            numSamples = AD5933_GetRegisterValue(AD5933_NR_INCR_REG_HB, 2);
            settlingReg = AD5933_GetRegisterValue(AD5933_NR_SETTLE_REG_HB, 2);

            memcpy(txBuf + memIndex, &ctrlReg, 2);
            memIndex += 2;
            memcpy(txBuf + memIndex, &statusReg, 1);
            memIndex += 1;
            memcpy(txBuf + memIndex, &startFreq, 4);
            memIndex += 4;
            memcpy(txBuf + memIndex, &incFreq, 4);
            memIndex += 4;
            memcpy(txBuf + memIndex, &numSamples, 2);
            memIndex += 2;
            memcpy(txBuf + memIndex, &settlingReg, 2);
            memIndex += 2;
            Serial_write(txBuf, memIndex);
            break;

        case CMD_SWEEP:
#if 1
            memset(txBuf, 0, MAX_TX_BUF);
            AD5933_StartSweep();
            memIndex = 0;
            counter = 0;
            while (1)
            {
                //While for real/image conversion
                while ((status & AD5933_STATUS_DATA_VALID) == 0)
                {
                    status = AD5933_GetRegisterValue(AD5933_STATUS_REG, 1);
                }
                counter++;
                realData = AD5933_GetRegisterValue(AD5933_REAL_REG_HB, 2);
                imgData = AD5933_GetRegisterValue(AD5933_IMG_REG_HB, 2);

                if (counter > 1){ //skip first sample
                    memcpy(txBuf + memIndex, &realData, 2);
                    memIndex += 2;
                    memcpy(txBuf + memIndex, &imgData, 2);
                    memIndex += 2;
                }

                status = AD5933_GetRegisterValue(AD5933_STATUS_REG, 1);
                if ((status & AD5933_STATUS_SWEEP_DONE) != 0)
                    break;
                else
                {
                    curFeq += DEFAULT_INC_FREQ;
                    AD5933_SetRegisterValue(
                            AD5933_CONTROL_REG_HB,
                            AD5933_CONTROL_FUNCTION(AD5933_INCR_FREQ), 1);
                }
                if (counter >= numSamples)
                    break;
            }
            AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB,
                                                AD5933_CONTROL_FUNCTION(AD5933_POWER_DOWN),
                                                1);
            Serial_write(txBuf, memIndex);
#else
            txBuf[0]++;
            if (txBuf[0] >= 3) txBuf[0] = 0;
            Serial_write(txBuf, 8);
#endif
            break;

        case CMD_GET_ONESHOT_100TIMES:
            for(i = 0; i < 100; i++){
                AD5933_SetRegisterValue(AD5933_START_FREQ_REG_HB, VAL2REG_FREQ(startFreq), 3);
            }
            break;

        case CMD_GET_TEMPERATURE:
            memset(txBuf, 0, MAX_TX_BUF);
            memIndex = 0;

            temperature = AD5933_GetTemperature();
            memcpy(txBuf + memIndex, &temperature, sizeof(int));
            memIndex += sizeof(int);
            Serial_write(txBuf, memIndex);
            break;

        case CMD_CALIBRATION:
            memset(rxBuf, 0, MAX_RX_BUF);
            for (i = 0; i < 4; i++)
            {
                 rxBuf[i] = Serial_readByte();
            }
            memcpy(&calibrateRes, rxBuf, 4); //Read Calibration Resistor

            memset(txBuf, 0, MAX_TX_BUF);
            memIndex = 0;
            AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB, RegCtlH | 1, 1);
            AD5933_SetRegisterValue(AD5933_START_FREQ_REG_HB, VAL2REG_FREQ(12000), 3);
            AD5933_CalculateGainFactor(calibrateRes,  &realData, &imgData);
            memcpy(txBuf + memIndex, &realData, sizeof(int16_t)); memIndex += sizeof(int16_t);
            memcpy(txBuf + memIndex, &imgData, sizeof(int16_t)); memIndex += sizeof(int16_t);

            AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB, RegCtlH | 1, 1);
            AD5933_SetRegisterValue(AD5933_START_FREQ_REG_HB, VAL2REG_FREQ(18000), 3);
            AD5933_CalculateGainFactor(calibrateRes,  &realData, &imgData);
            memcpy(txBuf + memIndex, &realData, sizeof(int16_t)); memIndex += sizeof(int16_t);
            memcpy(txBuf + memIndex, &imgData, sizeof(int16_t)); memIndex += sizeof(int16_t);
            Serial_write(txBuf, memIndex);
            break;

        case CMD_IMPEDANCE:
            memset(txBuf, 0, MAX_TX_BUF);
            memIndex = 0;
            AD5933_CalculateImpedance(&realData, &imgData);
            memcpy(txBuf + memIndex, &realData, sizeof(int16_t)); memIndex += sizeof(int16_t);
            memcpy(txBuf + memIndex, &imgData, sizeof(int16_t)); memIndex += sizeof(int16_t);
            Serial_write(txBuf, memIndex);
            break;

        case CMD_SET_FUNC:
            byte = Serial_readByte();

            RegCtlH = RegCtlH & 0x0F; //Clear D15-D12
            RegCtlH = RegCtlH | (byte << 4);
            AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB, RegCtlH, 1);

            //Response
            memset(txBuf, 0, MAX_TX_BUF);
            txBuf[0] = 0xfe;
            txBuf[1] = 0xff;
            txBuf[2] = cmd;
            Serial_write(txBuf, 3);
            break;

        case CMD_SET_RANGE:
            byte = Serial_readByte();

            RegCtlH = RegCtlH & 0xF9; //Clear D10-D9
            RegCtlH = RegCtlH | (byte << 1);
            AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB, RegCtlH, 1);

            //Response
            memset(txBuf, 0, MAX_TX_BUF);
            txBuf[0] = 0xfe;
            txBuf[1] = 0xff;
            txBuf[2] = cmd;
            Serial_write(txBuf, 3);
            break;

        case CMD_SET_GAIN:
            byte = Serial_readByte();

            RegCtlH = RegCtlH & 0xFE; //Clear D10-D9
            RegCtlH = RegCtlH | byte;
            AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB, RegCtlH, 1);

            //Response
            memset(txBuf, 0, MAX_TX_BUF);
            txBuf[0] = 0xfe;
            txBuf[1] = 0xff;
            txBuf[2] = cmd;
            Serial_write(txBuf, 3);
            break;

        case CMD_SET_RESET:

            AD5933_SetRegisterValue(AD5933_CONTROL_REG_LB, RegCtlL | (1 << 4),
                                    1);

            //Response
            memset(txBuf, 0, MAX_TX_BUF);
            txBuf[0] = 0xfe;
            txBuf[1] = 0xff;
            txBuf[2] = cmd;
            Serial_write(txBuf, 3);
            break;

        case CMD_SET_START_FREQ:
            memset(rxBuf, 0, MAX_RX_BUF);
            for (i = 0; i < 4; i++)
            {
                rxBuf[i] = Serial_readByte();
            }
            memcpy(&startFreq, rxBuf, 4);

            AD5933_SetRegisterValue(AD5933_START_FREQ_REG_HB,
                                    VAL2REG_FREQ(startFreq), 3);

            //Response
            memset(txBuf, 0, MAX_TX_BUF);
            txBuf[0] = 0xfe;
            txBuf[1] = 0xff;
            txBuf[2] = cmd;
            Serial_write(txBuf, 3);
            break;

        case CMD_SET_INC:
            memset(rxBuf, 0, MAX_RX_BUF);
            for (i = 0; i < 4; i++)
            {
                rxBuf[i] = Serial_readByte();
            }
            memcpy(&incFreq, rxBuf, 4);
            AD5933_SetRegisterValue(AD5933_FREQ_INCR_REG_HB,
                                    VAL2REG_FREQ(incFreq), 3);
            //Response
            memset(txBuf, 0, MAX_TX_BUF);
            txBuf[0] = 0xfe;
            txBuf[1] = 0xff;
            txBuf[2] = cmd;
            Serial_write(txBuf, 3);
            break;

        case CMD_SET_NUM_SAMPLES:
            memset(rxBuf, 0, MAX_RX_BUF);
            for (i = 0; i < 2; i++)
            {
                rxBuf[i] = Serial_readByte();
            }
            memcpy(&numSamples, rxBuf, 2);
            AD5933_SetRegisterValue(AD5933_NR_INCR_REG_HB, numSamples, 2);

            //Response
            memset(txBuf, 0, MAX_TX_BUF);
            txBuf[0] = 0xfe;
            txBuf[1] = 0xff;
            txBuf[2] = cmd;
            Serial_write(txBuf, 3);
            break;

        case CMD_SET_SETTILING:
            break;

        default:
            break;
        }
        P1OUT &= 0xfe;
    //    __bis_SR_register(LPM0_bits + GIE);
    }
    return 0;
}

