#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "AD5933.h"
#include "serial.h"
#include <string.h>

#include "lib.h"
#include "task.h"

/* By turn of this option we can test the correctness of intermittent computing block-based approach
 * Three tasks and each task has a number of blocks */
//#define TEST_INTERMITTENT

/* User code forward declaration*/
/* A task will be divided into multiple block from block id = 0 to block id = N*/
/* User can handle  operation within specific block with block id*/
/* Input data & output data belongs to block id*/
/* Run-time library will protect output data in case of power failure*/
void sense_block_handler(uint8_t* bid,  uint8_t* bnum, uint8_t* ibuf, uint8_t* obuf);
void compress_block_handler(uint8_t* bid,  uint8_t* bnum, uint8_t* ibuf, uint8_t* obuf);
void transmit_block_handler(uint8_t* bid,  uint8_t* bnum, uint8_t* ibuf, uint8_t* obuf);

void main(void)
{
    init_system();
    /* These lines of code only run for the first time */
    if (IS_FIRST_RUN())
    {
        reset_fram();
        init_tasks();
        set_user_func(TASK_SENSE, sense_block_handler);
        set_user_func(TASK_COMPRESS, compress_block_handler);
        set_user_func(TASK_TRANSMIT, transmit_block_handler);

        //Block number is set based on previous energy consumption
        //and can be changed inside user code for each task
#if defined(USE_MUL_SENSE_BLOCK)
        set_block_number(TASK_SENSE, 2); //2 for low-high ranges sweeping frequencies
#else
        set_block_number(TASK_SENSE, 1); //2 for low-high ranges sweeping frequencies
#endif
        set_block_number(TASK_COMPRESS, 1);
        set_block_number(TASK_TRANSMIT, 1);
        MARK_FIRST_RUN();
    }
    start(); //Start System
}

/* User code start here */
/* Sensing task
 *Parameter bid: block id
 *Parameter buf: volatile memory which is eventual written in run-time library
 *Therefore, data consistency is protected by run-time library, user only working volatile memory provided by the interface (bid, buf)
*/
unsigned long test = 0;
uint8_t flag_init_sense = OFF;
uint8_t numSamples = 0, counter = 0, status = 0;
int16_t realData = 0, imgData = 0;
void sense_block_handler(uint8_t* ptr_bid, uint8_t* ptr__bnum, uint8_t* ibuf,  uint8_t* obuf){
#define FREQ_NUM 16

    /*Handle as entire a task*/
#if defined(TEST_INTERMITTENT)
   led_power(ON);
   __delay_cycles(TIME_100MS);
   led_power(OFF);
   __delay_cycles(TIME_100MS);
   int i = 0;
   for(i = 0; i < bnum; i++){
       obuf[i] = i;
   }

   /*Handle specific block by block id*/
   switch (bid){
   case 0:
       //User code here
       //blokc1();
       break;
   case 1:
       //User code here
       //blokc2();
       break;

   case 2:
       //User code here
       break;
   default: break;
   }

#else
   /* Static volatile variable holding state of user code to avoid duplicated initialization */

      //static unsigned long freq[FREQ_NUM] = {10000, 11000, 12000, 13000, 14000, 15000, 16000, 17000, 18000, 19000,  100000, 200000, 300000, 400000, 500000, 600000};




       //__delay_cycles(TIME_10MS);

       if (flag_init_sense == OFF){
           flag_init_sense = ON;
           //power(POWER_SENSING, ON);
           //power(POWER_COMM, OFF);
           AD5933_Init(); //I2C
           __enable_interrupt();
       }

       power(POWER_SENSING, ON);
       __delay_cycles(TIME_20MS);
       //led_power(ON);

#if defined(USE_MUL_SENSE_BLOCK)
       if (*ptr_bid == 0)
#endif
       {
          uint8_t RegCtlH = 0;
          RegCtlH = AD5933_GetRegisterValue(AD5933_CONTROL_REG_HB, 1);
          AD5933_SetRegisterValue(AD5933_NR_SETTLE_REG_HB, 0, 1);
          AD5933_SetRegisterValue(AD5933_NR_SETTLE_REG_LB, 10, 1);
          RegCtlH = RegCtlH & 0xFE; //Clear D8s
          RegCtlH = RegCtlH | 1; //D8 = 1: Gain = 1; If D8=0: Gain = 5;
          AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB, RegCtlH, 1);
          __delay_cycles(TIME_10MS);

           numSamples = 10;
           counter = 0;
           //AD5933_SetRegisterValue(AD5933_START_FREQ_REG_HB, VAL2REG_FREQ(freq[*ptr_bid]), 3);
           AD5933_SetRegisterValue(AD5933_START_FREQ_REG_HB, VAL2REG_FREQ(10000), 3);
           AD5933_SetRegisterValue(AD5933_FREQ_INCR_REG_HB, VAL2REG_FREQ(1000), 3);
           AD5933_SetRegisterValue(AD5933_NR_INCR_REG_HB, numSamples, 2);
           AD5933_StartSweep();
           while (1)
           {
              //While for real/image conversion
              while ((status & AD5933_STATUS_DATA_VALID) == 0)
              {
                  status = AD5933_GetRegisterValue(AD5933_STATUS_REG, 1);
              }
              realData = AD5933_GetRegisterValue(AD5933_REAL_REG_HB, 2);
              imgData = AD5933_GetRegisterValue(AD5933_IMG_REG_HB, 2);

              //memcpy(obuf + (*ptr_bid)*4, &realData, 2);
              //memcpy(obuf + (*ptr_bid)*4 + 2, &imgData, 2);
              memcpy(obuf + counter*4, &realData, 2);
              memcpy(obuf + counter*4 + 2, &imgData, 2);

              status = AD5933_GetRegisterValue(AD5933_STATUS_REG, 1);
              if ((status & AD5933_STATUS_SWEEP_DONE) != 0) break;
              else
              {
                  AD5933_SetRegisterValue( AD5933_CONTROL_REG_HB, AD5933_CONTROL_FUNCTION(AD5933_INCR_FREQ), 1);
              }
              counter++;
              if (counter >= numSamples) break;
           }
       }
#if defined(USE_MUL_SENSE_BLOCK)
       else
#endif
       {
           numSamples = 6;
            counter = 0;
            //AD5933_SetRegisterValue(AD5933_START_FREQ_REG_HB, VAL2REG_FREQ(freq[*ptr_bid]), 3);
            AD5933_SetRegisterValue(AD5933_START_FREQ_REG_HB, VAL2REG_FREQ(100000), 3);
            AD5933_SetRegisterValue(AD5933_FREQ_INCR_REG_HB, VAL2REG_FREQ(100000), 3);
            AD5933_SetRegisterValue(AD5933_NR_INCR_REG_HB, numSamples, 2);
            AD5933_StartSweep();
            while (1)
            {
                 //While for real/image conversion
                 while ((status & AD5933_STATUS_DATA_VALID) == 0)
                 {
                     status = AD5933_GetRegisterValue(AD5933_STATUS_REG, 1);
                 }
                 realData = AD5933_GetRegisterValue(AD5933_REAL_REG_HB, 2);
                 imgData = AD5933_GetRegisterValue(AD5933_IMG_REG_HB, 2);

                 //memcpy(obuf + (*ptr_bid)*4, &realData, 2);
                 //memcpy(obuf + (*ptr_bid)*4 + 2, &imgData, 2);
                 memcpy(obuf + 4*10 + counter*4, &realData, 2);
                 memcpy(obuf + 4*10 + counter*4 + 2, &imgData, 2);

                 status = AD5933_GetRegisterValue(AD5933_STATUS_REG, 1);
                 if ((status & AD5933_STATUS_SWEEP_DONE) != 0) break;
                 else
                 {
                     AD5933_SetRegisterValue( AD5933_CONTROL_REG_HB, AD5933_CONTROL_FUNCTION(AD5933_INCR_FREQ), 1);
                 }
                 counter++;
                 if (counter >= numSamples) break;
            }
       }
       AD5933_SetRegisterValue(AD5933_CONTROL_REG_HB, AD5933_CONTROL_FUNCTION(AD5933_POWER_DOWN), 1);
       power(POWER_SENSING, OFF);
       //led_power(OFF);
#endif
}


/* Compressing data task
 *Parameter bid: block id
 *Parameter buf: volatile memory which is eventual written in run-time library */
void compress_block_handler(uint8_t* ptr_bid, uint8_t* ptr__bnum,  uint8_t* ibuf,  uint8_t* obuf){
#if defined(TEST_INTERMITTENT)
    led_power(ON);
    __delay_cycles(TIME_1SEC);
    __delay_cycles(TIME_1SEC);
    //__delay_cycles(TIME_1SEC);
    //__delay_cycles(TIME_1SEC);
    //__delay_cycles(TIME_1SEC);
    led_power(OFF);
    int i = 0;
    for(i = 0; i < bnum; i++){
        obuf[i] = i*2;
    }
#else
    //Do Human code
    //uint8_t number = get_block_number(TASK_COMPRESS);
    //uint8_t size = SHARED_MEM_SIZE/number;
    //memcpy(obuf, ibuf, size);
    memcpy(obuf, ibuf, SHARED_MEM_SIZE);
#endif
}


#define SENSOR_ID   0x01

/* Transmitting data task
 *Parameter bid: block id
 *Parameter buf: volatile memory which is eventual written in run-time library */
uint8_t flag_init_transmit = OFF;
uint8_t header[3]= {0x0D, 0x0A, SENSOR_ID};
void transmit_block_handler(uint8_t* ptr_bid, uint8_t* ptr__bnum,  uint8_t* ibuf,  uint8_t* obuf){
#if defined(TEST_INTERMITTENT)
    led_power(ON);
   __delay_cycles(TIME_500MS);
   led_power(OFF);
   __delay_cycles(TIME_500MS);
   int i = 0;
     for(i = 0; i < bnum; i++){
         obuf[i] = i*3;
     }

#else
   /* Static volatile variable holding state of user code to avoid duplicated initialization */
   power(POWER_COMM, ON);
   __delay_cycles(TIME_10MS);

   if (flag_init_transmit == OFF){
       flag_init_transmit = ON;
      lora_config(LORA_MODE_NORMAL);
      Serial_init(UART_9600);
      __enable_interrupt();
      __delay_cycles(TIME_10MS);
   }

   //uint8_t number = get_block_number(TASK_TRANSMIT);
   //uint8_t size = SHARED_MEM_SIZE/number;
#define CHUNCK_SIZE 32      //=(64/4): Use hard code for better speed
   if (*ptr_bid == 0){
       Serial_write(header, 3);
       //Serial_write(ibuf, CHUNCK_SIZE);
       Serial_write(ibuf, 2 + 4*3);
       __delay_cycles(TIME_10MS);
   }
   else{
       //Serial_write(ibuf, CHUNCK_SIZE);
   }
   __delay_cycles(TIME_50MS);
   __delay_cycles(TIME_100MS);
   //__delay_cycles(TIME_100MS);
   //power(POWER_COMM, OFF);
#endif
}

/*
uint8_t is_first_run(){
    return (RESTORE((unsigned char*)RESET_FRAM_BIT_ADDR) == 0);
}

void mark_first_run(){
    extern task_t** current_task;
    extern task_t*  sensing_task;
    SYSCFG0 = FRWPPW | PFWP;
    CHECKPOINT(current_task, sensing_task);
    CHECKPOINT(((unsigned char*)RESET_FRAM_BIT_ADDR), 1);
    SYSCFG0 = FRWPPW | PFWP | DFWP;
}*/
