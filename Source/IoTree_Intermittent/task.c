/*
 * task.c
 *
 *  Created on: Aug 14, 2021
 *      Author: tuan.dang@uta.edu
 */

#include <msp430.h>
#include <stdint.h>

#include "task.h"
#include "lib.h"

/* Forward declaration */
void task_func();
void MoveTo(task_t*);

/* Setup pointers to FRAM */
task_t* sensing_task        = (task_t*)(SENSING_TASK_ADDR);
task_t* compressing_task    = (task_t*)(COMPRESSING_TASK_ADDR);
task_t* transmitting_task   = (task_t*)(TRANSMITTING_TASK_ADDR);
task_t** current_task       = (task_t**)(CURRENT_TASK_POINTER_ADDR);


void start(){
    MoveTo(*current_task);
}

void MoveTo(task_t* task){
    FRAM_WRIRE_BEGIN();
    CHECKPOINT(current_task, task);
    FRAM_WRIRE_END();
    task->task_func();
}

void init_tasks(){
   FRAM_WRIRE_BEGIN(); //Start writing into FRAM

    //Task func
    CHECKPOINT(&sensing_task->task_func, task_func);
    CHECKPOINT(&compressing_task->task_func, task_func);
    CHECKPOINT(&transmitting_task->task_func, task_func);

    //Next task
    CHECKPOINT(&sensing_task->next_task, compressing_task);
    CHECKPOINT(&compressing_task->next_task, transmitting_task);
    CHECKPOINT(&transmitting_task->next_task, sensing_task);

    //Share memory pipe line here
    CHECKPOINT(&sensing_task->idata, (uint8_t*)0); //No input for sensing data
    CHECKPOINT(&sensing_task->odata, (uint8_t*)SHARED_DATA_ADDR1);

    CHECKPOINT(&compressing_task->idata, (uint8_t*)SHARED_DATA_ADDR1);
    CHECKPOINT(&compressing_task->odata, (uint8_t*)SHARED_DATA_ADDR2);

    CHECKPOINT(&transmitting_task->idata, (uint8_t*)SHARED_DATA_ADDR2);
    CHECKPOINT(&transmitting_task->odata, (uint8_t*)0); //No output for transmitting data

    FRAM_WRIRE_END();  //Clock writing into FRAM
    __delay_cycles(TIME_1US);
}


/* Task functions for all tasks*/

uint8_t bid = 0, bnumber = 0;
uint8_t  offset = 0, size = 0;
uint8_t local_ibuf[SHARED_MEM_SIZE] = {0};
uint8_t local_obuf[SHARED_MEM_SIZE] = {0};

void task_func(){
    /*Static local volatile variables to speedup on stack creation inside task function due to avoidance multiple creation local variables*/

    //Restore non-volatile memory into volatile memory
    bid       = (uint8_t) RESTORE(&(*current_task)->block_id);
    bnumber   = (uint8_t) RESTORE(&(*current_task)->block_number);

    /*This loop will execute block by block while maintaining forward progress and data consistency*/
    while (bid < bnumber){
        //Base on task type, block id, block number, offset and size are calculated as below

        //Copy shared memory into local memory
        //Some tasks do not have input data
        if ((*current_task)->idata)
            //memcpy(local_ibuf, (*current_task)->idata + offset, size);
            if (*current_task == transmitting_task){
                size = SHARED_MEM_SIZE/bnumber;
                memcpy(local_ibuf, (*current_task)->idata + bid*size, size);
            }
            else
                memcpy(local_ibuf, (*current_task)->idata, SHARED_MEM_SIZE);

        //User callback with block id and local buffer
        //User will collect data on output buffer
        (*current_task)->block_func(&bid, &bnumber, local_ibuf, local_obuf);

       FRAM_WRIRE_BEGIN();
        //Copy shared memory into local memory
        //Some tasks do not have output data
        if ((*current_task)->odata)
            //memcpy((*current_task)->odata + offset, local_obuf,  size);

            if (*current_task == sensing_task)
            {
#if defined(USE_MUL_SENSE_BLOCK) //Use 2 ranges:  low and high at difference blocks
                if (bid == 0)
                    memcpy((*current_task)->odata, local_obuf,  4*10);  //Low Range
                else memcpy((*current_task)->odata + 4*10, local_obuf + 4*10,  4*6); //High Range
#else
                memcpy((*current_task)->odata, local_obuf,  4*16);  //All
#endif
            }
            else
                memcpy((*current_task)->odata, local_obuf,  SHARED_MEM_SIZE);
        bid++;
        CHECKPOINT(&(*current_task)->block_id, bid);
        FRAM_WRIRE_END();
        __delay_cycles(TIME_1MS);
    }

    //On Exit: Reset Block Id
    FRAM_WRIRE_BEGIN();
    CHECKPOINT(&(*current_task)->block_id, 0);
    FRAM_WRIRE_END();

    //Next task
    MoveTo((*current_task)->next_task);
}


void set_user_func(TASK_TYPE type,  block_func_t* func){
   FRAM_WRIRE_BEGIN();
    switch (type){
    case TASK_SENSE:
        CHECKPOINT(&sensing_task->block_func, func);
        break;
    case TASK_COMPRESS:
        CHECKPOINT(&compressing_task->block_func, func);
        break;
    case TASK_TRANSMIT:
        CHECKPOINT(&transmitting_task->block_func, func);
        break;
    default:
        break;
    }
   FRAM_WRIRE_BEGIN();
}


void set_block_number(TASK_TYPE type,  uint8_t number){
   FRAM_WRIRE_BEGIN();
    switch (type){
    case TASK_SENSE:
        CHECKPOINT(&sensing_task->block_number, number);
        break;
    case TASK_COMPRESS:
        CHECKPOINT(&compressing_task->block_number, number);
        break;
    case TASK_TRANSMIT:
        CHECKPOINT(&transmitting_task->block_number, number);
        break;
    default:
        break;
    }
   FRAM_WRIRE_BEGIN();
}

uint8_t get_block_number(TASK_TYPE type){

    switch (type){
    case TASK_SENSE:
        return RESTORE(&sensing_task->block_number);
    case TASK_COMPRESS:
        return RESTORE(&compressing_task->block_number);
    case TASK_TRANSMIT:
        return RESTORE(&transmitting_task->block_number);
    default:
        return 0;
    }
}

void set_next_task(TASK_TYPE type,  task_t* task){
   FRAM_WRIRE_BEGIN();
    switch (type){
    case TASK_SENSE:
        CHECKPOINT(&sensing_task->next_task, task);
        break;
    case TASK_COMPRESS:
        CHECKPOINT(&compressing_task->next_task, task);
        break;
    case TASK_TRANSMIT:
        CHECKPOINT(&transmitting_task->next_task, task);
        break;
    default:
        break;
    }
   FRAM_WRIRE_BEGIN();
}


void reset_fram(){
    int i = 0;
    uint8_t* ptr = (uint8_t*) FRAM_ADDR;
   FRAM_WRIRE_BEGIN();
    for(i = 0; i <= 0xff; i++){
        *(ptr + i) = 0;
    }
    FRAM_WRIRE_END();
}



