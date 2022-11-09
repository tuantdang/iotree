/*
 * task.h
 *
 *  Created on: Aug 14, 2021
 *      Author: tuan.dang@uta.edu
 */

#ifndef TASK_H_
#define TASK_H_

/*FRAM Memory Address*/
#define FRAM_ADDR                        0x1800
#define SENSING_TASK_ADDR               (FRAM_ADDR)
#define COMPRESSING_TASK_ADDR           (FRAM_ADDR + 0x20)
#define TRANSMITTING_TASK_ADDR          (FRAM_ADDR + 0x40)
#define CURRENT_TASK_POINTER_ADDR       (FRAM_ADDR + 0x60)  //2 bytes pointer
#define SHARED_DATA_ADDR1               (FRAM_ADDR + 0x80)  //64 bytes
#define SHARED_DATA_ADDR2               (FRAM_ADDR + 0xC0)  //64 bytes
#define RESET_FRAM_BIT_ADDR             (FRAM_ADDR + 0x100)  //1 bytes

/* Shared Memory Size*/
#define SHARED_MEM_SIZE                 64

/* Write FRAM: MCU dependence */
#define FRAM_WRIRE_BEGIN(void)          SYSCFG0 = FRWPPW | PFWP
#define FRAM_WRIRE_END(void)            SYSCFG0 = FRWPPW | PFWP | DFWP

//Function pointer type
typedef void (task_func_t)(void);
typedef void (block_func_t)(uint8_t* bid, uint8_t* bnumber, uint8_t* ibuf, uint8_t* obuf);

//Structure padding default is 2 bytes
typedef struct {
    //Total 16 bytes:
    task_func_t* task_func;         //4 bytes
    block_func_t* block_func;         //4 bytes
    struct task_t* next_task;       //2 bytes
    uint8_t* block_id;              //2 bytes
    uint8_t* block_number;          //2 bytes
    uint8_t* idata;                 //2 bytes
    uint8_t* odata;                 //2 bytes
    //sizeof(task_t) =  8
    //sizeof(task_t*) = 2
} task_t;

typedef enum{
    TASK_SENSE = 1,
    TASK_COMPRESS,
    TASK_TRANSMIT,
}TASK_TYPE;


/* Primary operation on non-volatile memory */
#define INC(addr)                   (*addr = *(addr) + 1)
#define RESTORE(addr)               (*addr)
#define CHECKPOINT(addr, val)       (*addr = val)
#define MOVETO(task)                {   FRAM_WRIRE_BEGIN(); \
                                        CHECKPOINT(current_task, task); \
                                        FRAM_WRIRE_END(); \
                                        (*current_task)->task_func();\
                                    }
#define IS_FIRST_RUN(void)          (RESTORE((unsigned char*)RESET_FRAM_BIT_ADDR) == 0)
#define MARK_FIRST_RUN(void)        {\
                                        extern task_t** current_task; \
                                        extern task_t*  sensing_task; \
                                        FRAM_WRIRE_BEGIN();\
                                        CHECKPOINT(current_task, sensing_task); \
                                        CHECKPOINT(((unsigned char*)RESET_FRAM_BIT_ADDR), 1); \
                                        FRAM_WRIRE_END();\
                                    }
/* This macro is used when program is re-compiled.
 * That time user function and task offset are changed.
 * We need to set this line to setup for the first run.
 * */
#define SET_FIRST_RUN(void)        {\
                                        FRAM_WRIRE_BEGIN();\
                                        CHECKPOINT(((unsigned char*)RESET_FRAM_BIT_ADDR), 0); \
                                        FRAM_WRIRE_END();\
                                    }



/* Interface here */
void init_tasks();
void set_user_func(TASK_TYPE type,  block_func_t* func);
void set_block_number(TASK_TYPE type,  uint8_t number);
uint8_t get_block_number(TASK_TYPE type);
void start();
void reset_fram();

#endif /* TASK_H_ */
