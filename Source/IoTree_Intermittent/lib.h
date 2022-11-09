/*
 * lib.h
 *
 *  Created on: Aug 14, 2021
 *      Author: tuan.dang@uta.edu
 */

#ifndef LIB_H_
#define LIB_H_

/*TIME*/
#define TIME_1US        (16)
#define TIME_1MS        (16000)
#define TIME_2MS        (32000)
#define TIME_10MS       (160000)
#define TIME_20MS       (320000)
#define TIME_50MS       (800000)
#define TIME_100MS      (1600000)
#define TIME_200MS      (3200000)
#define TIME_500MS      (8000000)
#define TIME_1SEC         (16000000)


/*  POWER */
#define POWER_SENSING   1
#define POWER_COMM      2
#define ON              1
#define OFF             0

/* LORA DEFINE */
#define LORA_MODE_NORMAL        1
#define LORA_MODE_WAKEUP        2
#define LORA_MODE_PW_SAVING     3
#define LORA_MODE_SLEEP         4   //SETTING MODE


/* Interface */
void init_system();
void init_gpio();
void init_clock_16MHz();
void power(uint8_t source, uint8_t onOff);
void lora_config(uint8_t mode);
void lora_read_config();
void lora_write_config();
void led_power(uint8_t state);
void test_io();


#endif /* LIB_H_ */
