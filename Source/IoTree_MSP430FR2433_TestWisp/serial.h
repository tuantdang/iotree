/*
 * Serial.h
 *
 *  Created on: 2020. 7. 23.
 *      Author: tuandang
 */

#ifdef __cplusplus
extern "C"{
#endif


#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdint.h>
#include <stdbool.h>

#define MAX_STRBUF_SIZE 32

void Serial_init();
void Serial_receiveString(char data);
void Serial_transmitString(char *str);
void Serial_write(uint8_t *buf, int size);
void Serial_printStr(char *str);
void Serial_println();
void Serial_printInt(int32_t val, bool dec);
void Serial_printUint(uint32_t val, bool dec);
void Serial_printFloat(float val);
uint8_t Serial_readByte();


#endif /* SERIAL_H_ */


#ifdef __cplusplus
}
#endif

