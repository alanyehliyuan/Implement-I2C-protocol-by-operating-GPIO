#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include "main.h"

//I2C speed (latency time)
//I2C standard mode (100kHz) -> period = 1/100k = 10us -> half period = 5us

#define I2C_DELAY_US 5

//function declaration
void SoftI2C_Init(void);
void SoftI2C_Start(void);
void SoftI2C_Stop(void);

uint8_t SoftI2C_WriteByte(uint8_t data);
uint8_t SoftI2C_ReadByte(uint8_t ack);

#endif