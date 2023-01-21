/*
 * lis3dh_example.h
 *
 *  Created on: Jan 18, 2023
 *      Author: dnj
 */

#ifndef INC_LIS3DH_EXAMPLE_H_
#define INC_LIS3DH_EXAMPLE_H_
#include <string.h>
#include <stdio.h>
#include "stm32g0xx_hal.h"
#include "lis3dh_reg.h"
#include "main.h"

#define CS_up_GPIO_Port SPI1_CS_GPIO_Port
#define CS_up_Pin SPI1_CS_Pin

void lis3dh_read_fifo(void);

#endif /* INC_LIS3DH_EXAMPLE_H_ */
