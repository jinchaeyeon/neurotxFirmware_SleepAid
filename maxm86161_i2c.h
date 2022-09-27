/***************************************************************************//**
* @file maxm86161_i2c.h
* @brief Header file of maxm86161_i2c
* @version 1.0
*******************************************************************************
* # License
* <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
*******************************************************************************
*
* SPDX-License-Identifier: Zlib
*
* The licensor of this software is Silicon Laboratories Inc.
*
* This software is provided \'as-is\', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
*
* 1. The origin of this software must not be misrepresented; you must not
*    claim that you wrote the original software. If you use this software
*    in a product, an acknowledgment in the product documentation would be
*    appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
*    misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*
*******************************************************************************
*
* EVALUATION QUALITY
* This code has been minimally tested to ensure that it builds with the specified dependency versions and is suitable as a demonstration for evaluation purposes only.
* This code will be maintained at the sole discretion of Silicon Labs.
*
******************************************************************************/
#ifndef MAXM86161_I2C_H_
#define MAXM86161_I2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "compiler_abstraction.h"
#include "nrf.h"
#include "I2Cdev.h"

/* Communicate with the MAXM86161 over I2C or SPI */
#define MAXM86161_I2C   0
#define MAXM86161_SPI   1
#define MAXM86161_BUS   MAXM86161_I2C

//#define MAXM86161_SLAVE_ADDRESS         0xC4
#define MAXM86161_SLAVE_ADDRESS         0x62        //7-bit I2C Address 

#define MAXM86161_EN_GPIO_PORT          gpioPortC
#define MAXM86161_EN_GPIO_PIN           3

#define MAXM86161_INT_GPIO_PORT         gpioPortB
#define MAXM86161_INT_GPIO_PIN          3

 /** Button 0 for start and pause measurement */
#define MAXM86161_BTN0_GPIO_PORT        gpioPortC
#define MAXM86161_BTN0_GPIO_PIN         7




#endif

