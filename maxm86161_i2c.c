/***************************************************************************//**
* @file maxm86161_i2c.c
* @brief I2C device setup for maxm86161 driver.
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
#include "maxm86161.h"
#include "maxm86161_i2c.h"
#include "I2Cdev.h"
#include "app_util_platform.h"
#include "nrf_delay.h"


uint8_t maxm86161_i2c_write_byte_data(uint8_t address, uint8_t data);
uint8_t maxm86161_i2c_read_byte_data(uint8_t address, uint8_t *data);
uint8_t maxm86161_i2c_write_i2c_block_data(uint8_t address, uint8_t length, uint8_t * values);
uint8_t maxm86161_i2c_read_i2c_block_data(uint8_t address, uint16_t length, uint8_t* values);
uint8_t maxm86161_hrm_identify_part(void) ;

/**************************************************************************//**
 * @brief Write to Maxim register
 *****************************************************************************/
uint8_t maxm86161_i2c_write_to_register(uint8_t address, uint8_t data)
{
  return maxm86161_i2c_write_byte_data(address, data);
}

/**************************************************************************//**
 * @brief Read from Maxim register.
 *****************************************************************************/
uint8_t maxm86161_i2c_read_from_register(uint8_t address)
{
  uint8_t data;
  maxm86161_i2c_read_byte_data(address, &data);
  return data;
}

/**************************************************************************//**
 * @brief block write to maxim
 * Block writes should never be used.
 *****************************************************************************/
uint8_t maxm86161_i2c_block_write( uint8_t address, uint8_t length, uint8_t *values)
{
  return maxm86161_i2c_write_i2c_block_data(address,  length, values);
}

/**************************************************************************//**
 * @brief Block read from Maxim.
 *****************************************************************************/
uint8_t maxm86161_i2c_block_read(uint8_t address, uint16_t length, uint8_t *values)
{
	return maxm86161_i2c_read_i2c_block_data(address, length, values);
}

/**************************************************************************//**
 * @brief Write to Maxim i2c.
 *****************************************************************************/
uint8_t maxm86161_i2c_write_byte_data(uint8_t address, uint8_t data)
{
	bool ret ;
	uint8_t  len = 2;
	//ret = I2Cdev_writeBytes(MAXM86161_SLAVE_ADDRESS, address, len, &data);
	ret = I2Cdev_writeByte(MAXM86161_SLAVE_ADDRESS, address,  data);
	
  if (ret != NRF_SUCCESS) {
    return false;
  }

  return true;
}

/**************************************************************************//**
 * @brief read byte from maxim i2c.
 *****************************************************************************/
uint8_t maxm86161_i2c_read_byte_data(uint8_t address, uint8_t *data)
{
	uint8_t ret;
	uint8_t len = 1;
	ret = I2Cdev_readBytes(MAXM86161_SLAVE_ADDRESS, address, len, data);
	
	if (ret != NRF_SUCCESS) {
    return false;
  }

  return true;
}

/**************************************************************************//**
 * @brief Write block data to Maxim i2c.
 *****************************************************************************/
uint8_t maxm86161_i2c_write_i2c_block_data(uint8_t address, uint8_t length, uint8_t * data)
{
	uint8_t ret ;
	uint8_t  len = length + 1;
	ret = I2Cdev_writeBytes(MAXM86161_SLAVE_ADDRESS, address, len, data);
	
   if (ret != NRF_SUCCESS) {
    return false;
  }

  return true;
}

/**************************************************************************//**
 * @brief read block data from to maxim i2c.
 *****************************************************************************/
uint8_t maxm86161_i2c_read_i2c_block_data(uint8_t address, uint16_t length, uint8_t* data)
{
	uint8_t ret;
	uint8_t len = length;
	ret = I2Cdev_readBytes(MAXM86161_SLAVE_ADDRESS, address, len, data);

  if (ret != NRF_SUCCESS) {
    return false;
  }

  return true;
}

/**************************************************************************//**
 * @brief Identify maxm86161 parts
 *****************************************************************************/
uint8_t maxm86161_hrm_identify_part(void)
{
  int32_t valid_part = 0;
   
	uint8_t retid ;
  //maxm86161_i2c_read_from_register(MAXM86161_REG_PART_ID, part_id);
	I2Cdev_readByte(MAXM86161_SLAVE_ADDRESS, MAXM86161_REG_PART_ID, &retid);
  // Static HRM/SpO2 supports all maxm86161 parts
	/*
  switch (*part_id) {
    case 0x36:
      valid_part = 1;
      break;
    default:
      valid_part = 0;
      break;
  }
	*/

  return retid;
}
