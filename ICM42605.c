/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Butterfly default), respectively, and it uses the Butterfly STM32L433CU Breakout Board.
  The ICM42605 is a sensor hub with embedded accel and gyro, here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#include "ICM42605.h"
#include "I2Cdev.h"
#include "app_util_platform.h"
#include "nrf_delay.h"


ICM42605_t  icm42605; ;

float t_gRes, t_aRes;

void ICM42605( uint8_t address)
{
  icm42605.devAddr = address;
}


uint8_t ICM42605_getChipID()
{
  uint8_t c ;
  I2Cdev_readByte(ICM42605_ADDRESS, ICM42605_WHO_AM_I,&c);
  return c;
}


float ICM42605_getAres(uint8_t Ascale) {
	
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    case AFS_2G:
      t_aRes = 2.0f / 32768.0f;
      return t_aRes;
      break;
    case AFS_4G:
      t_aRes = 4.0f / 32768.0f;
      return t_aRes;
      break;
    case AFS_8G:
      t_aRes = 8.0f / 32768.0f;
      return t_aRes;
      break;
    case AFS_16G:
      t_aRes = 16.0f / 32768.0f;
      return t_aRes;
      break;
  }
}

float ICM42605_getGres(uint8_t Gscale) {
	
  switch (Gscale)
  {
    case GFS_15_125DPS:
      t_gRes = 15.125f / 32768.0f;
      return t_gRes;
      break;
    case GFS_31_25DPS:
      t_gRes = 31.25f / 32768.0f;
      return t_gRes;
      break;
    case GFS_62_5DPS:
      t_gRes = 62.5f / 32768.0f;
      return t_gRes;
      break;
    case GFS_125DPS:
      t_gRes = 125.0f / 32768.0f;
      return t_gRes;
      break;
    case GFS_250DPS:
      t_gRes = 250.0f / 32768.0f;
      return t_gRes;
      break;
    case GFS_500DPS:
      t_gRes = 500.0f / 32768.0f;
      return t_gRes;
      break;
    case GFS_1000DPS:
      t_gRes = 1000.0f / 32768.0f;
      return t_gRes;
      break;
    case GFS_2000DPS:
      t_gRes = 2000.0f / 32768.0f;
      return t_gRes;
      break;
  }
}


void ICM42605_reset()
{
  // reset device
  uint8_t temp;
	uint8_t ret;
	
	I2Cdev_readByte(ICM42605_ADDRESS, ICM42605_DEVICE_CONFIG,&temp);
  ret = I2Cdev_writeByte(ICM42605_ADDRESS, ICM42605_DEVICE_CONFIG, temp | 0x01); // Set bit 0 to 1 to reset ICM42605
  nrf_delay_ms(1000); // Wait for all registers to reset
}


uint8_t ICM42605_status()
{
  // reset device
  uint8_t temp;
	
	I2Cdev_readByte(ICM42605_ADDRESS, ICM42605_INT_STATUS, &temp);
  return temp;
}


void ICM42605_init(uint8_t Ascale, uint8_t Gscale, uint8_t t_AODR, uint8_t t_GODR)
{
  uint8_t temp;
	uint8_t g_cfg;
	uint8_t ret;

	I2Cdev_readByte(ICM42605_ADDRESS, ICM42605_PWR_MGMT0,&temp); // make sure not to disturb reserved bit values

	
  ret = I2Cdev_writeByte(ICM42605_ADDRESS, ICM42605_PWR_MGMT0, temp | 0x0F);  // enable gyro and accel in low noise mode

	
	

  I2Cdev_readByte(ICM42605_ADDRESS, ICM42605_GYRO_CONFIG0,&temp);
  ret = I2Cdev_writeByte(ICM42605_ADDRESS, ICM42605_GYRO_CONFIG0, temp | t_GODR | Gscale << 5); // gyro full scale and data rate

  I2Cdev_readByte(ICM42605_ADDRESS, ICM42605_ACCEL_CONFIG0,&temp);
  ret = I2Cdev_writeByte(ICM42605_ADDRESS, ICM42605_ACCEL_CONFIG0, temp | t_AODR | Ascale << 5); // set accel full scale and data rate

	
  I2Cdev_readByte(ICM42605_ADDRESS, ICM42605_GYRO_CONFIG1,&temp);
	ret = I2Cdev_writeByte(ICM42605_ADDRESS, ICM42605_GYRO_CONFIG1, temp | 0xD0); // set temperature sensor low pass filter to 5Hz, use first order gyro filter

  I2Cdev_readByte(ICM42605_ADDRESS, ICM42605_INT_CONFIG,&temp);
  ret = I2Cdev_writeByte(ICM42605_ADDRESS, ICM42605_INT_CONFIG, temp | 0x18 | 0x03 ); // set both interrupts active high, push-pull, pulsed

  I2Cdev_readByte(ICM42605_ADDRESS, ICM42605_INT_CONFIG1,&temp);
  ret = I2Cdev_writeByte(ICM42605_ADDRESS, ICM42605_INT_CONFIG1, temp & ~(0x10) ); // set bit 4 to zero for proper function of INT1 and INT2
 
  I2Cdev_readByte(ICM42605_ADDRESS, ICM42605_INT_SOURCE0,&temp);
  ret = I2Cdev_writeByte(ICM42605_ADDRESS, ICM42605_INT_SOURCE0, temp | 0x08 ); // route data ready interrupt to INT1
 
  I2Cdev_readByte(ICM42605_ADDRESS, ICM42605_INT_SOURCE3,&temp);
  ret = I2Cdev_writeByte(ICM42605_ADDRESS, ICM42605_INT_SOURCE3, temp | 0x01 ); // route AGC interrupt interrupt to INT2

  // Select Bank 4
  I2Cdev_readByte(ICM42605_ADDRESS, ICM42605_REG_BANK_SEL,&temp);
  ret = I2Cdev_writeByte(ICM42605_ADDRESS, ICM42605_REG_BANK_SEL, temp | 0x04 ); // select Bank 4

  I2Cdev_readByte(ICM42605_ADDRESS, ICM42605_APEX_CONFIG5,&temp);
  ret = I2Cdev_writeByte(ICM42605_ADDRESS, ICM42605_APEX_CONFIG5, temp & ~(0x07) ); // select unitary mounting matrix

  I2Cdev_readByte(ICM42605_ADDRESS, ICM42605_REG_BANK_SEL,&temp);
  ret = I2Cdev_writeByte(ICM42605_ADDRESS, ICM42605_REG_BANK_SEL, temp & ~(0x07) ); // select Bank 0
}

/*
void ICM42605_selfTest()
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int16_t accelPTest[3] = {0, 0, 0}, accelNTest[3] = {0, 0, 0}, gyroPTest[3] = {0, 0, 0}, gyroNTest[3] = {0, 0, 0};
  int16_t accelNom[3] = {0, 0, 0}, gyroNom[3] = {0, 0, 0};

  readData(temp);
  accelNom[0] = temp[4];
  accelNom[1] = temp[5];
  accelNom[2] = temp[6];
  gyroNom[0]  = temp[1];
  gyroNom[1]  = temp[2];
  gyroNom[2]  = temp[3];

  I2Cdev_writeByte(ICM42605_ADDRESS, ICM42605_CTRL5_C, 0x01); // positive accel self test
  delay(100); // let accel respond
  readData(temp);
  accelPTest[0] = temp[4];
  accelPTest[1] = temp[5];
  accelPTest[2] = temp[6];

  I2Cdev_writeByte(ICM42605_ADDRESS, ICM42605_CTRL5_C, 0x03); // negative accel self test
  delay(100); // let accel respond
  readData(temp);
  accelNTest[0] = temp[4];
  accelNTest[1] = temp[5];
  accelNTest[2] = temp[6];

  I2Cdev_writeByte(ICM42605_ADDRESS, ICM42605_CTRL5_C, 0x04); // positive gyro self test
  delay(100); // let gyro respond
  readData(temp);
  gyroPTest[0] = temp[1];
  gyroPTest[1] = temp[2];
  gyroPTest[2] = temp[3];

  I2Cdev_writeByte(ICM42605_ADDRESS, ICM42605_CTRL5_C, 0x0C); // negative gyro self test
  delay(100); // let gyro respond
  readData(temp);
  gyroNTest[0] = temp[1];
  gyroNTest[1] = temp[2];
  gyroNTest[2] = temp[3];

  I2Cdev_writeByte(ICM42605_ADDRESS, ICM42605_CTRL5_C, 0x00); // normal mode
  delay(100); // let accel and gyro respond

  Serial.println("Accel Self Test:");
  Serial.print("+Ax results:"); Serial.print(  (accelPTest[0] - accelNom[0]) * _aRes * 1000.0); Serial.println(" mg");
  Serial.print("-Ax results:"); Serial.println((accelNTest[0] - accelNom[0]) * _aRes * 1000.0);
  Serial.print("+Ay results:"); Serial.println((accelPTest[1] - accelNom[1]) * _aRes * 1000.0);
  Serial.print("-Ay results:"); Serial.println((accelNTest[1] - accelNom[1]) * _aRes * 1000.0);
  Serial.print("+Az results:"); Serial.println((accelPTest[2] - accelNom[2]) * _aRes * 1000.0);
  Serial.print("-Az results:"); Serial.println((accelNTest[2] - accelNom[2]) * _aRes * 1000.0);
  Serial.println("Should be between 90 and 1700 mg");

  Serial.println("Gyro Self Test:");
  Serial.print("+Gx results:"); Serial.print((gyroPTest[0] - gyroNom[0]) * _gRes); Serial.println(" dps");
  Serial.print("-Gx results:"); Serial.println((gyroNTest[0] - gyroNom[0]) * _gRes);
  Serial.print("+Gy results:"); Serial.println((gyroPTest[1] - gyroNom[1]) * _gRes);
  Serial.print("-Gy results:"); Serial.println((gyroNTest[1] - gyroNom[1]) * _gRes);
  Serial.print("+Gz results:"); Serial.println((gyroPTest[2] - gyroNom[2]) * _gRes);
  Serial.print("-Gz results:"); Serial.println((gyroNTest[2] - gyroNom[2]) * _gRes);
  Serial.println("Should be between 20 and 80 dps");
  delay(2000);


}

*/
void ICM42605_offsetBias(float * dest1, float * dest2)
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int32_t sum[7] = {0, 0, 0, 0, 0, 0, 0};

 // Serial.println("Calculate accel and gyro offset biases: keep sensor flat and motionless!");
 // delay(4000);

  for (int ii = 0; ii < 128; ii++)
  {
    ICM42605_readData(temp);
    sum[1] += temp[1];
    sum[2] += temp[2];
    sum[3] += temp[3];
    sum[4] += temp[4];
    sum[5] += temp[5];
    sum[6] += temp[6];
    nrf_delay_ms(50);
  }

  dest1[0] = sum[1] * t_aRes / 128.0f;
  dest1[1] = sum[2] * t_aRes / 128.0f;
  dest1[2] = sum[3] * t_aRes / 128.0f;
  dest2[0] = sum[4] * t_gRes / 128.0f;
  dest2[1] = sum[5] * t_gRes / 128.0f;
  dest2[2] = sum[6] * t_gRes / 128.0f;

  if (dest1[0] > 0.8f)  {
    dest1[0] -= 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
  }
  if (dest1[0] < -0.8f) {
    dest1[0] += 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
  }
  if (dest1[1] > 0.8f)  {
    dest1[1] -= 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
  }
  if (dest1[1] < -0.8f) {
    dest1[1] += 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
  }
  if (dest1[2] > 0.8f)  {
    dest1[2] -= 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
  }
  if (dest1[2] < -0.8f) {
    dest1[2] += 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
  }

}


void ICM42605_readData(int16_t * destination)
{
	uint8_t i;
  uint8_t rawData[14];  // x/y/z accel register data stored here
  I2Cdev_readBytes(ICM42605_ADDRESS, ICM42605_TEMP_DATA1, 14, rawData);  // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
  destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
  destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
  destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
  destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ;

	/*
	for ( i = 0 ; i <=13 ; i++ ) 
	{
		   I2Cdev_readByte(ICM42605_ADDRESS, ICM42605_TEMP_DATA1,  &rawData[i]);
	}
	
	//destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  //destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  //destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
  //destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
  //destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
  //destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
  //destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ;
	
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
  destination[3] = ((int16_t)rawData[6] << 8) | rawData[5] ;
  destination[4] = ((int16_t)rawData[9] << 8) | rawData[8] ;
  destination[5] = ((int16_t)rawData[11] << 8) | rawData[10] ;
  destination[6] = ((int16_t)rawData[13] << 8) | rawData[12] ;
  */
	
}
