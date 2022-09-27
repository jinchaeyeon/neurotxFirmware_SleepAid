/*
Arduino-MAX30100 oximetry / heart rate integrated sensor library
Copyright (C) 2016  OXullo Intersecans <x@brainrapers.org>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef MAX30100_REGISTERS_H
#define MAX30100_REGISTERS_H

#include <stdint.h>
#include "compiler_abstraction.h"
#include "nrf.h"
#include "I2Cdev.h"

#ifdef __cplusplus
	extern "C" {
#endif

#define MAX86150_ADDRESS          0x5E //7-bit I2C Address

#define I2C_BUFFER_LENGTH 32

bool MAX86150_begin(void);

//uint32_t getRed(void); //Returns immediate red value
//uint32_t getIR(void); //Returns immediate IR value
//int32_t getECG(void); //Returns immediate ECG value
//bool safeCheck(uint8_t maxTimeToCheck); //Given a max amount of time, check for new data

// Configuration
void MAX86150_softReset(void);
void MAX86150_shutDown(void);
void MAX86150_wakeUp(void);

void MAX86150_setLEDMode(uint8_t mode);

void MAX86150_setADCRange(uint8_t adcRange);
void MAX86150_setSampleRate(uint8_t sampleRate);
void MAX86150_setPulseWidth(uint8_t pulseWidth);

void MAX86150_setPulseAmplitudeRed(uint8_t value);
void MAX86150_setPulseAmplitudeIR(uint8_t value);
void MAX86150_setPulseAmplitudeProximity(uint8_t value);

void MAX86150_setProximityThreshold(uint8_t threshMSB);

//Multi-led configuration mode (page 22)
void MAX86150_enableSlot(uint8_t slotNumber, uint8_t device); //Given slot number, assign a device to slot
void MAX86150_disableSlots(void);

// Data Collection

//Interrupts (page 13, 14)
uint8_t MAX86150_getINT1(void); //Returns the main interrupt group
uint8_t MAX86150_getINT2(void); //Returns the temp ready interrupt
void MAX86150_enableAFULL(void); //Enable/disable individual interrupts
void MAX86150_disableAFULL(void);
void MAX86150_enableDATARDY(void);
void MAX86150_disableDATARDY(void);
void MAX86150_enableALCOVF(void);
void MAX86150_disableALCOVF(void);
void MAX86150_enablePROXINT(void);
void MAX86150_disablePROXINT(void);
void MAX86150_enableDIETEMPRDY(void);
void MAX86150_disableDIETEMPRDY(void);

//FIFO Configuration (page 18)
void MAX86150_setFIFOAverage(uint8_t samples);
void MAX86150_enableFIFORollover(void);
void MAX86150_disableFIFORollover(void);
void MAX86150_setFIFOAlmostFull(uint8_t samples);

//FIFO Reading
uint16_t MAX86150_check(void); //Checks for new data and fills FIFO
uint8_t MAX86150_available(void); //Tells caller how many new samples are available (head - tail)
void MAX86150_nextSample(void); //Advances the tail of the sense array
uint32_t MAX86150_getFIFORed(void); //Returns the FIFO sample pointed to by tail
uint32_t MAX86150_getFIFOIR(void); //Returns the FIFO sample pointed to by tail
int32_t MAX86150_getFIFOECG(void); //Returns the FIFO sample pointed to by tail

uint8_t MAX86150_getWritePointer(void);
uint8_t MAX86150_getReadPointer(void);
void MAX86150_clearFIFO(void); //Sets the read/write pointers to zero

//Proximity Mode Interrupt Threshold
void MAX86150_setPROXINTTHRESH(uint8_t val);

// Die Temperature
float MAX86150_readTemperature(void);
float MAX86150_readTemperatureF(void);

// Detecting ID/Revision
uint8_t MAX86150_getRevisionID(void);
uint8_t MAX86150_readPartID(void);
uint8_t MAX86150_readRegLED(void);

// Setup the IC with user selectable settings
//void setup(byte powerLevel = 0x1F, byte sampleAverage = 4, byte ledMode = 3, int sampleRate = 400, int pulseWidth = 411, int adcRange = 4096);
//void MAX86150_setup(uint8_t powerLevel = 0x1F, uint8_t sampleAverage = 1, uint8_t ledModeledMode = 3, int sampleRate = 200, int pulseWidth = 50, int adcRange = 32768);
void MAX86150_setup(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledModeledMode, int sampleRate, int pulseWidth, int adcRange);

void MAX86150_readRevisionID(void);

void MAX86150_bitMask(uint8_t reg, uint8_t mask, uint8_t thing);

#define STORAGE_SIZE 4 //Each long is 4 bytes so limit this to fit on your micro
typedef struct Record
{
  uint32_t red[STORAGE_SIZE];
  uint32_t IR[STORAGE_SIZE];
  int32_t ecg[STORAGE_SIZE];
  uint8_t head;
  uint8_t tail;
}sense_struct; //This is our circular buffer of readings from the sensor

#endif
