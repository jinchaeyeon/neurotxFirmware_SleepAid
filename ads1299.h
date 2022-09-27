/* Copyright (c) 2017 Musa Mahmood
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
 
/** @file
 *
 * @brief Functions for initializing and controlling Texas Instruments ADS1299 analog front-end.
 */
 
#ifndef ADS1299_H__
#define ADS1299_H__
 
#include <stdint.h>
#include "nrf_drv_spi.h"
#include "boards.h"

#ifdef __cplusplus
	extern "C" {
#endif
/**@SPI STUFF*/

//ADS1299 SPI Command Definition Byte Assignments
//#define _WAKEUP 0x02 // Wake-up from standby mode
//#define _STANDBY 0x04 // Enter Standby mode
//#define _RESET 0x06 // Reset the device registers to default

#define _RREG    0x20;		//Read n nnnn registers starting at address r rrrr
                                //first byte 001r rrrr (2xh)(2) - second byte 000n nnnn(2)
#define _WREG    0x40;		//Write n nnnn registers starting at address r rrrr
                                //first byte 010r rrrr (2xh)(2) - second byte 000n nnnn(2)

#define _START 0x08 // Start and restart (synchronize) conversions
#define _STOP 0x0A // Stop conversion
#define _RDATAC 0x10 // Enable Read Data Continuous mode (default mode at power-up)

//This mode is the default mode at power-up.
#define _SDATAC 0x11 // Stop Read Data Continuous mode
#define _RDATA 0x12 // Read data by command; supports multiple read back

//ADS1299 Register Addresses
/*#define ID			0x00
#define CONFIG1		0x01
#define CONFIG2		0x02
#define LOFF		0x03
#define CH1SET		0x04
#define CH2SET		0x05
#define RLDSENS		0x06
#define LOFFSENS    0x07
#define LOFFSTAT    0x08
#define RESP1	    0x09
#define RESP2	    0x0A*/

//ADS1299 Register Addresses
#define ID      0x00
#define CONFIG1 0x01
#define CONFIG2 0x02
#define CONFIG3 0x03
#define LOFF 0x04
#define CH1SET 0x05
#define CH2SET 0x06
#define CH3SET 0x07
#define CH4SET 0x08
#define CH5SET 0x09
#define CH6SET 0x0A
#define CH7SET 0x0B
#define CH8SET 0x0C
#define BIAS_SENSP 0x0D
#define BIAS_SENSN 0x0E
#define LOFF_SENSP 0x0F
#define LOFF_SENSN 0x10
#define LOFF_FLIP 0x11
#define LOFF_STATP 0x12
#define LOFF_STATN 0x13
#define GPIO 0x14
#define MISC1 0x15
#define MISC2 0x16
#define CONFIG4 0x17

#define OPENBCI_NCHAN (8)  // number of EEG channels
// CHANNEL SETTINGS 
#define POWER_DOWN      (0)
#define GAIN_SET        (1)
#define INPUT_TYPE_SET  (2)
#define BIAS_SET        (3)
#define SRB2_SET        (4)
#define SRB1_SET        (5)
#define YES      	(0x01)
#define NO      	(0x00)

//gainCode choices
#define ADS_GAIN06 0x00//(0b00000000)	// 0x00
#define ADS_GAIN01 0x10//(0b00010000)	// 0x10
#define ADS_GAIN02 0x20//(0b00100000)	// 0x20
#define ADS_GAIN03 0x30//(0b00110000)	// 0x30
#define ADS_GAIN04 0x40//(0b01000000)	// 0x40
#define ADS_GAIN08 0x50//(0b01010000)	// 0x50
#define ADS_GAIN12 0x60//(0b01100000)	// 0x60

//inputType choices
#define ADSINPUT_NORMAL 0x00//(0b00000000)
#define ADSINPUT_SHORTED 0x01//(0b00000001)
#define ADSINPUT_BIAS_MEAS 0x02//(0b00000010) //BIAS = RLD
#define ADSINPUT_MVDD 0x03//(0b00000011)
#define ADSINPUT_TEMP 0x04//(0b00000100)
#define ADSINPUT_TESTSIG 0x05//(0b00000101)
#define ADSINPUT_BIAS_DRP 0x06//(0b00000110)
#define ADSINPUT_BIAS_DRN 0x07//(0b00000111)
#define ADSINPUT_BIAS_DRPN 0x08//(0b00001000)
#define ADSINPUT_ROUTE_3TO_CH1 0x09//(0b00001001)


//??
//test signal choices...ADS1299 datasheet page 41
#define ADSTESTSIG_AMP_1X 0x00//(0b00000000)
#define ADSTESTSIG_AMP_2X 0x04//(0b00000100)
#define ADSTESTSIG_PULSE_SLOW 0x00//(0b00000000)
#define ADSTESTSIG_PULSE_FAST 0x01//(0b00000001)
#define ADSTESTSIG_DCSIG 0x03//(0b00000011)
#define ADSTESTSIG_NOCHANGE 0xff//(0b11111111)

//Lead-off signal choices
#define LOFF_MAG_6NA 0x00//(0b00000000)
#define LOFF_MAG_22NA 0x04//(0b00000100)
#define LOFF_MAG_6UA 0x08//(0b00001000)
#define LOFF_MAG_22UA 0x0c//(0b00001100)
#define LOFF_FREQ_DC 0x00//(0b00000000)
#define LOFF_FREQ_FS_4 0x01//(0b00000001)
#define PCHAN (0)
#define NCHAN (1)
#define OFF (0)
#define ON (1)


// used for channel settings
#define ACTIVATE_SHORTED (2)
#define ACTIVATE (1)
#define DEACTIVATE (0)

#define PCKT_START 0xA0	// prefix for data packet error checking
#define PCKT_END 0xC0	// postfix for data packet error checking

/**************************************************************************************************************************************************
*              Function Prototypes ADS1299 																																																			*
**************************************************************************************************************************************************/
void ADS1299_spi_init(void);
void ADS1299_initialize(void);

//ADS1299 SPI Command Definitions 
//System Commands
void ADS1299_WAKEUP(void);  // get out of low power mode
void ADS1299_STANDBY(void); // go into low power mode
void ADS1299_RESET(void);   // set all register values to default
void ADS1299_START(void);   // start data acquisition
void ADS1299_STOP(void);    // stop data acquisition

//Data Read Commands
void ADS1299_RDATAC(void);
void ADS1299_SDATAC(void);
void ADS1299_RDATA(void);

//Register Read/Write Commands

uint8_t ADS1299_RREG(uint8_t);                // read one register
void ADS1299_RREGS(uint8_t, uint8_t);         // read multiple registers
void ADS1299_WREG(uint8_t, uint8_t);          // write one register
void ADS1299_WREGS(uint8_t, uint8_t);         // write multiple registers
uint8_t ADS1299_getDeviceID(void);
void ADS1299_printRegisterName(uint8_t);   // used for verbosity
void ADS1299_updateChannelData(void);   // retrieve data from ADS

void ADS1299_resetADS(void);                      //reset all the ADS1299's settings.  Call however you'd like
void ADS1299_startADS(void);
void ADS1299_stopADS(void);

void ADS1299_writeChannelSettings(void);
//void ADS1299_writeChannelSettings(int);
void ADS1299_activateChannel(int);    // activate a given channel 1-8
void ADS1299_reportDefaultChannelSettings(void);
void ADS1299_deactivateChannel(int);  //disable given channel 1-8

void ADS1299_configureLeadOffDetection(uint8_t, uint8_t);
void ADS1299_changeChannelLeadOffDetection(void);
void ADS1299_configureInternalTestSignal(uint8_t, uint8_t);  

bool ADS1299_isDataAvailable(void);
void ADS1299_writeADSchannelData(void);
void ADS1299_writeADSstatData(void);  // KDS add for lead off stat
void ADS1299_printAllRegisters(void);
//void setSRB1(boolean desired_state);
void ADS1299_printDeviceID(void);

extern uint8_t bit24ChannelData[24];    // array to hold raw channel data

#endif // ADS1299_H__
