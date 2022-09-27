/* Copyright (c) 2016 Musa Mahmood
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

#include "ads1299.h"
#include "app_error.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "app_util_platform.h"
#include "nrf_log.h"
#include "nrf_delay.h"
/**@headers for µs delay:*/
#include <stdio.h> 
#include "compiler_abstraction.h"
#include "nrf.h"

int stat;			// used to hold the status register
uint8_t statData[3];			// KDS change for Lead off transfer
uint8_t regData [24];	        // array is used to mirror register data
long channelData [9];	// array used when reading channel data as ints
uint8_t bit24ChannelData[24];    // array to hold raw channel data
char channelSettings[8][6];  // array to hold current channel settings
//char defaultChannelSettings[6];  // default channel settings
char defaultChannelSettings[4];  // default channel settings
//bool useInBias[8];        // used to remember if we were included in Bias before channel power down
//bool useSRB1;             // used to keep track of if we are using SRB1
//bool useSRB2[8];          // used to remember if we were included in SRB2 before channel power down
char leadOffSettings[8][2];  // used to control on/off of impedance measure for P and N side of each channel
bool verbosity;		 // turn on/off Serial feedback

bool isRunning;

static uint8_t tx_data_spi[24];
static uint8_t rx_data_spi[24];
static uint8_t txrx_size=0;
static uint8_t delay_cnt=0;

uint8_t bitSet(uint8_t value, uint8_t bit_number)
{
  value |= 1<<bit_number;
  return value;
}

uint8_t bitClear(uint8_t value, uint8_t bit_number)
{
  value &= ~(1<<bit_number);
  return value;
}

bool bitRead(uint32_t value, uint8_t bit)
{ uint32_t bit_state = 1<<bit;
  if(value & bit_state) return 1;
  else return 0;
}

/**@SPI HANDLERS:
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
		/*switch (p_event->type) {
				case NRF_DRV_SPI_EVENT_DONE:
					break;
				default:
					break;
		}*/
    //NRF_LOG_PRINTF(" >>> Transfer completed.\r\n");
}

/**@INITIALIZE SPI INSTANCE */
#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE); //SPI INSTANCE

void ADS1299_spi_init(void) {
		nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
		//spi_config.bit_order						= NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
		//SCLK = 1MHz is right speed because fCLK = (1/2)*SCLK, and fMOD = fCLK/4, and fMOD MUST BE 128kHz. Do the math.
		spi_config.frequency						= NRF_DRV_SPI_FREQ_1M;
		spi_config.irq_priority					= APP_IRQ_PRIORITY_LOW;
		spi_config.mode									= NRF_DRV_SPI_MODE_1; //CPOL = 0 (Active High); CPHA = TRAILING (1)
		spi_config.miso_pin 						= ADS1299_SPI_MISO_PIN;
		spi_config.sck_pin 							= ADS1299_SPI_SCLK_PIN;
		spi_config.mosi_pin 						= ADS1299_SPI_MOSI_PIN;
		spi_config.ss_pin								= ADS1299_SPI_CS_PIN;   //okson
		spi_config.orc									= 0x55;
		APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler,NULL));
	//	NRF_LOG_PRINTF(" SPI Initialized..\r\n");
		printf(" SPI Initialized..\r\n");
}

void ADS1299_initialize(void){

// recommended power up sequence requiers Tpor (~32mS)			
  nrf_delay_ms(50);				
  nrf_gpio_pin_clear(ADS1299_RESET_PIN);
  nrf_delay_us(4);	// toggle reset pin
  nrf_gpio_pin_set(ADS1299_RESET_PIN);
  //delayMicroseconds(20);	// recommended to wait 18 Tclk before using device (~8uS);    
  nrf_delay_us(20);
	
  nrf_delay_ms(100);
  ADS1299_resetADS();
	nrf_delay_ms(100);

	ADS1299_WREG(CONFIG1,0xF6);  // Sample rate 250
	//WREG(CONFIG1,0x00);  // Sample rate 125
	nrf_delay_ms(100);
	ADS1299_WREG(CONFIG2,0xC2);   //intetnal test source, Not USE, default : 1100 0000
	nrf_delay_ms(100);
	
	ADS1299_WREG(CONFIG3,0xEC);   //intetnal test source, Not USE, default : 1100 0000
	nrf_delay_ms(100);

	ADS1299_WREG(LOFF,0x03);
	nrf_delay_ms(10);
	
	ADS1299_WREG(CH1SET,0x60);  //GAIN 12, normal
	nrf_delay_ms(10);
	ADS1299_WREG(CH2SET,0x60);  //original 0x60 
	nrf_delay_ms(10);
	ADS1299_WREG(CH3SET,0xF1);  //original 0x60 
	nrf_delay_ms(10);
	ADS1299_WREG(CH4SET,0xF1);  //original 0x60 
	nrf_delay_ms(10);
	ADS1299_WREG(CH5SET,0xF1);  //original 0x60 
	nrf_delay_ms(10);
	ADS1299_WREG(CH6SET,0xF1);  //original 0x60 
	nrf_delay_ms(10);
	ADS1299_WREG(CH7SET,0xF1);  //original 0x60 
	nrf_delay_ms(10);
	ADS1299_WREG(CH8SET,0xF1);  //original 0x60 
	nrf_delay_ms(10);
	
	//ADS1299_WREG(RLDSENS,0x00); 
	//nrf_delay_ms(10);
	ADS1299_WREG(BIAS_SENSP,0x01); 
	nrf_delay_ms(10);
	
	ADS1299_WREG(BIAS_SENSN,0x01); 
	nrf_delay_ms(10);
	
	ADS1299_WREG(LOFF_SENSP,0x02); 
	nrf_delay_ms(10);
	
	ADS1299_WREG(LOFF_SENSN,0x00); 
	nrf_delay_ms(10);
	
	ADS1299_WREG(LOFF_FLIP,0x00); 
	nrf_delay_ms(10);
	
	ADS1299_WREG(LOFF_STATP,0x00); 
	nrf_delay_ms(10);
	
	ADS1299_WREG(LOFF_STATN,0x00); 
	nrf_delay_ms(10);
	
	ADS1299_WREG(GPIO,0x0F); 
	nrf_delay_ms(10);
	
	ADS1299_WREG(MISC1,0x00); 
	nrf_delay_ms(10);
	
	ADS1299_WREG(MISC2,0x00); 
	nrf_delay_ms(10);
	
	ADS1299_WREG(CONFIG4,0x00); 
	nrf_delay_ms(10);
	
	
	
	//ADS1299_WREG(LOFFSENS,0x00);  
	//nrf_delay_ms(10);
	
	//ADS1299_WREG(RESP2,0x03);  
	//nrf_delay_ms(10);

  verbosity = false;      // when verbosity is true, there will be Serial feedback
};

//reset all the ADS1299's settings.  Call however you'd like.  Stops all data acquisition
void ADS1299_resetADS(void)
{
  //RESET();             // send RESET command to default all registers
  ADS1299_SDATAC();            // exit Read Data Continuous mode to communicate with ADS
  //nrf_delay_ms(10);
};

void ADS1299_reportDefaultChannelSettings(void){

    //Serial.write(defaultChannelSettings[POWER_DOWN] + '0');        // on = NO, off = YES
    //Serial.write((defaultChannelSettings[GAIN_SET] >> 4) + '0');     // Gain setting
    //Serial.write(defaultChannelSettings[INPUT_TYPE_SET] +'0');// input muxer setting
    //Serial.write(defaultChannelSettings[BIAS_SET] + '0');    // include in bias generation
    //Serial.write(defaultChannelSettings[SRB2_SET] + '0');       // connect this P side to SRB2
    //Serial.write(defaultChannelSettings[SRB1_SET] + '0');        // don't use SRB1
    printf("%d",defaultChannelSettings[POWER_DOWN]);
    printf("%d",defaultChannelSettings[GAIN_SET] >> 4);
    printf("%d",defaultChannelSettings[INPUT_TYPE_SET]);
    printf("%d",defaultChannelSettings[BIAS_SET]);
}

void ADS1299_writeChannelSettings(void){
  uint8_t setting;
  //boolean use_SRB1 = false;
//proceed...first, disable any data collection
  ADS1299_SDATAC(); 
  nrf_delay_ms(1);      // exit Read Data Continuous mode to communicate with ADS

  for(uint8_t i=0; i<2; i++){ // write 8 channel settings
    setting = 0x00;
    if(channelSettings[i][POWER_DOWN] == YES) {setting |= 0x80;}
    setting |= channelSettings[i][GAIN_SET]; // gain
    setting |= channelSettings[i][INPUT_TYPE_SET]; // input code
	/*
    if(channelSettings[i][SRB2_SET] == YES){
      setting |= 0x08; // close this SRB2 switch
      useSRB2[i] = true;
    }else{
      useSRB2[i] = false;
    }
	*/
    ADS1299_WREG(CH1SET+i, setting);  // write this channel's register settings
    /*
      // add or remove from inclusion in BIAS generation
      setting = RREG(BIAS_SENSP);       //get the current P bias settings
      if(channelSettings[i][BIAS_SET] == YES){
        bitSet(setting,i);    //set this channel's bit to add it to the bias generation
        useInBias[i] = true;
      }else{
        bitClear(setting,i);  // clear this channel's bit to remove from bias generation
        useInBias[i] = false;
      }
      WREG(BIAS_SENSP,setting); delay(1); //send the modified byte back to the ADS
      setting = RREG(BIAS_SENSN);       //get the current N bias settings
      if(channelSettings[i][BIAS_SET] == YES){
        bitSet(setting,i);    //set this channel's bit to add it to the bias generation
      }else{
        bitClear(setting,i);  // clear this channel's bit to remove from bias generation
      }
      WREG(BIAS_SENSN,setting); delay(1); //send the modified byte back to the ADS
      
    if(channelSettings[i][SRB1_SET] == YES){
      useSRB1 = true;
    }
	*/
  }
    /*
	if(useSRB1){
      for(int i=0; i<8; i++){
        channelSettings[i][SRB1_SET] = YES;
      }
      WREG(MISC1,0x20);     // close all SRB1 swtiches
    }else{
      for(int i=0; i<8; i++){
        channelSettings[i][SRB1_SET] = NO;
      }
      WREG(MISC1,0x00);
    }
	*/
}

#if 0

void ADS1299_writeChannelSettings(int N){
  uint8_t setting;
  if ((N < 1) || (N > 2)) return;  // must be a legit channel number
  N=N-1;
  //N = constrain(N-1,0,1);  //subtracts 1 so that we're counting from 0, not 1
//proceed...first, disable any data collection
  ADS1299_SDATAC(); 
  nrf_delay_ms(1);      // exit Read Data Continuous mode to communicate with ADS
  
    setting = 0x00;
    if(channelSettings[N][POWER_DOWN] == YES) {setting |= 0x80;}
    setting |= channelSettings[N][GAIN_SET]; // gain
    setting |= channelSettings[N][INPUT_TYPE_SET]; // input code
	/*
    if(channelSettings[N][SRB2_SET] == YES){
      setting |= 0x08; // close this SRB2 switch
      useSRB2[N] = true;  // keep track of SRB2 usage
    }else{
      useSRB2[N] = false;
    }
	*/
  ADS1299_WREG(CH1SET+N, setting);  // write this channel's register settings
    
	/*
      // add or remove from inclusion in BIAS generation
      setting = RREG(BIAS_SENSP);       //get the current P bias settings
      if(channelSettings[N][BIAS_SET] == YES){
        useInBias[N] = true;
        bitSet(setting,N);    //set this channel's bit to add it to the bias generation
      }else{
        useInBias[N] = false;
        bitClear(setting,N);  // clear this channel's bit to remove from bias generation
      }
      WREG(BIAS_SENSP,setting); delay(1); //send the modified byte back to the ADS
      setting = RREG(BIAS_SENSN);       //get the current N bias settings
      if(channelSettings[N][BIAS_SET] == YES){
        bitSet(setting,N);    //set this channel's bit to add it to the bias generation
      }else{
        bitClear(setting,N);  // clear this channel's bit to remove from bias generation
      }
      WREG(BIAS_SENSN,setting); delay(1); //send the modified byte back to the ADS
     
    	 
    if(channelSettings[N][SRB1_SET] == YES){
    for(int i=0; i<8; i++){
      channelSettings[i][SRB1_SET] = YES;
    }
    useSRB1 = true;
      WREG(MISC1,0x20);     // close all SRB1 swtiches
  }
  if((channelSettings[N][SRB1_SET] == NO) && (useSRB1 == true)){
    for(int i=0; i<8; i++){
      channelSettings[i][SRB1_SET] = NO;
    }
    useSRB1 = false;
    WREG(MISC1,0x00);
  }
  */
}
#endif

//  deactivate the given channel...note: stops data colleciton to issue its commands
//  N is the channel number: 1-8
void ADS1299_deactivateChannel(int N)
{
  uint8_t setting;  
  if ((N < 1) || (N > 2)) return;  //check the inputs  
  //proceed...first, disable any data collection
  ADS1299_SDATAC(); nrf_delay_ms(1);      // exit Read Data Continuous mode to communicate with ADS
  //shut down the channel
  N=N-1;
  //N = constrain(N-1,0,1);  //subtracts 1 so that we're counting from 0, not 1
//  reg = CH1SET+(byte)N;           // select the current channel
  setting = ADS1299_RREG(CH1SET+(uint8_t)N); nrf_delay_ms(1); // get the current channel settings
  setting = bitSet(setting,7);              // set bit7 to shut down channel
  channelSettings[N][POWER_DOWN] = YES;  // keep track of channel on/off state
  setting = bitClear(setting,3);    // clear bit3 to disclude from SRB2 
  ADS1299_WREG(CH1SET+(uint8_t)N,setting); 
  nrf_delay_ms(1);     // write the new value to disable the channel
  
  /*
  //remove the channel from the bias generation...
  setting = RREG(BIAS_SENSP); delay(1); //get the current bias settings
  bitClear(setting,N);                  //clear this channel's bit to remove from bias generation
  WREG(BIAS_SENSP,setting); delay(1);   //send the modified byte back to the ADS

  setting = RREG(BIAS_SENSN); delay(1); //get the current bias settings
  bitClear(setting,N);                  //clear this channel's bit to remove from bias generation
  WREG(BIAS_SENSN,setting); delay(1);   //send the modified byte back to the ADS
  */
  
  leadOffSettings[N][0] = leadOffSettings[N][1] = NO; // stop lead off detection 
  ADS1299_changeChannelLeadOffDetection();
}; 

//  Active a channel using stored channelSettings[][]  
//  N is 1 through 8
void ADS1299_activateChannel(int N) 
{
  uint8_t setting;  
   
  if ((N < 1) || (N > 2)) return; //check the inputs
  N=N-1;
  //N = constrain(N-1,0,1);  //shift down by one
  //proceed...first, disable any data collection
  ADS1299_SDATAC(); 
  nrf_delay_ms(1);      // exit Read Data Continuous mode to communicate with ADS
  setting = 0x00;
  channelSettings[N][POWER_DOWN] = NO; // keep track of channel on/off state
  setting |= channelSettings[N][GAIN_SET]; // gain
  setting |= channelSettings[N][INPUT_TYPE_SET]; // input code
  /*
  if(useSRB2[N] == true){channelSettings[N][SRB2_SET] = YES;}else{channelSettings[N][SRB2_SET] = NO;}
  if(channelSettings[N][SRB2_SET] == YES) bitSet(setting,3); // close this SRB2 switch
  */
  ADS1299_WREG(CH1SET+N, setting);
  /*
  // add or remove from inclusion in BIAS generation
    if(useInBias[N]){channelSettings[N][BIAS_SET] = YES;}else{channelSettings[N][BIAS_SET] = NO;}
    setting = RREG(BIAS_SENSP);       //get the current P bias settings
    if(channelSettings[N][BIAS_SET] == YES){
      bitSet(setting,N);    //set this channel's bit to add it to the bias generation
    }else{
      bitClear(setting,N);  // clear this channel's bit to remove from bias generation
    }
    WREG(BIAS_SENSP,setting); delay(1); //send the modified byte back to the ADS
    setting = RREG(BIAS_SENSN);       //get the current N bias settings
    if(channelSettings[N][BIAS_SET] == YES){
      bitSet(setting,N);    //set this channel's bit to add it to the bias generation
    }else{
      bitClear(setting,N);  // clear this channel's bit to remove from bias generation
    }
    WREG(BIAS_SENSN,setting); delay(1); //send the modified byte back to the ADS
    
  setting = 0x00;
  if(useSRB1) setting = 0x20;
  WREG(MISC1,setting);     // close all SRB1 swtiches if desired
  */
};

// Start continuous data acquisition
void ADS1299_startADS(void)
{
  ADS1299_RDATAC();
	nrf_delay_ms(1);   // enter Read Data Continuous mode
	ADS1299_START();        // start the data acquisition
  nrf_delay_ms(1);
  isRunning = true;
}

// Query to see if data is available from the ADS1299...return TRUE is data is available
bool ADS1299_isDataAvailable(void)
{
  return (!(nrf_gpio_pin_read(ADS1299_DRDY_PIN)));
}

// Get ADS channel data when DRDY goes low
void ADS1299_updateChannelData(){
    uint8_t inByte; 
    int byteCounter = 0;

	for(int i=0; i<3; i++)
	{ 
    tx_data_spi[0] = 0x00; 
    nrf_drv_spi_transfer(&spi, tx_data_spi, 1, rx_data_spi, 1); //  read status register (1100 + LOFF_STATP + LOFF_STATN + GPIO[7:4])
    nrf_delay_us(30);
		inByte = rx_data_spi[0];   
    stat = (stat << 8) | inByte;
  }
	
  for(int i = 0; i<8; i++)
  {
    for(int j=0; j<3; j++)
    {   //  read 24 bits of channel data in 8 3 byte chunks
      tx_data_spi[0] = 0x00; 
      nrf_drv_spi_transfer(&spi, tx_data_spi, 1, rx_data_spi, 1);
			nrf_delay_us(30);
      inByte = rx_data_spi[0]; 
	   // if (byteCounter < 6 )   // KDS, Limite 2 channel
          bit24ChannelData[byteCounter] = inByte;  // raw data gets streamed to the radio
      byteCounter++;
      // int data used for SD card write and DSP if you like
      channelData[i] = (channelData[i]<<8) | inByte;
    }
  }
	
// <<<<<<<  THE channelData ARRAY IS USED TO WRITE TO SD CARD >>>>>>>>>>
// need to convert 24bit to 32bit if using a filter
  for(int i=0; i<8; i++){     // convert 3 byte 2's compliment to 4 byte 2's compliment 
    if(bitRead(channelData[i],23) == 1){  
      channelData[i] |= 0xFF000000;
    }else{
      channelData[i] &= 0x00FFFFFF;
    }
  }
}

//write as binary each channel's 3 bytes of data
void ADS1299_writeADSchannelData(void)
{
  //DS Add for lead off status	
  uint8_t ch_off;
  
  //ch_off = bit24ChannelData[22];
  
//  if (ch_off == 16)
//  {
    printf("%c",bit24ChannelData[0]);
    printf("%c",bit24ChannelData[1]);
    printf("%c",bit24ChannelData[2]);
    
    printf("%c",bit24ChannelData[3]);
    printf("%c",bit24ChannelData[4]);
    printf("%c",bit24ChannelData[5]);
    
	  //Serial.write(bit24ChannelData[0]); 
	  //Serial.write(bit24ChannelData[1]); 
	  //Serial.write(bit24ChannelData[2]); 
	  
	  //Serial.write(bit24ChannelData[3]); 
	  //Serial.write(bit24ChannelData[4]); 
	  //Serial.write(bit24ChannelData[5]); 
//  }
//  else
//  {
//	  Serial.write(0x7F); 
//	  Serial.write(0xFF); 
//	  Serial.write(0xFF); 	  
 // }
 /*
  for (int i = 3; i < 8; i++)
  {
    Serial.write(bit24ChannelData[i]); 
  }
  */
  
}

// Stop the continuous data acquisition
void ADS1299_stopADS(void)
{
  ADS1299_STOP();
  nrf_delay_ms(1);       // start the data acquisition
  ADS1299_SDATAC();
  nrf_delay_ms(1);       // exit Read Data Continuous mode to communicate with ADS
  isRunning = false;
}

void ADS1299_changeChannelLeadOffDetection(void){
  //uint8_t P_setting = ADS1299_RREG(LOFF_SENSP);
  //uint8_t N_setting = ADS1299_RREG(LOFF_SENSN);
  /*
  for(int i=0; i<8;i++){
    if(leadOffSettings[i][0] == YES){
      bitSet(P_setting,i);
    }else{
      bitClear(P_setting,i);
    }
    if(leadOffSettings[i][1] == YES){
      bitSet(N_setting,i);
    }else{
      bitClear(N_setting,i);
    }
  }
   ADS1299_WREG(LOFF_SENSP,P_setting);
   ADS1299_WREG(LOFF_SENSN,N_setting);
  */
}

void ADS1299_configureLeadOffDetection(uint8_t amplitudeCode, uint8_t freqCode)
{
	amplitudeCode &= 0x0c;//0b00001100;  //only these two bits should be used
	freqCode &= 0x03;//0b00000011;  //only these two bits should be used

	uint8_t setting = 0x00;	
	setting = ADS1299_RREG(LOFF); //get the current bias settings
	
	//reconfigure the byte to get what we want
	setting &= 0xf0;//0b11110000;  //clear out the four ls bits
	setting |= amplitudeCode;  //set the amplitude
	setting |= freqCode;    //set the frequency
	
	//send the config byte back to the hardware
	ADS1299_WREG(LOFF,setting);   //send the modified byte back to the ADS
	nrf_delay_ms(1);
}

//Configure the test signals that can be inernally generated by the ADS1299
void ADS1299_configureInternalTestSignal(uint8_t amplitudeCode, uint8_t freqCode)
{
	if (amplitudeCode == ADSTESTSIG_NOCHANGE) amplitudeCode = ADS1299_RREG(CONFIG2) & 0x04;    //(0b00000100));
	if (freqCode == ADSTESTSIG_NOCHANGE) freqCode = ADS1299_RREG(CONFIG2) & 0x03;//(0b00000011));
	freqCode &= 0x03;//0b00000011;  		//only the last two bits are used
	amplitudeCode &= 0x04;//0b00000100;  	//only this bit is used
	uint8_t message = 0xd0 | freqCode | amplitudeCode;//0b11010000 | freqCode | amplitudeCode;  //compose the code
	
	ADS1299_WREG(CONFIG2,message); 
	nrf_delay_ms(1);	
}

//  <<<<<<  SYSTEM COMMANDS  >>>>>>
/*
void ADS1299_WAKEUP() {

	tx_data_spi[0] = _WAKEUP;
	
	nrf_drv_spi_transfer(&spi, tx_data_spi, 1, rx_data_spi, 1);
	
	nrf_delay_us(3);  //must wait 4 tCLK cycles before sending another command (Datasheet, pg. 35)    
}
*/

/*
void ADS1299_STANDBY() {   // only allowed to send WAKEUP after sending STANDBY

	tx_data_spi[0] = _STANDBY;
	
	nrf_drv_spi_transfer(&spi, tx_data_spi, 1, rx_data_spi, 1);
}
*/

/*
void ADS1299_RESET() {     // reset all the registers to default settings
  uint8_t tx_data_spi;
	uint8_t rx_data_spi;

	tx_data_spi = _RESET;
	
	nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1);
	
	nrf_delay_us(3);    //must wait 18 tCLK cycles to execute this command (Datasheet, pg. 35)
}
*/

void ADS1299_START(void) {     //start data conversion 
	
	tx_data_spi[0] = _START;

	nrf_drv_spi_transfer(&spi, tx_data_spi, 1, rx_data_spi, 1);
}

void ADS1299_STOP(void) {      //stop data conversion

	tx_data_spi[0] = _STOP;

	nrf_drv_spi_transfer(&spi, tx_data_spi, 1, rx_data_spi, 1);
}

void ADS1299_RDATAC(void) {    //read data continuous mode

	tx_data_spi[0] = _RDATAC;

	nrf_drv_spi_transfer(&spi, tx_data_spi, 1, rx_data_spi, 1);

  nrf_delay_us(3);   
}

//uint8_t tx_data_spi = 0x11;
//uint8_t rx_data_spi;
void ADS1299_SDATAC(void) {

	tx_data_spi[0] = _SDATAC;

	nrf_drv_spi_transfer(&spi, tx_data_spi, 1, rx_data_spi, 1);

  nrf_delay_us(3);   //must wait 4 tCLK cycles after executing this command (Datasheet, pg. 37)
}

void ADS1299_RDATA(void) {         //  use in Stop Read Continuous mode when DRDY goes low
  uint8_t inByte;            //  to read in one sample of the channels
	
	tx_data_spi[0] = _RDATAC;

	nrf_drv_spi_transfer(&spi, tx_data_spi, 1, rx_data_spi, 1);
	nrf_delay_us(30);
	stat = rx_data_spi[0];        //  read status register (1100 + LOFF_STATP + LOFF_STATN + GPIO[7:4])
    
  for(int i = 0; i<8; i++){
    for(int j=0; j<3; j++){   //  read in the status register and new channel data
      tx_data_spi[0] = 0x00; 
      nrf_drv_spi_transfer(&spi, tx_data_spi, 1, rx_data_spi, 1); //  read status register (1100 + LOFF_STATP + LOFF_STATN + GPIO[7:4])
      nrf_delay_us(30);
			inByte = rx_data_spi[0];   
      channelData[i] = (channelData[i]<<8) | inByte;
    }
  }
  
  for(int i=0; i<8; i++){
    if(bitRead(channelData[i],23) == 1){  // convert 3 byte 2's compliment to 4 byte 2's compliment
      channelData[i] |= 0xFF000000;
    }else{
      channelData[i] &= 0x00FFFFFF;
    }
  }
}

// REGISTER READ/WRITE COMMANDS
//static uint8_t rx_data_spi1[3];
uint8_t ADS1299_RREG(uint8_t _address) {   //  reads ONE register at _address
  uint8_t opcode = _address + 0x20;   //  RREG expects 001rrrrr where rrrrr = _address
  uint8_t err_code;
	txrx_size = 3;
	
	tx_data_spi[0] = opcode;  
	tx_data_spi[1] = 0x00;
	tx_data_spi[2] = 0x00;
	//nrf_gpio_pin_toggle(TEST_PIN);
	err_code = nrf_drv_spi_transfer(&spi, tx_data_spi, txrx_size, rx_data_spi, txrx_size);
	//nrf_gpio_pin_toggle(TEST_PIN);
	nrf_delay_us(50);
	regData[_address] = rx_data_spi[2];     //  update the mirror array

  if (verbosity){           //  verbosity output
    //printRegisterName(_address);
    //printHex(_address);
    //Serial.print(F(", "));
    //printHex(regData[_address]);
    //Serial.print(F(", "));
    ADS1299_printRegisterName(_address);
    printf("0x%x, 0x%x, ",_address, regData[_address]);
    for(int j = 0; j<8; j++){
      //Serial.print(bitRead(regData[_address], 7-j));
      printf("%d",bitRead(regData[_address], 7-j));
      if(j!=7)  printf(", ");//Serial.print(F(", "));
    }
    
    //Serial.println();
    printf("\r\n");
  }
  return regData[_address];     // return requested register value
}

// Read more than one register starting at _address
void ADS1299_RREGS(uint8_t _address, uint8_t _numRegistersMinusOne) {

  uint8_t err_code;
	uint8_t num_registers = _numRegistersMinusOne+1;
	uint8_t wreg_init_opcode = _address + 0x20;
	
	txrx_size = num_registers+2;
	delay_cnt = (txrx_size * 10) + 10;
	
	for (int i = 0; i < txrx_size; ++i) {
		tx_data_spi[i] = 0;
		rx_data_spi[i] = 0;
	}
	
	tx_data_spi[0] = wreg_init_opcode;
	tx_data_spi[1] = _numRegistersMinusOne;
	
	err_code = nrf_drv_spi_transfer(&spi, tx_data_spi, txrx_size, rx_data_spi, txrx_size);
	nrf_delay_us(delay_cnt);
	
	for (int j = 0; j < num_registers; ++j) {
		 regData[_address+j] = rx_data_spi[j+2];
	}

  if(verbosity){            //  verbosity output
    for(int i = 0; i<= _numRegistersMinusOne; i++){
      //printRegisterName(_address + i);
      //printHex(_address + i);
      //Serial.print(F(", "));
      //printHex(regData[_address + i]);
      //Serial.print(F(", "));
      ADS1299_printRegisterName(_address + i);
      printf("0x%x, 0x%x, ",_address + i, regData[_address + i]);
      
      for(int j = 0; j<8; j++){
        //Serial.print(bitRead(regData[_address + i], 7-j));
        printf("%d",bitRead(regData[_address + i], 7-j));
        if(j!=7) printf(", ");
      }
      printf("\r\n");
      //Serial.println();
      //delay(30);
    }
  }
}

void ADS1299_WREG(uint8_t _address, uint8_t _value) {  //  Write ONE register at _address
  
  uint8_t err_code;
	txrx_size = 3;
	 
	uint8_t opcode = _address + 0x40;   //  WREG expects 010rrrrr where rrrrr = _address
	
	tx_data_spi[0] = opcode;  
	tx_data_spi[1] = 0x00;
	tx_data_spi[2] = _value;

	err_code = nrf_drv_spi_transfer(&spi, tx_data_spi, txrx_size, rx_data_spi, txrx_size);
	regData[_address] = _value;     //  update the mirror array
	//printf(" Power-on reset and initialization procedure.. EC: %d \r\n",err_code);
	  if(verbosity){            //  verbosity output
    //Serial.print(F("Register "));
    //printHex(_address);
    //Serial.println(F(" modified."));
    printf("Registers 0x%x modified.\r\n",_address);
  }
}

void ADS1299_WREGS(uint8_t _address, uint8_t _numRegistersMinusOne) {
  
  uint8_t err_code;
	uint8_t num_registers = _numRegistersMinusOne+1;
	uint8_t wreg_init_opcode = _address + 0x40;
	txrx_size = num_registers+2;
	
	for (int i = 0; i < txrx_size; ++i) {
		tx_data_spi[i] = 0;
		rx_data_spi[i] = 0;
	}
	
	tx_data_spi[0] = wreg_init_opcode;
	tx_data_spi[1] = _numRegistersMinusOne;
	
	for (int j = 0; j < num_registers; ++j) {
		tx_data_spi[j+2] = regData[_address+j];
	}
	err_code = nrf_drv_spi_transfer(&spi, tx_data_spi, txrx_size, rx_data_spi, txrx_size);
	//printf(" Power-on reset and initialization procedure.. EC: %d \r\n",err_code);

  if(verbosity){
    //Serial.print(F("Registers "));
    //printHex(_address); Serial.print(F(" to "));
    //printHex(_address + _numRegistersMinusOne);
    //Serial.println(F(" modified"));
    printf("Registers 0x%x to 0x%x modified\r\n",_address, _address+_numRegistersMinusOne);
  }
}

void ADS1299_printDeviceID(void)
{
    bool wasRunning;
    bool prevverbosityState = verbosity;
    if (isRunning){ ADS1299_stopADS(); wasRunning = true;}
        verbosity = true;
        ADS1299_getDeviceID();
        verbosity = prevverbosityState;
    if (wasRunning){ ADS1299_startADS(); }    
}

uint8_t ADS1299_getDeviceID(void) {     // simple hello world com check
  uint8_t data = ADS1299_RREG(0x00);
  if(verbosity){            // verbosity otuput
    //Serial.print(F("Device ID "));
    //printHex(data); 
    //    Serial.println();
    printf("ADS Device ID : 0x%x\r\n",data);
  }
  return data;
}

//print out the state of all the control registers
void ADS1299_printAllRegisters(void)   
{
    bool wasRunning = false;
    bool prevverbosityState = verbosity;
    if (isRunning){ ADS1299_stopADS(); wasRunning = true; }
        verbosity = true;           // set up for verbosity output
        ADS1299_RREGS(0x00,0x0B);       // read out the first registers
//        delay(10);              // stall to let all that data get read by the PC
//        RREGS(0x11,0x17-0x11);  // read out the rest
        verbosity = prevverbosityState;
    if (wasRunning){ ADS1299_startADS(); }
}

// String-Byte converters for RREG and WREG
void ADS1299_printRegisterName(uint8_t _address) {
  /*  if(_address == ID){ // CHANGE THIS TO SWITCH/CASE
        printf("ID, ");
    }
    else if(_address == CONFIG1){
        printf("CONFIG1, ");
    }
    else if(_address == CONFIG2){
        printf("CONFIG2, ");
    }
    //else if(_address == CONFIG3){
    //    printf("CONFIG3, ");
    //}
    else if(_address == LOFF){
        printf("LOFF, ");
    }
    else if(_address == CH1SET){
        printf("CH1SET, ");
    }
    else if(_address == CH2SET){
        printf("CH2SET, ");
    }
	else if(_address == RLDSENS){
        printf("RLDSENS, ");
    }
    else if(_address == LOFFSENS){
        printf("LOFFSENS, ");
    }
    else if(_address == LOFFSTAT){
        printf("LOFFSTAT, ");
    }
    else if(_address == RESP1){
        printf("RESP1, ");
    }
	else if(_address == RESP2){
        printf("RESP2, ");
    }*/
}

// // // // // //
// End of File //
// // // // // // 
