/*
MODIFIED VERSION FOR ADAFRUIT FEATHER M0


Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*/

#include <Arduino.h>
#include "stdio.h"
#include "AD5940.h"

/* Functions that used to initialize MCU platform */
uint32_t MCUPlatformInit(void *pCfg);

/* 
int main(void)
{
  void AD5940_Main(void);
  MCUPlatformInit(0);
  AD5940_MCUResourceInit(0);
  printf("Hello AD5940-Build Time:%s\n",__TIME__);
  AD5940_Main();
} */

/* Below functions are used to initialize MCU Platform */
uint32_t MCUPlatformInit(void *pCfg)
{
  int UrtCfg(int iBaud);

  UrtCfg(230400);/*Baud rate: 230400*/
  return 1;
}

/**
	@brief int UrtCfg(int iBaud, int iBits, int iFormat)
			==========Configure the UART.
	@param iBaud :{B1200,B2200,B2400,B4800,B9600,B19200,B38400,B57600,B115200,B230400,B430800}	\n
		Set iBaud to the baudrate required:
		Values usually: 1200, 2200 (for HART), 2400, 4800, 9600,
		        19200, 38400, 57600, 115200, 230400, 430800, or type in baud-rate directly
	@note
		- Powers up UART if not powered up.
		- Standard baudrates are accurate to better than 0.1% plus clock error.\n
		- Non standard baudrates are accurate to better than 1% plus clock error.
   @warning - If an external clock is used for the system the ullRtClk must be modified with \n
         the speed of the clock used.
**/

int UrtCfg(int iBaud)
{
  Serial.begin(iBaud);
  return 0;
}
