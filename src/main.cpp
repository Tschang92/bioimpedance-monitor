/****************************************
 Code for 2-wire bioipedance measurement,
 written for Adafruit Feather M0 MCU in
 conjunction with Analog Devices AD5941

 Pin Connections AD5941 <--> MCU
 CS --> A5
 RESET --> A4
 Interrupt (GPIO0) --> A1

 based on example code AD5941_BIOZ
  
---------------------------------------------------
This code is based off Analog Devices' examples
 *******************************************/


#include <Arduino.h>
#include "ad5940.h"
#include <LibPrintf.h>
#include <SPI.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <BIOZ-2Wire.h>

uint32_t MCUPlatformInit(void *pCfg);

static uint32_t IntCount;
static uint32_t count;
uint32_t temp;

#define APPBUFF_SIZE 512
uint32_t AppBuff[APPBUFF_SIZE];

/* print results to UART */
int32_t BIOZShowResult(uint32_t *pData, uint32_t DataCount)
{
  float freq;
  fImpCar_Type *pImp2 = (fImpCar_Type *)pData;
  AppBIOZCtrl(BIOZCTRL_GETFREQ, &freq);    //Get Frequency and write it to the address of freq
  printf("Freq: %.2f, ", freq);

  /* Process data */
  for (int i = 0; i < DataCount; i++)
  {
    printf ("RzResistance: %f Ohm, RzReactance: %f Ohm\n", pImp2[i].Real, pImp2[i].Image);
  }
  return 0;
}

/* Initialize AD5940 basic blocks like clock */
static int32_t AD5940PlaformCfg(void)
{
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;

  /* Use hardware reset */
  AD5940_HWReset();
  /* Platform configuration */
  AD5940_Initialize();
  /* Step 1. Configure clock */
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);

  /* Step 2. Configure FIFO and Sequencer */
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB; // 4kB for FIFO, the rest 2kB for sequencer
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = 4;  // AppBIOZCfg.FifoThresh // DFT result. One pair for RCAL, another for Rz
  AD5940_FIFOCfg(&fifo_cfg);  // disable to reset fifo
  fifo_cfg.FIFOEn = bTRUE;  
  AD5940_FIFOCfg(&fifo_cfg);  // Enable FIFO here

  /* Step3. Interrupt Controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE); // Enable all interrup in Interrupt Controller 1, so we can check INTC flags
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE); // Interrupt Controller 0 will control GP0 to generate interrupt to MCU

  /* Step 4. Reconfigure GPIO */
  gpio_cfg.FuncSet = GP6_SYNC | GP5_SYNC | GP4_SYNC | GP2_TRIG | GP1_SYNC | GP0_INT;
  gpio_cfg.InputEnSet = AGPIO_Pin2;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin4|AGPIO_Pin5|AGPIO_Pin6;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;

  AD5940_AGPIOCfg(&gpio_cfg);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  // Allow AFE to enter sleep mode
  return 0;
}

/* !!Change Application parameters here if you want to change it to none default values */
void AD5940BIOZStructInit(void)
{
  AppBIOZCfg_Type *pBIOZCfg;
  AppBIOZGetCfg(&pBIOZCfg);

  pBIOZCfg->SeqStartAddr = 0;
  pBIOZCfg->MaxSeqLen = 512;

  pBIOZCfg->SinFreq = 100000.0; /* 100kHz, Value is ignored if SweepEn = bTRUE */
  pBIOZCfg->RcalVal = 10000.0;  /* Value of RCAL in Circuit */
  pBIOZCfg->HstiaRtiaSel = HSTIARTIA_200;

  /* Configure Switch matrix */
  pBIOZCfg->DswitchSel = SWD_CE0; // Measuring Lead 1
  pBIOZCfg->PswitchSel = SWP_CE0;
  pBIOZCfg->NswitchSel = SWN_AIN2; // Measuring Lead 2
  pBIOZCfg->TswitchSel = SWN_AIN2;

  /* Configure Sweep parameters */
  pBIOZCfg->SweepCfg.SweepEn = bFALSE; /* Measuring at single Frequency */
  pBIOZCfg->SweepCfg.SweepStart = 1000;
  pBIOZCfg->SweepCfg.SweepStop = 200000.0;
  pBIOZCfg->SweepCfg.SweepPoints = 100; /* Max is 100 */
  pBIOZCfg->SweepCfg.SweepLog = bFALSE;

  pBIOZCfg->BIOZODR = 5;  /* ODR (Sample Rate) 5 Hz */
  pBIOZCfg->NumOfData = -1; /* Never Stop, until you stop in manually by AppBIOZCtrl() function */
}



void setup() {

  MCUPlatformInit(0);
  AD5940_MCUResourceInit(0);
  AD5940PlaformCfg();
  AD5940BIOZStructInit();
  AppBIOZInit(AppBuff, APPBUFF_SIZE); /* Initialize BIOZ application. Provide buffer, which is used to store sequencer commands */
  AppBIOZInit(BIOZCTRL_START, 0); /* Control BIOZ measurement to start, second parameter has no meaning here */

}

void loop() {
  /* Check if interrupt flag which will be set when interrupt occured */
  if(AD5940_GetMCUIntFlag())
  {
    AD5940_ClrMCUIntFlag(); /* Clear this flag */
    temp = APPBUFF_SIZE;
    AppBIOZISR(AppBuff, &temp); /* Deal with it and provide a buffer to store data we got */
    BIOZShowResult(AppBuff, temp); /* Show the result to UART */
  }
}