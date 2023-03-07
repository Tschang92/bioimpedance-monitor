/*********************************************************************
Code for 4-wire Impedance Analysis
written for Adafruit Feather M0 Basic MCU

Pin Connections:
CS --> A5
RESET --> A4
Interrupt (GPIO0) --> A1

based on ADI example code: AD5940_BIOZ
-----------------------------------------------------------------------------
This software is based on Analog Devices' example code.
*********************************************************************/

#include <Arduino.h>
#include "ad5940.h"
#include <LibPrintf.h>
#include <SPI.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "BIOZ-2Wire.h""

uint32_t MCUPlatformInit(void *pCfg);

static uint32_t IntCount;
static uint32_t count;
uint32_t temp;

#define APPBUFF_SIZE 512
uint32_t AppBuff[APPBUFF_SIZE];

#define LED_GREEN 12
#define LED_YELLOW 11
#define LED_RED LED_BUILTIN


/* Measurement active status */
void isActive(void)
{
  AppBIOZCfg_Type *pBIOZCfg;
  if (AppBIOZGetCfg(&pBIOZCfg->BIOZInited) == bTRUE);
    digitalWrite(LED_YELLOW, HIGH);
}

/* print results to UART */
int32_t BIOZShowResult(uint32_t *pData, uint32_t DataCount)
{
  float freq;

  //fImpPol_Type *pImp = (fImpPol_Type *)pData;
  fImpCar_Type *pImp2 = (fImpCar_Type *)pData;
  AppBIOZCtrl(BIOZCTRL_GETFREQ, &freq);

  printf("Freq: %.2f, ", freq);
  /*Process data*/
  for (int i = 0; i < DataCount; i++)
  {
    //printf("RzMag: %f Ohm, RzPhase: %f \n", pImp[i].Magnitude, pImp[i].Phase * 180 / MATH_PI); //Phase in degrees
    //printf("Freq: %.2f, ", freq);
    printf("RzResistance: %f Ohm, RzReactance: %f Ohm\n", pImp2[i].Real, pImp2[i].Image);
  }
  return 0;
}

/* Tissue Classification */

void isEpidural(uint32_t *pData, uint32_t DataCount)
{
  fImpCar_Type *pImp2 = (fImpCar_Type *)pData;

  // feed data into kNN Model

  // Test
  for (int i = 0; i < DataCount; i++)
  {
    if (pImp2[i].Real >= 40000.0)
      digitalWrite(LED_GREEN, HIGH);
    else
      digitalWrite(LED_GREEN, LOW);
  }
}

void isCSF(uint32_t *pData, uint32_t DataCount)
{
  fImpCar_Type *pImp2 = (fImpCar_Type *)pData;

  // feed data into kNN Model

  // Test
  for (int i = 0; i < DataCount; i++)
  {
    if (pImp2[i].Real < 40000.0)
    {
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, HIGH);
    }
    else
      digitalWrite(LED_RED, LOW);
  }
}


/* Initialize AD5940 basic blocks like clock */
static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;

  /* Use hardware reset */
  AD5940_HWReset();
  /* Platform configuration */
  AD5940_Initialize();
  /* Step1. Configure clock */
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB; /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = 4;   // AppBIOZCfg.FifoThresh;        /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
  AD5940_FIFOCfg(&fifo_cfg); /* Disable to reset FIFO. */
  fifo_cfg.FIFOEn = bTRUE;
  AD5940_FIFOCfg(&fifo_cfg); /* Enable FIFO here */

  /* Step3. Interrupt controller */

  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);         /* Enable all interrupt in Interrupt Controller 1, so we can check INTC flags */
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE); /* Interrupt Controller 0 will control GP0 to generate interrupt to MCU */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Reconfigure GPIO */
  gpio_cfg.FuncSet = GP6_SYNC | GP5_SYNC | GP4_SYNC | GP2_TRIG | GP1_SYNC | GP0_INT;
  gpio_cfg.InputEnSet = AGPIO_Pin2;
  gpio_cfg.OutputEnSet = AGPIO_Pin0 | AGPIO_Pin1 | AGPIO_Pin4 | AGPIO_Pin5 | AGPIO_Pin6;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;

  AD5940_AGPIOCfg(&gpio_cfg);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK); /* Allow AFE to enter sleep mode. */
  return 0;
}

/* !!Change the application parameters here if you want to change it to none-default value */
void AD5940BIOZStructInit(void)
{
  AppBIOZCfg_Type *pBIOZCfg;

  AppBIOZGetCfg(&pBIOZCfg);

  pBIOZCfg->SeqStartAddr = 0;
  pBIOZCfg->MaxSeqLen = 512; /** @todo add checker in function */

  
  pBIOZCfg->DftNum = DFTNUM_8192;
  pBIOZCfg->NumOfData = -1; /* Never stop until you stop it manually by AppBIOZCtrl() function */
  pBIOZCfg->BIOZODR = 10;    /* ODR(Sample Rate) 20Hz */
  pBIOZCfg->FifoThresh = 4; /* 4 */
  pBIOZCfg->ADCSinc3Osr = ADCSINC3OSR_2;

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
  pBIOZCfg->PwrMod = AFEPWR_HP;

  /* Configure Measurement setup */
  pBIOZCfg->SinFreq = 100000.0;
  pBIOZCfg->RcalVal = 10000.0;
}

/****************** SETUP AND LOOP ***********************/

void setup()
{

  // Configure LED pins
  pinMode(LED_GREEN, OUTPUT); 
  pinMode(LED_YELLOW, OUTPUT);

  MCUPlatformInit(0);
  AD5940_MCUResourceInit(0);
  // AD5940_HWReset();
  // AD5940_Initialize();
  AD5940PlatformCfg();
  AD5940BIOZStructInit();             /* Configure your parameters in this function */
  AppBIOZInit(AppBuff, APPBUFF_SIZE); /* Initialize BIOZ application. Provide a buffer, which is used to store sequencer commands */
  AppBIOZCtrl(BIOZCTRL_START, 0);      /* Control BIOZ measurement to start. Second parameter has no meaning with this command. */

  // Indicate running system
  isActive();

}

void loop()
{
  
  /* Check if interrupt flag which will be set when interrupt occurred. */
  if (AD5940_GetMCUIntFlag())
  {
    IntCount++;
    AD5940_ClrMCUIntFlag(); /* Clear this flag */
    temp = APPBUFF_SIZE;
    AppBIOZISR(AppBuff, &temp);    /* Deal with it and provide a buffer to store data we got */
    BIOZShowResult(AppBuff, temp); /* Show the results to UART */

    // Check for Epidural Tissue at Needle Tip
    isEpidural(AppBuff, temp);

    // CHeck for CSF at Needle Tip
    isCSF(AppBuff, temp);

    if (IntCount == 240)
    {
      IntCount = 0;
      // AppBIOZCtrl(BIOZCTRL_SHUTDOWN, 0);
    }
  }
} 