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
#include "impedance.h"

uint32_t MCUPlatformInit(void *pCfg);
uint32_t temp;

#define APPBUFF_SIZE 512
uint32_t AppBuff[APPBUFF_SIZE];

#define BUZZER_PIN 13
#define LED_GREEN 12
#define LED_YELLOW 11
#define LED_RED 10
#define BUTTON_PIN 9



/****************************** Initialize AD5940 basic blocks like clock ************************************/

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
  fifo_cfg.FIFOThresh = 4;   // AppImpedanceCfg.FifoThresh;        /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
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
void AD5940ImpedanceStructInit(void)
{
  AppIMPCfg_Type *pImpedanceCfg;

  AppIMPGetCfg(&pImpedanceCfg);

  pImpedanceCfg->SeqStartAddr = 0;
  pImpedanceCfg->MaxSeqLen = 512; /** @todo add checker in function */

  
  pImpedanceCfg->DftNum = DFTNUM_8192;
  pImpedanceCfg->NumOfData = 100; /* Never stop until you stop it manually by AppImpedanceCtrl() function */
  pImpedanceCfg->ImpODR = 2;    /* ODR(Sample Rate) 20Hz */
  pImpedanceCfg->FifoThresh = 4; /* 4 */
  pImpedanceCfg->ADCSinc3Osr = ADCSINC3OSR_4;

   /* Configure Switch matrix */
  pImpedanceCfg->DswitchSel = SWD_CE0; // Measuring Lead 1
  pImpedanceCfg->PswitchSel = SWP_CE0;
  pImpedanceCfg->NswitchSel = SWN_AIN2; // Measuring Lead 2
  pImpedanceCfg->TswitchSel = SWN_AIN2;

  /* Configure Sweep parameters */
  // pImpedanceCfg->SweepCfg.SweepEn = bTRUE; /* Measuring at single Frequency */
  // pImpedanceCfg->SweepCfg.SweepStart = 1000;
  // pImpedanceCfg->SweepCfg.SweepStop = 100000.0;
  // pImpedanceCfg->SweepCfg.SweepPoints = 100; /* Max is 100 */
  // pImpedanceCfg->SweepCfg.SweepLog = bFALSE;
  pImpedanceCfg->PwrMod = AFEPWR_HP;

  pImpedanceCfg->SweepCfg.SweepEn = bFALSE,
  pImpedanceCfg->SweepCfg.SweepStart = 1000,
  pImpedanceCfg->SweepCfg.SweepStop = 100000.0,
  pImpedanceCfg->SweepCfg.SweepPoints = 101,
  pImpedanceCfg->SweepCfg.SweepLog = bFALSE,
  pImpedanceCfg->SweepCfg.SweepIndex = 0,

  /* Configure Measurement setup */
  pImpedanceCfg->SinFreq = 10000.0;
  pImpedanceCfg->RcalVal = 1000.0;
  pImpedanceCfg->HstiaRtiaSel = HSTIARTIA_1K;
  pImpedanceCfg->DacVoltPP = 600.0; //600.0,
}

/****************************** print Measured Impedance to UART **********************/

int32_t ImpedanceShowResult(uint32_t *pData, uint32_t DataCount)
{
  float freq;

  fImpPol_Type *pImp = (fImpPol_Type*)pData;
  AppIMPCtrl(IMPCTRL_GETFREQ, &freq);

  printf("Freq:%.2f ", freq);
  /*Process data*/
  for(int i=0;i<DataCount;i++)
  {
    printf("RzMag: %f Ohm , RzPhase: %f \n",pImp[i].Magnitude,pImp[i].Phase*180/MATH_PI);
  }
  return 0;
}


/****************************** Tissue Classification *********************************/

void isEpidural(uint32_t *pData, uint32_t DataCount)
{
  fImpCar_Type *pImp = (fImpCar_Type *)pData;

  // feed data into kNN Model

  // Test
  for (int i = 0; i < DataCount; i++)
  {
    if (pImp[i].Real >= 9000.0)
      digitalWrite(LED_GREEN, HIGH);
    else
      digitalWrite(LED_GREEN, LOW);
  }
}


void isCSF(uint32_t *pData, uint32_t DataCount)
{
  fImpCar_Type *pImp = (fImpCar_Type *)pData;

  // feed data into kNN Model

  // Test
  for (int i = 0; i < DataCount; i++)
  {
    if (pImp[i].Real < 1000.0)
    {
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, HIGH);
      digitalWrite(BUZZER_PIN, HIGH);
    }
    else
      digitalWrite(LED_RED, LOW);
      digitalWrite(BUZZER_PIN, HIGH);
  }
}

/******************* BUTTON FUNCTIONS *********************/

int buttonState;  // Holds state of button, HIGH triggers switch of state in State Machine

void Button()
{
  #define buttonPressed LOW
  uint32_t currentMillis = millis();                          // current timestamp in ms
  static uint32_t lastMillis;                                 // last timestamp in ms
  const uint32_t DEBOUNCEDELAY = 200;                         // Debounce delay for button
  bool currentButtonState = digitalRead(BUTTON_PIN);          // Current state of button
  static bool lastButtonState = HIGH;                         // last remembered button state

  if (lastButtonState != currentButtonState)
  {
    if (currentMillis - lastMillis >= DEBOUNCEDELAY)
    {
      lastButtonState = currentButtonState;
      if (currentButtonState == buttonPressed)
        buttonState = HIGH;                                   // Update button state
    }
    else {
      lastMillis = currentMillis;
      buttonState = LOW;                                      // Make sure button state is defined (dont know if this helps anything)
    }
  }
}


/****************** STATE MACHINE ************************/

const long blinkInterval = 500;                               // Blink interval for yellow LED in ms
unsigned long previousMillis = 0;                             // Last remembered timestamp for Yellow LED blink in ms
int YellowLEDstate = LOW;                                     // State of Yellow LED

// States of State Machine
enum states {
  NONE,           // Default State before initialization
  STANDBY,        // Standby State, no active data acquisition
  ACTIVE          // Active State, data acquisition running
};

states prev_state, state;                                     //Global variables to store the previous and current states

void standby()
{
  if (state != prev_state)                                    // If entering this state, do initialization stuff
  {
    prev_state = state;
    AppIMPCtrl(IMPCTRL_STOPNOW, 0);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, LOW);
    digitalWrite(BUZZER_PIN, LOW);
  }

  // Perform state tasks

  /* Blink Standby LED */
   unsigned long currentLEDMillis = millis();
  
    if (currentLEDMillis - previousMillis >= blinkInterval)
    {
      previousMillis = currentLEDMillis;

      if (YellowLEDstate == LOW) 
      {
        YellowLEDstate = HIGH;
      }
      else
      {
        YellowLEDstate = LOW;
      }
      digitalWrite(LED_YELLOW, YellowLEDstate);
    } 

  // Check for state transitions
    if(buttonState == HIGH)   
    {
      state = ACTIVE;
    }

    if(state != prev_state)               // Clean up
    {
      YellowLEDstate = LOW;
      digitalWrite(LED_YELLOW, YellowLEDstate);
      buttonState = LOW;
    }
}

void active() 
{
  if (state != prev_state)
  {
    prev_state = state;
    digitalWrite(LED_YELLOW, HIGH);
    AppIMPCtrl(IMPCTRL_START, 0);
  }

  // Perform state tasks

  // Handle Measurment data
  if (AD5940_GetMCUIntFlag())
  {
    AD5940_ClrMCUIntFlag(); /* Clear this flag */
    temp = APPBUFF_SIZE;
    AppIMPISR(AppBuff, &temp);    /* Deal with it and provide a buffer to store data we got */
    ImpedanceShowResult(AppBuff, temp); /* Show the results to UART */

    // Check for Epidural Tissue at Needle Tip
    isEpidural(AppBuff, temp);

    // CHeck for CSF at Needle Tip
    isCSF(AppBuff, temp);
  }

  // Check for state transitions
  if (buttonState == HIGH)
  {
    state = STANDBY;
  }

  if (state != prev_state)
  {
    buttonState = LOW;  // Cleanup
  }
}

/****************** SETUP AND LOOP ***********************/

void setup()
{

  // Configure LED pins
  pinMode(LED_GREEN, OUTPUT); 
  pinMode(LED_YELLOW, OUTPUT);

  // Configure Button as Input
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  MCUPlatformInit(0);
  AD5940_MCUResourceInit(0);
  // AD5940_HWReset();
  // AD5940_Initialize();
  AD5940PlatformCfg();
  AD5940ImpedanceStructInit();             /* Configure your parameters in this function */
  AppIMPInit(AppBuff, APPBUFF_SIZE); /* Initialize BIOZ application. Provide a buffer, which is used to store sequencer commands */
  
  prev_state = NONE;
  state = STANDBY;
  buttonState = LOW;
}

void loop()
{
  Button();
  switch (state)
  {
    case STANDBY:
      standby();
      break;
    case ACTIVE:
      active();
      break;
  } 
} 