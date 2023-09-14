/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Change duty cycle and period of output waveform by EPWM Double Buffer function.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/



void EPWM0_P0_IRQHandler(void);
void SYS_Init(void);
void UART1_Init(void);

/**
 * @brief       EPWM0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle EPWM0 interrupt event
 */
void EPWM0P0_IRQHandler(void)
{
    static int32_t i32Toggle = 0;

    /* Update EPWM0 channel 0 period and duty */
    if(i32Toggle == 0)
    {
        EPWM_SET_CNR(EPWM0, 0, 99);
        EPWM_SET_CMR(EPWM0, 0, 40);
    }
    else
    {
        EPWM_SET_CNR(EPWM0, 0, 224);
        EPWM_SET_CMR(EPWM0, 0, 112);
    }
    i32Toggle ^= 1;
    /* Clear channel 0 period interrupt flag */
    EPWM_ClearPeriodIntFlag(EPWM0, 0);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART1_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL4_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART1_MODULE);

    /* Enable EPWM0 and EPWM1 module clock */
    CLK_EnableModuleClock(EPWM0_MODULE);
    CLK_EnableModuleClock(EPWM1_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* EPWM clock frequency configuration                                                                      */
    /*---------------------------------------------------------------------------------------------------------*/
    CLK_SetModuleClock(EPWM0_MODULE, CLK_CLKSEL2_EPWM0SEL_PCLK0, 0);
    CLK_SetModuleClock(EPWM1_MODULE, CLK_CLKSEL2_EPWM1SEL_PCLK1, 0);

    /* Reset EPWM0 and EPWM1 module */
    SYS_ResetModule(EPWM0_RST);
    SYS_ResetModule(EPWM1_RST);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART1 RXD=PA.8 and TXD=PA.9 */
    Uart1DefaultMPF();

    /* Set PE multi-function pins for EPWM0 Channel0~5 */
    SYS->GPE_MFP1 = (SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE7MFP_Msk)) | SYS_GPE_MFP1_PE7MFP_EPWM0_CH0;
    SYS->GPE_MFP1 = (SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE6MFP_Msk)) | SYS_GPE_MFP1_PE6MFP_EPWM0_CH1;
    SYS->GPE_MFP1 = (SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE5MFP_Msk)) | SYS_GPE_MFP1_PE5MFP_EPWM0_CH2;
    SYS->GPE_MFP1 = (SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE4MFP_Msk)) | SYS_GPE_MFP1_PE4MFP_EPWM0_CH3;
}

void UART1_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure UART1 and set UART1 baud rate */
    UART_Open(UART1, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART1_Init();

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          EPWM Driver Sample Code                       |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use EPWM0 channel 0 to output waveform\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: EPWM0 channel 0 (PE.7)\n");
    printf("\nUse double buffer feature.\n");

    /*
        EPWM0 channel 0 waveform of this sample shown below (up counter type):

        |<-        CNR + 1  clk     ->|  CNR + 1 = 224 + 1 CLKs
        |<-  CMR clk ->|                 CMR = 112 CLKs
                                      |<-   CNR + 1  ->|  CNR + 1 = 99 + 1 CLKs
                                      |<-CMR->|           CMR = 40 CLKs

         ______________                _______          ____
        |      112     |_____113______|   40  |____60__|     EPWM waveform

    */

    /*
      Configure EPWM0 channel 0 init period and duty(up counter type).
      Period is PLL / 2 / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Frequency = 36 MHz / (1 * (224 + 1)) = 160000 Hz
      Duty ratio = (112) / (224 + 1) = 49.8%
    */
    /* EPWM0 channel 0 frequency is 160000Hz, duty 50%, */
    EPWM_ConfigOutputChannel(EPWM0, 0, 160000, 50);

    /* Enable output of EPWM0 channel 0 */
    EPWM_EnableOutput(EPWM0, EPWM_CH_0_MASK);

    /* Enable EPWM0 channel 0 period interrupt, use channel 0 to measure time. */
    EPWM_EnablePeriodInt(EPWM0, 0, 0);
    NVIC_EnableIRQ(EPWM0_P0_IRQn);

    /* Start */
    EPWM_Start(EPWM0, EPWM_CH_0_MASK);

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
