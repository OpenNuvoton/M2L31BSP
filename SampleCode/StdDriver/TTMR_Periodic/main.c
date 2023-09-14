/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use the TTMR periodic mode to generate timer interrupt every 1 second.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


void TTMR0_IRQHandler(void)
{
    static uint32_t sec = 1;
    printf("%d sec\n", sec++);

    /* clear TTMR interrupt flag */
    TTMR_ClearIntFlag(TTMR0);
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HIRC clock (Internal RC 48 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL4_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART1_MODULE);

    /* Select TTMR0 clock source */
    CLK_SetModuleClock(TTMR0_MODULE, LPSCC_CLKSEL0_TTMR0SEL_HIRC, 0);

    /* Enable TTMR0 clock */
    CLK_EnableModuleClock(TTMR0_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART */
    Uart1DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

/*----------------------------------------------------------------------*/
/* Init UART1                                                           */
/*----------------------------------------------------------------------*/
void UART1_Init(void)
{
    /* Reset UART1 */
    SYS_ResetModule(UART1_RST);

    /* Configure UART1 and set UART1 baud rate */
    UART_Open(UART1, 115200);
}

int main(void)
{
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART1 for printf */
    UART1_Init();

    printf("\nThis sample code use TTMR to generate interrupt every 1 second \n");

    /* Set TTMR frequency to 1HZ */
    TTMR_Open(TTMR0, TTMR_PERIODIC_MODE, 1);

    /* Enable TTMR interrupt */
    TTMR_EnableInt(TTMR0);
    NVIC_EnableIRQ(TTMR0_IRQn);

    /* Start TTMR0 */
    TTMR_Start(TTMR0);

    while(1);
}
