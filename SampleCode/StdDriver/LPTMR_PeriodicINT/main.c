/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Implement LPTMR counting in periodic mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*----------------------------------------------------------------------*/
/* Global Interface Variables Declarations                              */
/*----------------------------------------------------------------------*/
volatile uint32_t g_au32LPTMRINTCount[2] = {0};


void LPTMR0_IRQHandler(void)
{
    if(LPTMR_GetIntFlag(LPTMR0) == 1)
    {
        /* Clear LPTMR0 time-out interrupt flag */
        LPTMR_ClearIntFlag(LPTMR0);

        g_au32LPTMRINTCount[0]++;
    }
}


void LPTMR1_IRQHandler(void)
{
    if(LPTMR_GetIntFlag(LPTMR1) == 1)
    {
        /* Clear LPTMR1 time-out interrupt flag */
        LPTMR_ClearIntFlag(LPTMR1);

        g_au32LPTMRINTCount[1]++;
    }
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
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select LPTMR clock source */
    CLK_SetModuleClock(LPTMR0_MODULE, LPSCC_CLKSEL0_LPTMR0SEL_HIRC, 0);
    CLK_SetModuleClock(LPTMR1_MODULE, LPSCC_CLKSEL0_LPTMR1SEL_HIRC, 0);

    /* Enable LPTMR clock */
    CLK_EnableModuleClock(LPTMR0_MODULE);
    CLK_EnableModuleClock(LPTMR1_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

/*----------------------------------------------------------------------*/
/* Init UART0                                                           */
/*----------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

int main(void)
{
    uint32_t u32InitCount, au32Counts[4];

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+--------------------------------------------+\n");
    printf("|    Timer Periodic Interrupt Sample Code    |\n");
    printf("+--------------------------------------------+\n\n");

    printf("# LPTMR0 Settings:\n");
    printf("    - Clock source is HIRC       \n");
    printf("    - Time-out frequency is 1 Hz\n");
    printf("    - Periodic mode             \n");
    printf("    - Interrupt enable          \n");
    printf("# LPTMR1 Settings:\n");
    printf("    - Clock source is HIRC      \n");
    printf("    - Time-out frequency is 2 Hz\n");
    printf("    - Periodic mode             \n");
    printf("    - Interrupt enable          \n");
    printf("# Check LPTMR0 ~ LPTMR1 interrupt counts are reasonable or not.\n\n");

    /* Open LPTMR0 in periodic mode, enable interrupt and 1 interrupt tick per second */
    LPTMR_Open(LPTMR0, LPTMR_PERIODIC_MODE, 1);
    LPTMR_EnableInt(LPTMR0);

    /* Open LPTMR1 in periodic mode, enable interrupt and 2 interrupt ticks per second */
    LPTMR_Open(LPTMR1, LPTMR_PERIODIC_MODE, 2);
    LPTMR_EnableInt(LPTMR1);

    /* Enable LPTMR0 ~ LPTMR1 NVIC */
    NVIC_EnableIRQ(LPTMR0_IRQn);
    NVIC_EnableIRQ(LPTMR1_IRQn);

    /* Clear LPTMR0 ~ LPTMR1 interrupt counts to 0 */
    g_au32LPTMRINTCount[0] = g_au32LPTMRINTCount[1] = 0;
    u32InitCount = g_au32LPTMRINTCount[0];

    /* Start LPTMR0 ~ LPTMR1 counting */
    LPTMR_Start(LPTMR0);
    LPTMR_Start(LPTMR1);

    /* Check LPTMR0 ~ LPTMR1 interrupt counts */
    printf("# LPTMR interrupt counts :\n");
    while(u32InitCount < 20)
    {
        if(g_au32LPTMRINTCount[0] != u32InitCount)
        {
            au32Counts[0] = g_au32LPTMRINTCount[0];
            au32Counts[1] = g_au32LPTMRINTCount[1];
            printf("    LPTMR0:%3d    LPTMR1:%3d\n",
                   au32Counts[0], au32Counts[1]);
            u32InitCount = g_au32LPTMRINTCount[0];

            if((au32Counts[1] > (au32Counts[0] * 2 + 1)) || (au32Counts[1] < (au32Counts[0] * 2 - 1)))
            {
                printf("*** FAIL ***\n");
                while(1);
            }
        }
    }

    printf("*** PASS ***\n");

    while(1);
}