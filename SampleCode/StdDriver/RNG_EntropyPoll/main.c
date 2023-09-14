/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Generate random numbers using TRNG.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART1_MODULE);

    /* Enable TRNG module clock */
    CLK_EnableModuleClock(TRNG_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL4_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set GPA multi-function pins for UART1 RXD(PA.8) and TXD(PA.9) */
    Uart1DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    int32_t i, n;
    uint8_t au8Buf[32];

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART1, 115200);

    printf("CPU @ %dHz\n", SystemCoreClock);
    printf("Hardware Entropy Pool Demo:\n");

    /* Initial Random Number Generator */
    RNG_Open();

    do
    {
        /* Get entropy */
        n = RNG_EntropyPoll(au8Buf, 32);

        if(n < 0)
        {
            printf("Entropy poll fail. return code = %d\n", n);
            break;
        }

        for(i = 0; i < 32; i++)
        {
            printf("%02x", au8Buf[i]);
        }
        printf("\n");

        CLK_SysTickDelay(100000);
    }
    while(1);

}
