/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to set GPIO pin mode and use pin data input/output control.
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

    /* Enable GPIO Port B clock */
    CLK_EnableModuleClock(GPB_MODULE);

    /* Enable GPIO Port C clock */
    CLK_EnableModuleClock(GPC_MODULE);

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

int32_t main(void)
{
    int32_t i32Err, i32TimeOutCnt;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART1 for printf */
    UART1_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+-------------------------------------------------+\n");
    printf("|    PB.3 (Output) and PC.1 (Input) Sample Code   |\n");
    printf("+-------------------------------------------------+\n\n");

    /*--------------------------------------------------------------------------*/
    /* GPIO Basic Mode Test --- Use Pin Data Input/Output to control GPIO pin   */
    /*--------------------------------------------------------------------------*/
    printf("  >> Please connect PB.3 and PC.1 first << \n");
    printf("     Press any key to start test by using [Pin Data Input/Output Control] \n\n");
    getchar();

    /* Configure PB.3 as Output mode and PC.1 as Input mode then close it */
    GPIO_SetMode(PB, BIT3, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PC, BIT1, GPIO_MODE_INPUT);

    i32Err = 0;
    printf("GPIO PB.3 (output mode) connect to PC.1 (input mode) ......");

    /* Use Pin Data Input/Output Control to pull specified I/O or get I/O pin status */
    /* Set PB.3 output pin value is low */
    PB3 = 0;

    /* Set time out counter */
    i32TimeOutCnt = 100;

    /* Wait for PC.1 input pin status is low for a while */
    while(PC1 != 0)
    {
        if(i32TimeOutCnt > 0)
        {
            i32TimeOutCnt--;
        }
        else
        {
            i32Err = 1;
            break;
        }
    }

    /* Set PB.3 output pin value is high */
    PB3 = 1;

    /* Set time out counter */
    i32TimeOutCnt = 100;

    /* Wait for PC.1 input pin status is high for a while */
    while(PC1 != 1)
    {
        if(i32TimeOutCnt > 0)
        {
            i32TimeOutCnt--;
        }
        else
        {
            i32Err = 1;
            break;
        }
    }

    /* Print test result */
    if(i32Err)
    {
        printf("  [FAIL].\n");
    }
    else
    {
        printf("  [OK].\n");
    }

    /* Configure PB.3 and PC.1 to default Quasi-bidirectional mode */
    GPIO_SetMode(PB, BIT3, GPIO_MODE_QUASI);
    GPIO_SetMode(PC, BIT1, GPIO_MODE_QUASI);

    while(1);
}
