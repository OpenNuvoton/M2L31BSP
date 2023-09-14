/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 23/02/24 4:40p $
 * @brief    Demonstrate how to use RMC CRC32 ISP command to calculate the CRC32 checksum of APROM and LDROM.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
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

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins */
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
    uint32_t    u32Data, u32ChkSum;    /* temporary data */

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART1 for printf */
    UART1_Init();

    printf("+-----------------------------------+\n");
    printf("|       RMC CRC32 Sample Demo       |\n");
    printf("+-----------------------------------+\n");

    SYS_UnlockReg();                   /* Unlock protected registers */

    RMC_Open();                        /* Enable RMC ISP function */

    u32Data = RMC_ReadCID();           /* Read company ID. Should be 0xDA. */
    if (g_RMC_i32ErrCode != 0)
    {
        printf("RMC_ReadCID failed!\n");
        goto lexit;
    }

#ifdef __ICCARM__
    printf("  Company ID ............................ [0x%x]\n", u32Data);
#else
    printf("  Company ID ............................ [0x%08x]\n", u32Data);
#endif
    u32Data = RMC_ReadPID();           /* Read product ID. */
    if (g_RMC_i32ErrCode != 0)
    {
        printf("RMC_ReadPID failed!\n");
        goto lexit;
    }
#ifdef __ICCARM__
    printf("  Product ID ............................ [0x%x]\n", u32Data);
#else
    printf("  Product ID ............................ [0x%08x]\n", u32Data);
#endif
    /* Read User Configuration CONFIG0 */
#ifdef __ICCARM__
    printf("  User Config 0 ......................... [0x%x]\n", RMC_Read(RMC_CONFIG_BASE));
#else
    printf("  User Config 0 ......................... [0x%08x]\n", RMC_Read(RMC_CONFIG_BASE));
#endif
    if (g_RMC_i32ErrCode != 0)
    {
        printf("RMC_Read(RMC_CONFIG_BASE) failed!\n");
        goto lexit;
    }
    /* Read User Configuration CONFIG3 */
#ifdef __ICCARM__
    printf("  User Config 3 ......................... [0x%x]\n", RMC_Read(RMC_CONFIG_BASE + 0xC));
#else
    printf("  User Config 3 ......................... [0x%08x]\n", RMC_Read(RMC_CONFIG_BASE + 0xC));
#endif
    if (g_RMC_i32ErrCode != 0)
    {
        printf("RMC_Read(RMC_CONFIG_BASE + 0xC) failed!\n");
        goto lexit;
    }
    RMC_GetChkSum(RMC_LDROM_BASE, 0x5);
    printf("\nLDROM (0xF100000 ~ 0xF102000) CRC32 checksum =>  ");

    /*
     *  Request RMC hardware to run CRC32 caculation on LDROM.
     */
    u32ChkSum = RMC_GetChkSum(RMC_LDROM_BASE, RMC_LDROM_SIZE);
    if(u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating LDROM CRC32 checksum!\n");
        goto lexit;                    /* failed */
    }
    printf("0x%x\n", u32ChkSum);       /* print out LDROM CRC32 check sum value */

    printf("\nAPROM bank0 (0x00000 ~ 0x40000) CRC32 checksum =>  ");

    /*
     *  Request RMC hardware to run CRC32 caculation on APROM bank 0.
     *  Note that RMC CRC32 checksum calculation area must not cross bank boundary.
     */
    u32ChkSum = RMC_GetChkSum(RMC_APROM_BASE, 0x40000);
    if(u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating APROM bank0 CRC32 checksum!\n");
        goto lexit;
    }
    printf("0x%x\n", u32ChkSum);       /* print out APROM CRC32 check sum value */

    /*
     *  Request RMC hardware to run CRC32 calculation on APROM bank 1.
     */
    printf("\nAPROM bank1 (0x40000 ~ 0x80000) CRC32 checksum =>  ");

    u32ChkSum = RMC_GetChkSum(RMC_APROM_BASE + 0x40000, 0x40000);

    if(u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating APROM bank1 CRC32 checksum!\n");
        goto lexit;
    }
    printf("0x%x\n", u32ChkSum);       /* print out APROM CRC32 check sum value */

    printf("\nRMC CRC32 checksum test done.\n");

lexit:
    RMC_Close();                       /* Disable RMC ISP function */
    SYS_LockReg();                     /* Lock protected registers */

    while(1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/


