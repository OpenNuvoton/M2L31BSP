/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Show how to set LPI2C use Multi bytes API Read and Write data to Slave.
 *           Needs to work with LPI2C_Slave sample code.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8MstTxData[3];
volatile uint8_t g_u8MstRxData;
volatile uint8_t g_u8MstDataLen;
volatile uint8_t g_u8MstEndFlag = 0;

typedef void (*LPI2C_FUNC)(uint32_t u32Status);
volatile static LPI2C_FUNC s_LPI2C0HandlerFn = NULL;


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

    /* Set HCLK1 to HIRC */
    CLK_SetModuleClock(HCLK1_MODULE, CLK_CLKSEL0_HCLK1SEL_HIRC, LPSCC_CLKDIV0_HCLK1(1));

    /* Enable HCLK1 clock */
    CLK_EnableModuleClock(HCLK1_MODULE);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable LPI2C0 clock */
    CLK_EnableModuleClock(LPI2C0_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPA_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    Uart0DefaultMPF();

    /* Set LPI2C0 multi-function pins */
    SYS->GPA_MFP1 = (SYS->GPA_MFP1 & ~(SYS_GPA_MFP1_PA4MFP_Msk | SYS_GPA_MFP1_PA5MFP_Msk)) |
                    (SYS_GPA_MFP1_PA4MFP_LPI2C0_SDA | SYS_GPA_MFP1_PA5MFP_LPI2C0_SCL);

    /* LPI2C pin enable schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN4_Msk | GPIO_SMTEN_SMTEN5_Msk;

    /* Lock protected registers */
    SYS_LockReg();
}

void LPI2C0_Init(void)
{
    /* Open LPI2C module and set bus clock */
    LPI2C_Open(LPI2C0, 100000);

    /* Get LPI2C0 Bus Clock */
    printf("LPI2C clock %d Hz\n", LPI2C_GetBusClockFreq(LPI2C0));
}

void LPI2C0_Close(void)
{
    /* Disable LPI2C0 interrupt and clear corresponding NVIC bit */
    LPI2C_DisableInt(LPI2C0);
    NVIC_DisableIRQ(LPI2C0_IRQn);

    /* Disable LPI2C0 and close LPI2C0 clock */
    LPI2C_Close(LPI2C0);
    CLK_DisableModuleClock(LPI2C0_MODULE);

}

int32_t main(void)
{
    uint32_t i;
    uint8_t txbuf[256] = {0}, rDataBuf[256] = {0};

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*
        This sample code sets LPI2C bus clock to 100kHz. Then, Master accesses Slave with Multi Bytes Write
        and Multi Bytes Read operations, and check if the read data is equal to the programmed data.
    */
    printf("+----------------------------------------------------------+\n");
    printf("| LPI2C Driver Sample Code for Multi Bytes Read/Write Test |\n");
    printf("| Needs to work with LPI2C_Slave sample code               |\n");
    printf("|                                                          |\n");
    printf("| LPI2C Master (LPI2C0) <---> LPI2C Slave(LPI2C0)          |\n");
    printf("| !! This sample code requires two borads to test !!       |\n");
    printf("+----------------------------------------------------------+\n");

    printf("\n");

    /* Init LPI2C0 */
    LPI2C0_Init();

    /* Slave address */
    g_u8DeviceAddr = 0x15;

    /* Prepare data for transmission */
    for(i = 0; i < 256; i++)
    {
        txbuf[i] = (uint8_t) i + 3;
    }

    for(i = 0; i < 256; i += 32)
    {
        /* Write 32 bytes data to Slave */
        while(LPI2C_WriteMultiBytesTwoRegs(LPI2C0, g_u8DeviceAddr, i, &txbuf[i], 32) < 32);
    }

    printf("Multi bytes Write access Pass.....\n");

    printf("\n");

    /* Use Multi Bytes Read from Slave (Two Registers) */
    while(LPI2C_ReadMultiBytesTwoRegs(LPI2C0, g_u8DeviceAddr, 0x0000, rDataBuf, 256) < 256);

    /* Compare TX data and RX data */
    for(i = 0; i < 256; i++)
    {
        if(txbuf[i] != rDataBuf[i])
            printf("Data compare fail... R[%d] Data: 0x%X\n", i, rDataBuf[i]);
    }
    printf("Multi bytes Read access Pass.....\n");

    while(1);

}
/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/

