/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Show how to set LPI2C in Slave mode and receive the data from Master.
 *           This sample code needs to work with LPI2C_Master.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8SlvDataLen;
volatile uint32_t slave_buff_addr;
volatile uint8_t g_au8SlvData[256];
volatile uint8_t g_au8SlvRxData[3];

typedef void (*LPI2C_FUNC)(uint32_t u32Status);

volatile static LPI2C_FUNC s_LPI2C0HandlerFn = NULL;


void LPI2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = LPI2C_GET_STATUS(LPI2C0);

    if (LPI2C_GET_TIMEOUT_FLAG(LPI2C0))
    {
        /* Clear LPI2C0 Timeout Flag */
        LPI2C_ClearTimeoutFlag(LPI2C0);
    }
    else
    {
        if (s_LPI2C0HandlerFn != NULL)
            s_LPI2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPI2C TRx Callback Function                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void LPI2C_SlaveTRx(uint32_t u32Status)
{
    uint8_t u8Data;

    if(u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8SlvDataLen = 0;
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
    }
    else if(u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        u8Data = (unsigned char) LPI2C_GET_DATA(LPI2C0);
        if(g_u8SlvDataLen < 2)
        {
            g_au8SlvRxData[g_u8SlvDataLen++] = u8Data;
            slave_buff_addr = (uint32_t)(g_au8SlvRxData[0] << 8) + g_au8SlvRxData[1];
        }
        else
        {
            g_au8SlvData[slave_buff_addr++] = u8Data;
            if(slave_buff_addr == 256)
            {
                slave_buff_addr = 0;
            }
        }

        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {
        LPI2C_SET_DATA(LPI2C0, g_au8SlvData[slave_buff_addr]);
        slave_buff_addr++;
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
    }
    else if(u32Status == 0xB8)                  /* Data byte in I2CDAT has been transmitted ACK has been received */
    {
        LPI2C_SET_DATA(LPI2C0, g_au8SlvData[slave_buff_addr++]);
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
    }
    else if(u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
    }
    else if(u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u8SlvDataLen = 0;
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8SlvDataLen = 0;
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

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
    CLK_EnableModuleClock(UART1_MODULE);

    /* Enable LPI2C0 clock */
    CLK_EnableModuleClock(LPI2C0_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPA_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL4_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set GPA multi-function pins for UART1 RXD(PA.8) and TXD(PA.9) */
    Uart1DefaultMPF();

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
    /* Open LPI2C0 and set clock to 100k */
    LPI2C_Open(LPI2C0, 100000);

    /* Get LPI2C0 Bus Clock */
    printf("LPI2C clock %d Hz\n", LPI2C_GetBusClockFreq(LPI2C0));

    /* Set LPI2C0 4 Slave Addresses */
    LPI2C_SetSlaveAddr(LPI2C0, 0, 0x15, LPI2C_GCMODE_DISABLE);   /* Slave Address : 0x15 */
    LPI2C_SetSlaveAddr(LPI2C0, 1, 0x35, LPI2C_GCMODE_DISABLE);   /* Slave Address : 0x35 */
    LPI2C_SetSlaveAddr(LPI2C0, 2, 0x55, LPI2C_GCMODE_DISABLE);   /* Slave Address : 0x55 */
    LPI2C_SetSlaveAddr(LPI2C0, 3, 0x75, LPI2C_GCMODE_DISABLE);   /* Slave Address : 0x75 */

    LPI2C_SetSlaveAddrMask(LPI2C0, 0, 0x01);
    LPI2C_SetSlaveAddrMask(LPI2C0, 1, 0x04);
    LPI2C_SetSlaveAddrMask(LPI2C0, 2, 0x01);
    LPI2C_SetSlaveAddrMask(LPI2C0, 3, 0x04);

    LPI2C_EnableInt(LPI2C0);
    NVIC_EnableIRQ(LPI2C0_IRQn);
}

int32_t main (void)
{
    uint32_t i;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART1, 115200);

    /*
        This sample code sets LPI2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+-------------------------------------------------------+\n");
    printf("|             LPI2C Driver Sample Code(Slave)           |\n");
    printf("+-------------------------------------------------------+\n");

    /* Init LPI2C0 */
    LPI2C0_Init();

    /* LPI2C enter no address SLV mode */
    LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI | LPI2C_CTL_AA);

    for (i = 0; i < 0x100; i++)
    {
        g_au8SlvData[i] = 0;
    }

    /* LPI2C function to Slave receive/transmit data */
    s_LPI2C0HandlerFn=LPI2C_SlaveTRx;

    printf("\n");
    printf("LPI2C Slave Mode is Running.\n");

    while(1);
}
