/**************************************************************************//**
 * @file     main.c
 * @version  V1.0
 * @brief    Configure EBI interface to access BS616LV4017 (SRAM) on EBI interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PDMA_CH     0

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
extern void SRAM_BS616LV4017(uint32_t u32MaxSize);
void AccessEBIWithPDMA(void);

void Configure_EBI_16BIT_Pins(void)
{
    /* EBI AD[5:0]=PC[5:0] */
    SYS->GPC_MFP0 &= ~(SYS_GPC_MFP0_PC3MFP_Msk | SYS_GPC_MFP0_PC2MFP_Msk
                       | SYS_GPC_MFP0_PC1MFP_Msk | SYS_GPC_MFP0_PC0MFP_Msk);
    SYS->GPC_MFP1 &= ~(SYS_GPC_MFP1_PC4MFP_Msk | SYS_GPC_MFP1_PC5MFP_Msk);
    SYS->GPC_MFP0 |= SYS_GPC_MFP0_PC3MFP_EBI_AD3 | SYS_GPC_MFP0_PC2MFP_EBI_AD2
                     | SYS_GPC_MFP0_PC1MFP_EBI_AD1 | SYS_GPC_MFP0_PC0MFP_EBI_AD0;
    SYS->GPC_MFP1 |= SYS_GPC_MFP1_PC5MFP_EBI_AD5 | SYS_GPC_MFP1_PC4MFP_EBI_AD4;

    /* AD[7:6]=PA[7:6] */
    SYS->GPA_MFP1 &= ~(SYS_GPA_MFP1_PA6MFP_Msk | SYS_GPA_MFP1_PA7MFP_Msk);
    SYS->GPA_MFP1 |= SYS_GPA_MFP1_PA7MFP_EBI_AD7 | SYS_GPA_MFP1_PA6MFP_EBI_AD6;

    /* AD[9:8]=PC[7:6] */
    SYS->GPC_MFP1 &= ~(SYS_GPC_MFP1_PC6MFP_Msk | SYS_GPC_MFP1_PC7MFP_Msk);
    SYS->GPC_MFP1 |= SYS_GPC_MFP1_PC7MFP_EBI_AD9 | SYS_GPC_MFP1_PC6MFP_EBI_AD8;

    /* AD[13:10]=PD[0:3] */
    SYS->GPD_MFP0 &= ~(SYS_GPD_MFP0_PD3MFP_Msk | SYS_GPD_MFP0_PD2MFP_Msk
                       | SYS_GPD_MFP0_PD1MFP_Msk | SYS_GPD_MFP0_PD0MFP_Msk);
    SYS->GPD_MFP0 |= (SYS_GPD_MFP0_PD1MFP_EBI_AD12 | SYS_GPD_MFP0_PD0MFP_EBI_AD13
                      | SYS_GPD_MFP0_PD2MFP_EBI_AD11 | SYS_GPD_MFP0_PD3MFP_EBI_AD10);

    /* AD[15:14]=PB[12:13] */
    SYS->GPB_MFP3 &= ~(SYS_GPB_MFP3_PB12MFP_Msk | SYS_GPB_MFP3_PB13MFP_Msk);
    SYS->GPB_MFP3 |= SYS_GPB_MFP3_PB13MFP_EBI_AD14 | SYS_GPB_MFP3_PB12MFP_EBI_AD15;

    /* ADR[19:16]=PB[8:11] */
    SYS->GPB_MFP2 &= ~(SYS_GPB_MFP2_PB8MFP_Msk | SYS_GPB_MFP2_PB9MFP_Msk
                       | SYS_GPB_MFP2_PB10MFP_Msk | SYS_GPB_MFP2_PB11MFP_Msk);
    SYS->GPB_MFP2 |= (SYS_GPB_MFP2_PB11MFP_EBI_ADR16 | SYS_GPB_MFP2_PB10MFP_EBI_ADR17
                      | SYS_GPB_MFP2_PB9MFP_EBI_ADR18 | SYS_GPB_MFP2_PB8MFP_EBI_ADR19);

    /* #WRL and #WRH pins on PB.7 and PB.6 */
    SYS->GPB_MFP1 &= ~(SYS_GPB_MFP1_PB7MFP_Msk | SYS_GPB_MFP1_PB6MFP_Msk);
    SYS->GPB_MFP1 |= SYS_GPB_MFP1_PB7MFP_EBI_nWRL | SYS_GPB_MFP1_PB6MFP_EBI_nWRH;

    /* #RD and #WR pins on PA.11 and PA.10 */
    SYS->GPA_MFP2 &= ~(SYS_GPA_MFP2_PA11MFP_Msk  | SYS_GPA_MFP2_PA10MFP_Msk);
    SYS->GPA_MFP2 |= (SYS_GPA_MFP2_PA11MFP_EBI_nRD  | SYS_GPA_MFP2_PA10MFP_EBI_nWR);

    /* #CS[0:2]=PD[12:10] */
    SYS->GPD_MFP3 &= ~(SYS_GPD_MFP3_PD12MFP_Msk);
    SYS->GPD_MFP3 |= SYS_GPD_MFP3_PD12MFP_EBI_nCS0;
    SYS->GPD_MFP2 &= ~(SYS_GPD_MFP2_PD11MFP_Msk | SYS_GPD_MFP2_PD10MFP_Msk);
    SYS->GPD_MFP2 |= (SYS_GPD_MFP2_PD11MFP_EBI_nCS1 | SYS_GPD_MFP2_PD10MFP_EBI_nCS2);

    /* ALE and MCLK pins on PE.2 and PE.3 */
    SYS->GPE_MFP0 &= ~(SYS_GPE_MFP0_PE2MFP_Msk | SYS_GPE_MFP0_PE3MFP_Msk);
    SYS->GPE_MFP0 |= (SYS_GPE_MFP0_PE2MFP_EBI_ALE | SYS_GPE_MFP0_PE3MFP_EBI_MCLK);
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

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL4_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART1_MODULE);

    /* Enable EBI peripheral clock */
    CLK_EnableModuleClock(EBI_MODULE);

    /* Enable PDMA peripheral clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    Uart1DefaultMPF();
}

void UART1_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART1 and set UART1 Baudrate */
    UART_Open(UART1, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART1 for printf */
    UART1_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+--------------------------------------------------------+\n");
    printf("|    EBI SRAM Sample Code on Bank0 with PDMA transfer    |\n");
    printf("+--------------------------------------------------------+\n\n");

    printf("************************************************************************\n");
    printf("* Please connect BS616LV4017 SRAM to EBI bank0 before accessing !!     *\n");
    printf("* EBI pins settings:                                                   *\n");
    printf("*                                                                      *\n");
    printf("*   - AD0 ~ AD5     on PC.0 ~ PC.5                                     *\n");
    printf("*   - AD6 ~ AD7     on PA.6 ~ PA.7                                     *\n");
    printf("*   - AD8 ~ AD9     on PC.6 ~ PC.7                                     *\n");
    printf("*   - AD10 ~ AD11   on PD.3 ~ PD.2                                     *\n");
    printf("*   - AD12 ~ AD13   on PD.1 ~ PD.0                                     *\n");
    printf("*   - AD14 ~ AD15   on PB.13 ~ PB.12                                   *\n");
    printf("*   - ADR16 ~ ADR17 on PB.11 ~ PB.10                                   *\n");
    printf("*   - ADR18 ~ ADR19 on PB.9  ~ PB.8                                    *\n");
    printf("*   - nWR           on PA.10                                           *\n");
    printf("*   - nRD           on PA.11                                           *\n");
    printf("*   - nWRL          on PB.7                                            *\n");
    printf("*   - nWRH          on PB.6                                            *\n");
    printf("*   - nCS0          on PD.12                                           *\n");
    printf("*   - nCS1          on PD.11                                           *\n");
    printf("*   - nCS2          on PD.10                                           *\n");
    printf("*   - ALE           on PE.2                                            *\n");
    printf("*   - MCLK          on PE.3                                            *\n");
    printf("*                                                                      *\n\n");
    printf("************************************************************************\n\n");

    /* Configure multi-function pins for EBI 16-bit application */
    Configure_EBI_16BIT_Pins();

    /* Initialize EBI bank0 to access external SRAM */
    EBI_Open(EBI_BANK0, EBI_BUSWIDTH_16BIT, EBI_TIMING_NORMAL, 0, EBI_CS_ACTIVE_LOW);

    /* Start to test EBI SRAM */
    SRAM_BS616LV4017(512 * 1024);

    /* EBI SRAM with PDMA test */
    AccessEBIWithPDMA();

    /* Disable EBI function */
    EBI_Close(EBI_BANK0);

    /* Disable EBI clock */
    CLK_DisableModuleClock(EBI_MODULE);

    printf("*** SRAM Test OK ***\n");

    while (1);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables for PDMA                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t PDMA_TEST_LENGTH = 64;
uint32_t SrcArray[64];
uint32_t DestArray[64];
uint32_t volatile u32IsTestOver = 0;

void PDMA0_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if (status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
        if (PDMA_GET_ABORT_STS(PDMA) & PDMA_ABTSTS_ABTIF0_Msk)
            u32IsTestOver = 2;

        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF0_Msk);
    }
    else if (status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        if (PDMA_GET_TD_STS(PDMA) & PDMA_TDSTS_TDIF0_Msk)
            u32IsTestOver = 1;

        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF0_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

void AccessEBIWithPDMA(void)
{
    uint32_t i;
    uint32_t u32Result0 = 0x5A5A, u32Result1 = 0x5A5A;

    printf("[[ Access EBI with PDMA ]]\n");

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA0_MODULE);

    for (i = 0; i < 64; i++)
    {
        SrcArray[i] = 0x76570000 + i;
        u32Result0 += SrcArray[i];
    }

    /* Open Channel 0 */
    PDMA_Open(PDMA0, (1 << PDMA_CH));

    //burst size is 4
    PDMA_SetBurstType(PDMA0, PDMA_CH, PDMA_REQ_BURST, PDMA_BURST_4);

    /* transfer width is one word(32 bit) */
    PDMA_SetTransferCnt(PDMA0, PDMA_CH, PDMA_WIDTH_32, PDMA_TEST_LENGTH);
    PDMA_SetTransferAddr(PDMA0, PDMA_CH, (uint32_t)SrcArray, PDMA_SAR_INC, EBI_BANK0_BASE_ADDR, PDMA_DAR_INC);
    PDMA_SetTransferMode(PDMA0, PDMA_CH, PDMA_MEM, FALSE, 0);

    PDMA_EnableInt(PDMA0, PDMA_CH, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);

    u32IsTestOver = 0;
    PDMA_Trigger(PDMA0, PDMA_CH);

    while (u32IsTestOver == 0);

    /* Transfer internal SRAM to EBI SRAM done */

    /* Clear internal SRAM data */
    for (i = 0; i < 64; i++)
    {
        SrcArray[i] = 0x0;
    }

    /* transfer width is one word(32 bit) */
    PDMA_SetTransferCnt(PDMA0, PDMA_CH, PDMA_WIDTH_32, PDMA_TEST_LENGTH);
    PDMA_SetTransferAddr(PDMA0, PDMA_CH, EBI_BANK0_BASE_ADDR, PDMA_SAR_INC, (uint32_t)SrcArray, PDMA_DAR_INC);
    PDMA_SetTransferMode(PDMA0, PDMA_CH, PDMA_MEM, FALSE, 0);

    u32IsTestOver = 0;
    PDMA_Trigger(PDMA0, PDMA_CH);

    while (u32IsTestOver == 0);

    /* Transfer EBI SRAM to internal SRAM done */
    for (i = 0; i < 64; i++)
    {
        u32Result1 += SrcArray[i];
    }

    if (u32IsTestOver == 1)
    {
        if ((u32Result0 == u32Result1) && (u32Result0 != 0x5A5A))
        {
            printf("        PASS (0x%X)\n\n", u32Result0);
        }
        else
        {
            printf("        FAIL - data matched (0x%X)\n\n", u32Result0);
            while (1);
        }
    }
    else
    {
        printf("        PDMA fail\n\n");
        while (1);
    }

    PDMA_Close(PDMA);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
