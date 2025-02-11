/**************************************************************************//**
 * @file     main.c
 * @version  V1.01
 * @brief    Demonstrate output different duty waveform in Low Power TMR0~1 PWM.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
extern void initialise_monitor_handles(void);
#endif

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HIRC clock (Internal RC 48MHz) */
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

    /* Enable HCLK1 clock */
    CLK_EnableModuleClock(HCLK1_MODULE);

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(LPTMR0_MODULE);
    CLK_EnableModuleClock(LPTMR1_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(LPTMR0_MODULE, LPSCC_CLKSEL0_LPTMR0SEL_HIRC, 0);
    CLK_SetModuleClock(LPTMR1_MODULE, LPSCC_CLKSEL0_LPTMR0SEL_HIRC, 0);


    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set Low Power TMR0~1 PWM output pins */
    SYS->GPB_MFP1 = (SYS->GPB_MFP1 & (SYS_GPB_MFP1_PB5MFP_Msk | SYS_GPB_MFP1_PB4MFP_Msk)) | \
                    (SYS_GPB_MFP1_PB5MFP_LPTM0 | SYS_GPB_MFP1_PB4MFP_LPTM1);

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);

    printf("+---------------------------------------------------+\n");
    printf("|  Low Power TMR0~TMR1 PWM Output Duty Sample Code  |\n");
    printf("+---------------------------------------------------+\n\n");

    printf("# Low Power TMR0 PWM_CH0 output frequency is 18000 Hz and duty is 50%%.\n");
    printf("# Low Power TMR1 PWM_CH0 output frequency is 10000 Hz and duty is 10%%.\n");

    printf("# I/O configuration:\n");
    printf("    - Low Power TMR0 PWM_CH0 on PB.5\n");
    printf("    - Low Power TMR1 PWM_CH0 on PB.4\n");


    /* Change Timer to PWM counter mode */
    LPTPWM_ENABLE_PWM_MODE(LPTMR0);
    LPTPWM_ENABLE_PWM_MODE(LPTMR1);


    /* Enable output of PWM_CH0 */
    LPTPWM_ENABLE_OUTPUT(LPTMR0, TPWM_CH0);
    LPTPWM_ENABLE_OUTPUT(LPTMR1, TPWM_CH0);

    /* Set Low Power TMR0 PWM output frequency is 18000 Hz, duty 50% in up count type */
    if (LPTPWM_ConfigOutputFreqAndDuty(LPTMR0, 18000, 50) != 18000)
    {
        printf("Set the frequency different from the user\n");
    }

    /* Set Low Power TMR1 PWM output frequency is 10000 Hz, duty 10% in up count type */
    if (LPTPWM_ConfigOutputFreqAndDuty(LPTMR1, 10000, 10) != 10000)
    {
        printf("Set the frequency different from the user\n");
    }

    /* Start Low Power TMR PWM counter */
    TPWM_START_COUNTER(LPTMR0);
    TPWM_START_COUNTER(LPTMR1);

    printf("*** Check LPTMR0~LPTMR1 PWM_CH0 output waveform by oscilloscope ***\n");

    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
