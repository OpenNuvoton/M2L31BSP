/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2020 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

//***********************************************************************************************************
//  Website: http://www.nuvoton.com
//  E-Mail : MicroC-8bit@nuvoton.com
//***********************************************************************************************************

//***********************************************************************************************************
//  ML56 simple UART0 using timer 1
//  Using UART0 to communication with NuSenAdj tool which is used for calibration the specified touch key.
//  Further, use the tool for RAW data view for each key.
//***********************************************************************************************************

#include "NuMicro.h"
#include "TK_Demo.h"

#ifdef DEMO_CALIBRATION

//#define set_SMOD    PCON    |= SET_BIT7
//#define set_T1M     CKCON   |= SET_BIT4
//#define clr_BRCK    T3CON   &= ~SET_BIT5
//#define set_TR1     TR1      = 1

//void InitialUART0_Timer1(unsigned long Fsys, uint32_t u32Baudrate)
//{
//    SCON = 0x52;     //UART0 Mode1,REN=1,TI=1
//    TMOD |= 0x20;    //Timer1 Mode1

//    set_SMOD;
//    set_T1M;         //FOSC/ 1
//    clr_BRCK;        //Serial port 0 baud rate clock source = Timer1

//    //TH1 = 256-((Fsys>>4)/u32Baudrate);
//    TH1 = 0xF3;
//    set_TR1;
//}

void UART0_Init(void)
{

#ifdef BOARD_TIM
    /* Tim */
    //MFP_P30_UART0_RXD;      /* set P3.0 and P3.1 as Quasi mode for UART0 trasnfer */
    //MFP_P31_UART0_TXD;
    //P32_QUASI_MODE;
    //P31_QUASI_MODE;
//    MFP_P52_UART0_RXD;      /* set P3.0 and P3.1 as Quasi mode for UART0 trasnfer */
//    MFP_P53_UART0_TXD;
//    P52_QUASI_MODE;
//    P53_QUASI_MODE;

#else //BOARD_TIM

    /* PY */
//    MFP_P64_UART0_RXD;      /* set P6.4 and P6.5 as Quasi mode for UART0 trasnfer */
//    MFP_P65_UART0_TXD;
//    P64_QUASI_MODE;
//    P65_QUASI_MODE;

#endif //BOARD_TIM
//    SFRS = 0x00;
//    InitialUART0_Timer1(24000000, 115200);
//    set_SCON_TI;
//    IP = IP | BIT4;
//    ES=1;


    /* Configure UART0 and set UART0 baud rate */
    SYS->IPRST1 = SYS->IPRST1 | SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 = SYS->IPRST1 & ~SYS_IPRST1_UART0RST_Msk;
    UART_Open(UART0, 115200);
    //UART0->FIFO = (UART0->FIFO &~UART_FIFO_RFITL_Msk)| UART_FIFO_RFITL_4BYTES;
    UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk);
    NVIC_EnableIRQ(UART0_IRQn);

}
#endif //DEMO_CALIBRATION

#ifdef USE_DEBUG_PORT
//***********************************************************************************************************
//  ML56 simple UART1 using timer 3
//  Using UART1 for debug purpose.
//
//***********************************************************************************************************

//#define set_TR3     T3CON   |= SET_BIT3

//void InitialUART1_Timer3(unsigned long Fsys, UINT32 u32Baudrate) //use timer3 as Baudrate generator
//{
//    S1CON = 0x52;          //UART1 Mode1,REN_1=1,TI_1=1
//    T3CON &= 0xF8;         //T3PS2=0,T3PS1=0,T3PS0=0(Prescale=1)

//    //RH3 = HIBYTE(65536-((Fsys>>4)/u32Baudrate));
//    //RL3 = LOBYTE(65536-((Fsys>>4)/u32Baudrate));
//    RH3 = 0xFF;
//    RL3 = 0xF3;
//    set_TR3;               //Trigger Timer3
//}

void UART1_Init(void)
{
#ifdef BOARD_TIM  /* HCKuo */
//    MFP_P10_UART1_RXD;      /* set P6.4 and P6.5 as Quasi mode for UART0 trasnfer */
//    MFP_P27_UART1_TXD;
//    P10_QUASI_MODE;
//    P27_QUASI_MODE;
#else
//    MFP_P26_UART1_RXD;      /* set P6.4 and P6.5 as Quasi mode for UART0 trasnfer */
//    MFP_P27_UART1_TXD;
//    P26_QUASI_MODE;
//    P27_QUASI_MODE;
#endif
//    SFRS = 0x00;
//    InitialUART1_Timer3(24000000, 115200);
//    SFRS = 0x00;
//    set_SCON_TI;

    /* Configure UART0 and set UART0 baud rate */
    SYS->IPRST1 = SYS->IPRST1 | SYS_IPRST1_UART1RST_Msk;
    SYS->IPRST1 = SYS->IPRST1 & ~SYS_IPRST1_UART1RST_Msk;
    UART_Open(UART1, 115200);

}

void UART3_Init(void)
{
    /* Configure UART3 and set UART3 baud rate */
    SYS->IPRST1 = SYS->IPRST1 | SYS_IPRST1_UART3RST_Msk;
    SYS->IPRST1 = SYS->IPRST1 & ~SYS_IPRST1_UART3RST_Msk;
    UART_Open(UART3, 115200);
}

//#define clr_TI_1    TI_1  = 0
//#define clr_RI_1    RI_1  = 0
#if 0
#define XON             0x11
#define XOFF            0x13
/***********************************************************************/
/* putchar (full version):  expands '\n' into CR LF and handles        */
/*                          XON/XOFF (Ctrl+S/Ctrl+Q) protocol          */
/* %d     int     2 bytes                                              */
/* %bd    char    1 bytes                                              */
/* %ld    long    4 bytes                                              */
/* %x     int     2 bytes                                              */
/* %bx    char    1 bytes                                              */
/* %lx    long    4 bytes                                              */
/***********************************************************************/
char putchar (char c)
{
    if (c == '\n')
    {
        if (RI_1)
        {
            if (SBUF1 == XOFF)
            {
                do
                {
                    clr_RI_1;
                    while (!RI_1);
                }
                while (SBUF1 != XON);
                clr_RI_1;
            }
        }
        while (!TI_1);
        clr_TI_1;
        SBUF1 = 0x0d;                          /* output CR  */
    }
    if (RI_1)
    {
        if (SBUF1 == XOFF)
        {
            do
            {
                clr_RI_1;
                while (!RI_1);
            }
            while (SBUF1 != XON);
            clr_RI_1;
        }
    }
    while (!TI_1);
    clr_TI_1;

    return (SBUF1 = c);
}
#endif
#if 0 /* code size is less about 49 bytes than above */
/***********************************************************************/
/* putchar (basic version): expands '\n' into CR LF                    */
/***********************************************************************/
char putchar (char c)
{
#if 1 /* UART1 */
    if (c == '\n')
    {
        while (!TI_1);
        TI_1 = 0;
        SBUF1 = 0x0d;                         /* output CR  */
    }
    while (!TI_1);
    TI_1 = 0;
    return (SBUF1 = c);
#else /* UART0 */
    if (c == '\n')
    {
        while (!TI);
        TI = 0;
        SBUF = 0x0d;                         /* output CR  */
    }
    while (!TI);
    TI = 0;
    return (SBUF = c);
#endif
}
#endif
/***********************************************************************/
/*  putchar (mini version): outputs charcter only                      */
/***********************************************************************/
#if 0
char putchar (char c)
{
#if 1
    while (!TI_1);
    TI_1 = 0;
    return (SBUF1 = c);
#else
    while (!TI);
    TI = 0;
    return (SBUF = c);
#endif
}
#endif /* #if 0 */

/***********************************************************************/
/*  This file is part of the C51 Compiler package                      */
/*  Copyright KEIL ELEKTRONIK GmbH 1993 - 2002                         */
/***********************************************************************/
/*                                                                     */
/*  GETKEY.C:  This routine is the general character input of C51.     */
/*  You may add this file to a uVision2 project.                       */
/*                                                                     */
/*  To translate this file use C51 with the following invocation:      */
/*     C51 GETKEY.C  <memory model>                                    */
/*                                                                     */
/*  To link the modified GETKEY.OBJ file to your application use the   */
/*  following Lx51 invocation:                                         */
/*     Lx51 <your object file list>, GETKEY.OBJ <controls>             */
/*                                                                     */
/***********************************************************************/
//char _getkey ()
//{
//    char c;

//    while (!RI_1);
//    c = SBUF1;
//    RI_1 = 0;
//    return (c);
//}




#endif /* OPT_UART1 */