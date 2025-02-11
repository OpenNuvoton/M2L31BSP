/**************************************************************************//**
 * @file     isp_user.c
 * @brief    ISP Command source file
 * @version  0x32
 * @date     14, June, 2017
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "isp_user.h"
#include "rmc_user.h"

volatile uint8_t bISPDataReady;
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t response_buff[64];
static uint8_t aprom_buf[RMC_FLASH_PAGE_SIZE];
#else
uint8_t response_buff[64] __attribute__((aligned(4)));
static uint8_t aprom_buf[RMC_FLASH_PAGE_SIZE] __attribute__((aligned(4)));
#endif

uint32_t bUpdateApromCmd;
uint32_t g_apromSize, g_dataFlashAddr, g_dataFlashSize;
volatile uint8_t bISPDataReady;

__STATIC_INLINE uint16_t Checksum(unsigned char *buf, int len)
{
    int i;
    uint16_t c;

    for (c = 0, i = 0 ; i < len; i++)
    {
        c += buf[i];
    }

    return (c);
}

int ParseCmd(unsigned char *buffer, uint8_t len)
{
    static uint32_t StartAddress, TotalLen, LastDataLen, g_packno = 1;
    uint8_t *response;
    uint16_t lcksum;
    uint32_t lcmd, srclen, i, regcnf0, security;
    unsigned char *pSrc;
    static uint32_t gcmd;
    response = response_buff;
    pSrc = buffer;
    srclen = len;
    lcmd = inpw(pSrc);
    outpw(response + 4, 0);
    pSrc += 8;
    srclen -= 8;
    ReadData(Config0, Config0 + 44, (uint32_t *)(response + 8)); /*read config */
    regcnf0 = *(uint32_t *)(response + 8);
    security = regcnf0 & 0x2;

    if (lcmd == CMD_SYNC_PACKNO)
    {
        g_packno = inpw(pSrc);
    }

    if ((lcmd) && (lcmd != CMD_RESEND_PACKET))
    {
        gcmd = lcmd;
    }

    if (lcmd == CMD_GET_FWVER)
    {
        response[8] = FW_VERSION;
    }
    else if (lcmd == CMD_GET_DEVICEID)
    {
        outpw(response + 8, SYS->PDID);
        goto out;
    }
    else if (lcmd == CMD_RUN_APROM || lcmd == CMD_RUN_LDROM || lcmd == CMD_RESET)
    {
        SYS->RSTSTS = 3; /*clear bit*/

        /* Set BS */
        if (lcmd == CMD_RUN_APROM)
        {
            i = (RMC->ISPCTL & 0xFFFFFFFC);
        }
        else if (lcmd == CMD_RUN_LDROM)
        {
            i = (RMC->ISPCTL & 0xFFFFFFFC);
            i |= 0x00000002;
        }
        else
        {
            i = (RMC->ISPCTL & 0xFFFFFFFE);/* ISP disable */
        }

        RMC->ISPCTL = i;
        SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

        /* Trap the CPU */
        while (1);
    }
    else if (lcmd == CMD_CONNECT)
    {
        g_packno = 1;
        goto out;
    }
    else if ((lcmd == CMD_UPDATE_APROM) || (lcmd == CMD_ERASE_ALL))
    {
        if (lcmd == CMD_ERASE_ALL)
        {
            memset((uint32_t *)aprom_buf,0xFF,RMC_FLASH_PAGE_SIZE);
            TotalLen = g_apromSize;
            StartAddress = 0;
            do {
                WriteData(StartAddress, StartAddress+ RMC_FLASH_PAGE_SIZE, (uint32_t *)&aprom_buf);
                TotalLen -= RMC_FLASH_PAGE_SIZE;
                StartAddress += RMC_FLASH_PAGE_SIZE;
                
            } while ( TotalLen >0);  
            *(uint32_t *)(response + 8) = regcnf0 | 0x02;
            UpdateConfig((uint32_t *)(response + 8), NULL);
        }

        bUpdateApromCmd = TRUE;
    }  
    else if (lcmd == CMD_GET_FLASHMODE)
    {
        //return 1: APROM, 2: LDROM
        outpw(response + 8, (RMC->ISPCTL & 0x2) ? 2 : 1);
    }    
    
    if ((lcmd == CMD_UPDATE_APROM) || (lcmd == CMD_UPDATE_DATAFLASH))
    {
        if (lcmd == CMD_UPDATE_DATAFLASH)
        {
            StartAddress = g_dataFlashAddr;

            if (g_dataFlashSize)   /*g_dataFlashAddr*/
            {

            }
            else
            {
                goto out;
            }
        }
        else
        {
            StartAddress = 0;
        }

        TotalLen = inpw(pSrc + 4);
        pSrc += 8;
        srclen -= 8;
    }
    else if (lcmd == CMD_UPDATE_CONFIG)
    {
        if ((security == 0) && (!bUpdateApromCmd))   /*security lock*/
        {
            goto out;
        }

        UpdateConfig((uint32_t *)(pSrc), (uint32_t *)(response + 8));
        GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
        goto out;
    }
    else if (lcmd == CMD_RESEND_PACKET)     /*for APROM&Data flash only*/
    {
        uint32_t PageAddress;
        StartAddress -= LastDataLen;
        TotalLen += LastDataLen;
        PageAddress = StartAddress & (0x100000 - RMC_FLASH_PAGE_SIZE);

        if (PageAddress >= Config0)
        {
            goto out;
        }

        ReadData(PageAddress, StartAddress, (uint32_t *)aprom_buf);
        WriteData(PageAddress, StartAddress, (uint32_t *)aprom_buf);

        if ((StartAddress % RMC_FLASH_PAGE_SIZE) >= (RMC_FLASH_PAGE_SIZE - LastDataLen))
        {

        }

        goto out;
    }

    if ((gcmd == CMD_UPDATE_APROM) || (gcmd == CMD_UPDATE_DATAFLASH))
    {
        if (TotalLen < srclen)
        {
            srclen = TotalLen;/*prevent last package from over writing*/
        }

        TotalLen -= srclen;
        WriteData(StartAddress, StartAddress + srclen, (uint32_t *)pSrc);
        memset(pSrc, 0, srclen);
        ReadData(StartAddress, StartAddress + srclen, (uint32_t *)pSrc);
        StartAddress += srclen;
        LastDataLen =  srclen;
    }

out:
    lcksum = Checksum(buffer, len);
    outps(response, lcksum);
    ++g_packno;
    outpw(response + 4, g_packno);
    g_packno++;
    return (0);
}

