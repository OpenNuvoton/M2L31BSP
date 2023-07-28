/***************************************************************************//**
 * @file     rmc_user.c
 * @brief    M030G series RMC driver source file
 * @version  2.0.0
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "rmc_user.h"

#if 0
#define ISPCTL      ISPCON
#define ISPADDR     ISPADR
#define RMC_ISPCTL_ISPFF_Msk        RMC_ISPCON_ISPFF_Msk
#endif

int RMC_Proc(uint32_t u32Cmd, uint32_t addr_start, uint32_t addr_end, uint32_t *data)
{
    unsigned int u32Addr, Reg;

    for (u32Addr = addr_start; u32Addr < addr_end; data++)
    {
        RMC->ISPCMD = u32Cmd;
        RMC->ISPADDR = u32Addr;

        if (u32Cmd == RMC_ISPCMD_PROGRAM)
        {
            RMC->ISPDAT = *data;
        }

        RMC->ISPTRG = 0x1;
        __ISB();

        while (RMC->ISPTRG & 0x1) ;  /* Wait for ISP command done. */

        Reg = RMC->ISPCTL;

        if (Reg & RMC_ISPCTL_ISPFF_Msk)
        {
            RMC->ISPCTL = Reg;
            return (-1);
        }

        if (u32Cmd == RMC_ISPCMD_READ)
        {
            *data = RMC->ISPDAT;
        }

        if (u32Cmd == RMC_ISPCMD_PAGE_ERASE)
        {
            u32Addr += RMC_FLASH_PAGE_SIZE;
        }
        else
        {
            u32Addr += 4;
        }
    }

    return (0);
}

void UpdateConfig(uint32_t *data, uint32_t *res)
{
    unsigned int u32Size = CONFIG_SIZE;
    RMC_ENABLE_CFG_UPDATE();
    RMC_Proc(RMC_ISPCMD_PAGE_ERASE, Config0, Config0 + 8, 0);
    RMC_Proc(RMC_ISPCMD_PROGRAM, Config0, Config0 + u32Size, data);

    if (res)
    {
        RMC_Proc(RMC_ISPCMD_READ, Config0, Config0 + u32Size, res);
    }

    RMC_DISABLE_CFG_UPDATE();
}
