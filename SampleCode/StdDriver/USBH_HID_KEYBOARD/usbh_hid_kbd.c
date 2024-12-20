/**************************************************************************//**
 * @file     usbh_hid_kbd.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 23/02/24 4:40p $
 * @brief    USB Host HID keyboard driver
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "usbh_lib.h"
#include "usbh_hid.h"
#include "usbh_hid_kbd.h"

static struct hid_kbd_dev s_kbd_dev;    /* Support one keyboard device at the same time.         */
/* If you want to support mulitiple keyboards, please
   implement an array and handle it.                     */

static const uint8_t au8NumKeys[] = { '!', '@', '#', '$', '%', '^', '&', '*', '(', ')' };
static const uint8_t au8SymKeysUp[] = { '_', '+', '{', '}', '|', '~', ':', '"', '~', '<', '>', '?' };
static const uint8_t au8SymKeysLo[] = { '-', '=', '[', ']', '\\', ' ', ';', '\'', '`', ',', '.', '/' };
static const uint8_t au8PadKeys[] = { '/', '*', '-', '+', 0x13 };

void print_key(uint8_t u8Mod, uint8_t u8Key);
uint8_t oem_key_to_ascii(HID_DEV_T *hdev, uint8_t u8Mod, uint8_t u8Key);
uint8_t update_locking_keys(HID_DEV_T *hdev, uint8_t u8Key);
int kbd_parse_report(HID_DEV_T *hdev, uint8_t *pu8Buf, int i8Len);

void print_key(uint8_t u8Mod, uint8_t u8Key)
{
    printf(" mod => ");

    if(u8Mod & LeftCtrl)
        printf("L-Ctrl");
    if(u8Mod & LeftShift)
        printf("L-Shift");
    if(u8Mod & Alt)
        printf("Alt");
    if(u8Mod & LeftCmd)
        printf("L-Cmd");
    if(u8Mod & RightCtrl)
        printf("R-Ctrl");
    if(u8Mod & RightShift)
        printf("R-Shift");
    if(u8Mod & AltGr)
        printf("AltGr");
    if(u8Mod & RightCmd)
        printf("R-Cmd");

    printf("  %c\n", u8Key);
}

/*
 *  Translate OEM key to ASCII code.
 */
uint8_t oem_key_to_ascii(HID_DEV_T *hdev, uint8_t u8Mod, uint8_t u8Key)
{
    uint8_t u8Shift = (u8Mod & 0x22);

    (void)hdev;

    // [a-z]
    if((u8Key > 0x03) && (u8Key < 0x1e))
    {
        // Upper case letters
        if((!(s_kbd_dev.bLED & LED_CapsLoock) && (u8Mod & 2)) ||
                ((s_kbd_dev.bLED & LED_CapsLoock) && ((u8Mod & 2) == 0)))
            return (u8Key - 4 + 'A');

        // Lower case letters
        else
            return (u8Key - 4 + 'a');
    }
    // Numbers
    else if((u8Key > 0x1d) && (u8Key < 0x27))
    {
        if(u8Shift)
            return (au8NumKeys[u8Key - 0x1e]);
        else
            return (u8Key - 0x1e + '1');
    }
    // Keypad Numbers
    else if(u8Key > 0x58 && u8Key < 0x62)
    {
        if(s_kbd_dev.bLED & LED_NumLock)
            return (u8Key - 0x59 + '1');
    }
    else if((u8Key > 0x2c) && (u8Key < 0x39))
        return ((u8Shift) ? au8SymKeysUp[u8Key - 0x2d] : au8SymKeysLo[u8Key - 0x2d]);
    else if((u8Key > 0x53) && (u8Key < 0x59))
        return au8PadKeys[u8Key - 0x54];
    else
    {
        switch(u8Key)
        {
        case KEY_SPACE:
            return (0x20);
        case KEY_ENTER:
            return (0x13);
        case KEY_ZERO:
            return ((u8Shift) ? ')' : '0');
        case KEY_ZERO2:
            return ((s_kbd_dev.bLED & LED_NumLock) ? '0' : 0);
        case KEY_PERIOD:
            return ((s_kbd_dev.bLED & LED_NumLock) ? '.' : 0);
        }
    }
    return 0;
}

uint8_t update_locking_keys(HID_DEV_T *hdev, uint8_t u8Key)
{
    uint8_t u8OldLED;
    int i8Ret;

    u8OldLED = s_kbd_dev.bLED;

    switch(u8Key)
    {
    case KEY_NUM_LOCK:
        s_kbd_dev.bLED = s_kbd_dev.bLED ^ LED_NumLock;
        break;
    case KEY_CAPS_LOCK:
        s_kbd_dev.bLED = s_kbd_dev.bLED ^ LED_CapsLoock;
        break;
    case KEY_SCROLL_LOCK:
        s_kbd_dev.bLED = s_kbd_dev.bLED ^ LED_ScrollLock;
        break;
    }

    if(s_kbd_dev.bLED != u8OldLED)
    {
        i8Ret = usbh_hid_set_report(hdev, 2, 0, &s_kbd_dev.bLED, 1);
        if(i8Ret < 0)
            printf("usbh_hid_set_report failed - %d\n", i8Ret);
        return (uint8_t)i8Ret;
    }

    return 0;
}

int kbd_parse_report(HID_DEV_T *hdev, uint8_t *pu8Buf, int i8Len)
{
    int i8CntI, i8CntJ;
    char chDown, chUp;
    uint8_t u8Key;

    (void)i8Len;

    // On error - return
    if(pu8Buf[2] == 1)
        return -1;

    for(i8CntI = 2; i8CntI < 8; ++i8CntI)
    {
        chDown = chUp = 0;

        for(i8CntJ = 2; i8CntJ < 8; i8CntJ++)
        {
            if((pu8Buf[i8CntI] == s_kbd_dev.pre_data[i8CntJ]) && (pu8Buf[i8CntI] != 1))
                chDown = 1;
            if((pu8Buf[i8CntJ] == s_kbd_dev.pre_data[i8CntI]) && (s_kbd_dev.pre_data[i8CntI] != 1))
                chUp = 1;
        }

        if(!chDown)
        {
            update_locking_keys(hdev, pu8Buf[i8CntI]);
            printf("Pressed: 0x%x ", pu8Buf[i8CntI]);
            u8Key = oem_key_to_ascii(hdev, pu8Buf[0], pu8Buf[i8CntI]);
            print_key(pu8Buf[0], u8Key);
        }

        if(!chUp)
        {
            printf("Released: 0x%x ", pu8Buf[i8CntI]);
            u8Key = oem_key_to_ascii(hdev, pu8Buf[0], pu8Buf[i8CntI]);
            print_key(pu8Buf[0], u8Key);
        }
    }

    for(i8CntI = 0; i8CntI < 8; ++i8CntI)
        s_kbd_dev.pre_data[i8CntI] = pu8Buf[i8CntI];

    return 0;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/