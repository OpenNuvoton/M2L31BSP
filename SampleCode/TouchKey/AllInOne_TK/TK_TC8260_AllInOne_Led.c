#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"
#include "tklib.h"

#define OPT_USE_LED

#ifdef OPT_USE_LED
typedef struct tagLEG
{
    //unsigned char u8Port;
    unsigned char chan;
    GPIO_T*  pu8Port;
    uint32_t u32Msk;
} LED_T;


LED_T LEDInfo[TKLIB_TOL_NUM_KEY] =
{
    {0x00, PE, BIT11},  //TK0
    {0x01, PE, BIT10},  //TK1
    {0x02, PE, BIT9},   //TK2
    {0x03, PE, BIT8},   //TK3
    {0xFF, PE, BIT7},   //TK4 is invalid
    {0x05, PG, BIT4},   //TK5
    {0x06, PG, BIT3},   //TK6
    {0x07, PG, BIT2},   //TK7
    {0x08, PD, BIT10},  //TK8
    {0xFF, PG, BIT4},   //TK9
    {0xFF, PG, BIT3},   //TK10
    {0xFF, PG, BIT2},   //TK11
    {0xFF, PD, BIT10},  //TK12
    {0xFF, PG, BIT4},   //TK13
    {0xFF, PG, BIT3},   //TK14
    {0xFF, PG, BIT2},   //TK15
    {0xFF, PD, BIT10},  //TK16
    {0xFF, PG, BIT4},   //TK17

};
/**
  * @brief      Light the led(s)
  * @param[in]  onOff    0: Touch Key LED turn off
  *                      1: Touch Key LED turn on
  * @param[in]  chanN    the channel number to be configured. (TK1 = 1, TK2 = 2, TK3 = 3, TK4 = 4)
  * @retval     None.
  * @details    This function is used to light the led(s)
  */

void TK_lightLED(unsigned char onOff, int chanN)
{
    if(LEDInfo[chanN].chan != 0xFF)
    {
        if(onOff == 1)      //On
        {
            LEDInfo[chanN].pu8Port->DOUT &= ~(LEDInfo[chanN].u32Msk);
        }
        else
        {
            LEDInfo[chanN].pu8Port->DOUT |= LEDInfo[chanN].u32Msk;
        }
    }
}

void TK_LEDAllOff(void)
{
    uint32_t i;
    for(i=0; i<TKLIB_TOL_NUM_KEY; i=i+1)
    {
        if(LEDInfo[i].chan != 0xFF)
        {
            TK_lightLED(FALSE, i);
        }
    }
}

/**
  * @brief      Initialize touch key LED output state
  * @param      None.
  * @retval     None.
  * @details    This function is used to initialize touch key LED output state. Default MFP is GPIO
  *             Foe code size, not to set MFP regster.
  */
void InitLEDIO(void)
{

    uint32_t i;
    for(i=0; i<TKLIB_TOL_NUM_KEY; i=i+1)
    {
        if(LEDInfo[i].chan != 0xFF)
            GPIO_SetMode(LEDInfo[i].pu8Port, LEDInfo[i].u32Msk, GPIO_MODE_QUASI);
    }
}

#endif
