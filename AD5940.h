/**************************************************************************/
/*!
    @file     AD5940.h
    This is a library for the Analog Devices AD5940.
    Written by Lukas Vasadi for Hexagonfab Ltd.
    All text here must be included in any redistribution
*/
/**************************************************************************/

#ifndef __AD5940_H__
#define __AD5940_H__

#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <SPI.h>

/*=========================================================================
    SPI COMMANDS
    -----------------------------------------------------------------------*/
#define SPICMD_SETADDR (0x20)
#define SPICMD_READREG (0x6D)
#define SPICMD_WRITEREG (0x2d)
#define SPICMD_READFIFO (0x5F)
/*=========================================================================*/

/*=========================================================================
    SYSTEM INITIALIZATION
    -----------------------------------------------------------------------*/
#define AD5940_REG_INIT01 (0x0908) ///< 12 bit address
#define AD5940_REG_INIT02 (0x0C08)
#define AD5940_REG_INIT03 (0x21F0)
#define AD5940_REG_INIT04 (0x0410)
#define AD5940_REG_INIT05 (0x0A28)
#define AD5940_REG_INIT06 (0x238C)
#define AD5940_REG_INIT07 (0x0A04)
#define AD5940_REG_INIT08 (0x0A04)
#define AD5940_REG_INIT09 (0x0A00)
#define AD5940_REG_INIT10 (0x22F0)

#define AD5940_DATA_INIT01 (0x02C9) ///< 12 bit data
#define AD5940_DATA_INIT02 (0x206C)
#define AD5940_DATA_INIT03 (0x0010)
#define AD5940_DATA_INIT04 (0x02C9)
#define AD5940_DATA_INIT05 (0x0009)
#define AD5940_DATA_INIT06 (0x0104)
#define AD5940_DATA_INIT07 (0x4859)
#define AD5940_DATA_INIT08 (0xF27B)
#define AD5940_DATA_INIT09 (0x8009)
#define AD5940_DATA_INIT10 (0x0000)
/*=========================================================================*/

/*=========================================================================
    ADC CONFIGURATION
    -----------------------------------------------------------------------*/
#define AD5940_REG_CONFIG_PGA_4 (0x0908)
#define AD5940_REG_CONFIG_PGA_3 (0x0908)
#define AD5940_REG_CONFIG_PGA_2 (0x0908)
#define AD5940_REG_CONFIG_PGA_1 (0x0908)
#define AD5940_REG_CONFIG_PGA_0 (0x0908)
/*=========================================================================*/

/** Gain settings */
typedef enum
{
    GAIN_ONE = AD5940_REG_CONFIG_PGA_4,
    GAIN_THREE_HAVLES = AD5940_REG_CONFIG_PGA_3,
    GAIN_TWO = AD5940_REG_CONFIG_PGA_2,
    GAIN_FOUR = AD5940_REG_CONFIG_PGA_1,
    GAIN_NINE = AD5940_REG_CONFIG_PGA_0
} adcGain_t;

/**************************************************************************/
/*!
    @brief  Driver for the AD5940.
*/
/**************************************************************************/
class AD5940
{
protected:
    // Instance-specific properties
    adcGain_t adcGain; ///< ADC gain
    byte cs;

public:
    AD5940(byte chipSelect);
    void begin(void);
    void init(void);
    uint16_t readADC(void);
    adcGain_t getGain(void);
    uint16_t readOutput(void);

private:
};

#endif
