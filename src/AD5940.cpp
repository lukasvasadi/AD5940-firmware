/**************************************************************************/
/*!
    @file     AD5940.cpp
    @author   LUKAS J. VASADI

    @section INTRO

    This is a library for the AD5940 analog front end.

    @section  HISTORY

    v1.0  - First release
    v1.1  - Added ADS1115 support - W. Earl

    @section LICENSE

    All text here must be included in any redistribution
*/
/**************************************************************************/

#if ARDUINO >= 100
#include <Arduino.h>
#endif

#include <SPI.h>
#include <AD5940.h>
