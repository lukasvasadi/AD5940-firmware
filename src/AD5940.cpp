/**************************************************************************/
/*!
    @file     AD5940.cpp
    @author   LUKAS J. VASADI

    @section INTRO

    This is a library for the AD5940 analog front end.

    @section  HISTORY

    v1.0  - First release

    @section LICENSE

    All text here must be included in any redistribution
*/
/**************************************************************************/

#if ARDUINO >= 100
#include <Arduino.h>
#endif

#include <SPI.h>
#include <AD5940.h>

/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register

    @param cs chip select pin to target device
    @param reg register address
    @param data value to write to register
*/
/**************************************************************************/

static void writeRegister(byte cs, uint16_t reg, uint32_t data)
{
    SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));

    digitalWrite(cs, LOW);
    SPI.transfer(SPICMD_SETADDR);
    SPI.transfer16(reg); // Target register memory location
    digitalWrite(cs, HIGH);

    digitalWrite(cs, LOW);
    SPI.transfer(SPICMD_WRITEREG);
    SPI.transfer16(data >> 16); // Write data to register
    SPI.transfer16(data & 0xFF);
    digitalWrite(cs, HIGH);

    digitalWrite(cs, LOW);
    SPI.transfer(SPICMD_READREG);
    uint16_t msg = SPI.transfer16(0); // Read from register
    digitalWrite(cs, HIGH);

    SPI.endTransaction();

    Serial.print(reg, HEX);
    Serial.print(", ");
    Serial.print((data >> 16), BIN);
    Serial.print(", ");
    Serial.print((data & 0xFF), BIN);
    Serial.print(", ");
    Serial.println(msg, HEX);
}

/**************************************************************************/
/*!
    @brief  Read 8-bits from the specified destination register

    @param cs chip select pin to target device
    @param reg register address

    @return 16 bit register value read
*/
/**************************************************************************/
static uint16_t readRegister(byte cs, uint16_t reg)
{
    SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));

    digitalWrite(cs, LOW);
    SPI.transfer(SPICMD_SETADDR);
    SPI.transfer16(reg);
    digitalWrite(cs, HIGH);

    digitalWrite(cs, LOW);
    SPI.transfer(SPICMD_READREG);
    uint16_t msg = SPI.transfer16(0); // Read from register
    digitalWrite(cs, HIGH);

    SPI.endTransaction();

    return msg;
}

/**************************************************************************/
/*!
    @brief  Instantiates a new AD5940 class w/appropriate properties

    @param i2cAddress I2C address of device
*/
/**************************************************************************/
AD5940::AD5940(byte chipSelect)
{
    adcGain = GAIN_ONE;
    cs = chipSelect;
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);
}

/**************************************************************************/
/*!
    @brief  Sets up the serial communication
*/
/**************************************************************************/
void AD5940::begin() { SPI.begin(); }

/**************************************************************************/
/*!
    @brief  Initialize the analog front end (AFE)
*/
/**************************************************************************/
void AD5940::init()
{ 
    writeRegister(cs, AD5940_REG_INIT01, AD5940_DATA_INIT01);
    writeRegister(cs, AD5940_REG_INIT02, AD5940_DATA_INIT02);
    writeRegister(cs, AD5940_REG_INIT03, AD5940_DATA_INIT03);
    writeRegister(cs, AD5940_REG_INIT04, AD5940_DATA_INIT04);
    writeRegister(cs, AD5940_REG_INIT05, AD5940_DATA_INIT05);
    writeRegister(cs, AD5940_REG_INIT06, AD5940_DATA_INIT06);
    writeRegister(cs, AD5940_REG_INIT07, AD5940_DATA_INIT07);
    writeRegister(cs, AD5940_REG_INIT08, AD5940_DATA_INIT08);
    writeRegister(cs, AD5940_REG_INIT09, AD5940_DATA_INIT09);
    writeRegister(cs, AD5940_REG_INIT10, AD5940_DATA_INIT10);
}

/**************************************************************************/
/*!
    @brief  Gets a gain and input voltage range

    @return Gain setting
*/
/**************************************************************************/
adcGain_t AD5940::getGain() { return adcGain; }

/**************************************************************************/
/*!
    @brief  Gets a single-ended ADC reading from the specified channel

    @param channel ADC channel to read

    @return the ADC reading
*/
/**************************************************************************/
uint16_t AD5940::readADC()
{
    return 1;
}

/**************************************************************************/
/*!
    @brief  Gets a single-ended ADC reading from the specified channel

    @param channel ADC channel to read

    @return the ADC reading
*/
/**************************************************************************/
uint16_t AD5940::readOutput()
{
    writeRegister(cs, 0x00002000, 0x00080000);
    writeRegister(cs, 0x00002000, 0x00000005);
    return readRegister(cs, 0x00000400);
}
