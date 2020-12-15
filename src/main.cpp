#include <Arduino.h>
#include <SPI.h>
// #include <AD5940.h>

// SPI commands
#define SPICMD_SETADDR  0x20 // Set register address for read/write
#define SPICMD_READREG  0x6D // SPI read command
#define SPICMD_WRITEREG 0x2D // SPI write command
#define SPICMD_READFIFO 0x5F // Read data from FIFO

// System initialization
#define AD5940_REG_INIT01 0x0908 ///< 12 bit address
#define AD5940_REG_INIT02 0x0C08
#define AD5940_REG_INIT03 0x21F0
#define AD5940_REG_INIT04 0x0410
#define AD5940_REG_INIT05 0x0A28
#define AD5940_REG_INIT06 0x238C
#define AD5940_REG_INIT07 0x0A04
#define AD5940_REG_INIT08 0x0A04
#define AD5940_REG_INIT09 0x0A00
#define AD5940_REG_INIT10 0x22F0

#define AD5940_DATA_INIT01 0x02C9 ///< 12 bit data
#define AD5940_DATA_INIT02 0x206C
#define AD5940_DATA_INIT03 0x0010
#define AD5940_DATA_INIT04 0x02C9
#define AD5940_DATA_INIT05 0x0009
#define AD5940_DATA_INIT06 0x0104
#define AD5940_DATA_INIT07 0x4859
#define AD5940_DATA_INIT08 0xF27B
#define AD5940_DATA_INIT09 0x8009
#define AD5940_DATA_INIT10 0x0000

// Low power TIA
#define LPTIASW0_ADDR 0x000020E4
#define LPTIASW0_RST  0x00000000

// Pin definitions
const byte chipSelect = 12;
const byte reset = 13;

// AD5940 AD5940_EVAL(chipSelect); // Instantiate AD5940

void writeRegister16(uint16_t reg, uint16_t data)
{
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));

  // Target address
  digitalWrite(chipSelect, LOW);
  SPI.transfer(SPICMD_SETADDR);
  SPI.transfer16(reg);
  digitalWrite(chipSelect, HIGH);

  delay(1);

  // Write data
  digitalWrite(chipSelect, LOW);
  SPI.transfer(SPICMD_WRITEREG);
  SPI.transfer16(data);
  digitalWrite(chipSelect, HIGH);

  delay(1);

  SPI.endTransaction();
}

void writeRegister32(uint16_t reg, uint32_t data)
{
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));

  // Target address
  digitalWrite(chipSelect, LOW);
  SPI.transfer(SPICMD_SETADDR);
  SPI.transfer16(reg);
  digitalWrite(chipSelect, HIGH);

  delay(1);

  // Write data
  digitalWrite(chipSelect, LOW);
  SPI.transfer(SPICMD_WRITEREG);
  SPI.transfer16(data >> 16);
  SPI.transfer16(data & 0xFF);
  digitalWrite(chipSelect, HIGH);

  delay(1);

  SPI.endTransaction();
}

uint16_t readRegister16(uint16_t reg)
{
  uint16_t returnMsg;

  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));

  // Target address
  digitalWrite(chipSelect, LOW);
  SPI.transfer(SPICMD_SETADDR);
  SPI.transfer16(reg);
  digitalWrite(chipSelect, HIGH);

  delay(1);

  // Read from address
  digitalWrite(chipSelect, LOW);
  SPI.transfer(SPICMD_READREG);
  returnMsg = SPI.transfer16(0);
  digitalWrite(chipSelect, HIGH);

  delay(1);

  SPI.endTransaction();

  return returnMsg;
}

uint32_t readRegister32(uint16_t reg)
{
  uint32_t returnMsg[2];

  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));

  // Target address
  digitalWrite(chipSelect, LOW);
  SPI.transfer(SPICMD_SETADDR);
  SPI.transfer16(reg);
  digitalWrite(chipSelect, HIGH);

  delay(1);

  // Read data
  digitalWrite(chipSelect, LOW);
  SPI.transfer(SPICMD_READREG);
  returnMsg[0] = SPI.transfer16(0);
  returnMsg[1] = SPI.transfer16(0);
  digitalWrite(chipSelect, HIGH);

  delay(1);

  SPI.endTransaction();

  return ((returnMsg[0] << 16) | (returnMsg[1] & 0xFF));
}

void setup()
{
  Serial.begin(115200);
  // Wait for serial connection setup
  while (!Serial) ;

  SPI.begin();
  // SPI.setClockDivider(SPI_CLOCK_DIV8);

  pinMode(chipSelect, OUTPUT);
  pinMode(reset, OUTPUT);

  digitalWrite(chipSelect, HIGH);

  // Toggle pin to reset chip
  digitalWrite(reset, HIGH);
  // delay(1);
  digitalWrite(reset, LOW);
  // delay(1);
  digitalWrite(reset, HIGH);
  Serial.println("Reset complete");

  delay(10);

  // AD5940_EVAL.begin();
  // AD5940_EVAL.init();
  writeRegister16(AD5940_REG_INIT01, AD5940_DATA_INIT01);
  writeRegister16(AD5940_REG_INIT02, AD5940_DATA_INIT02);
  writeRegister16(AD5940_REG_INIT03, AD5940_DATA_INIT03);
  writeRegister16(AD5940_REG_INIT04, AD5940_DATA_INIT04);
  writeRegister16(AD5940_REG_INIT05, AD5940_DATA_INIT05);
  writeRegister16(AD5940_REG_INIT06, AD5940_DATA_INIT06);
  writeRegister16(AD5940_REG_INIT07, AD5940_DATA_INIT07);
  writeRegister16(AD5940_REG_INIT08, AD5940_DATA_INIT08);
  writeRegister16(AD5940_REG_INIT09, AD5940_DATA_INIT09);
  writeRegister16(AD5940_REG_INIT10, AD5940_DATA_INIT10);
}

void loop() 
{
  // writeRegister32(0x00000040, 0x00000000);
  Serial.println(readRegister16(AD5940_REG_INIT08), HEX);

  // writeRegister32(0x0000215C, 0x00000001);
  // Serial.println(readRegister32(0x0000215C), HEX);
  
  Serial.println();
  delay(2000);
}
