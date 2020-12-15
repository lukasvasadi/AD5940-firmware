#include <Arduino.h>
#include <SPI.h>
// #include <AD5940.h>

// SPI commands
#define SPICMD_SETADDR  (0x20) // Set register address for read/write
#define SPICMD_READREG  (0x6D) // SPI read command
#define SPICMD_WRITEREG (0x2d) // SPI write command
#define SPICMD_READFIFO (0x5F) // Read data from FIFO

// System initialization
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

// Low power TIA
#define LPTIASW0_ADDR (0x000020E4)
#define LPTIASW0_RST  (0x00000000)

const byte chipSelect = 12;
const byte reset = 13;

// AD5940 AD5940_EVAL(chipSelect); // Instantiate AD5940

void writeRegister16(uint16_t reg, uint16_t data)
{
  // Package register address into buffer
  uint8_t regBuffer[2];
  regBuffer[0] = (reg >> 8);
  regBuffer[1] = (reg & 0xFF);

  Serial.print("REG: ");
  Serial.print(regBuffer[0], HEX); Serial.print(" ");
  Serial.println(regBuffer[1], HEX);

  // Package data into buffer
  uint8_t dataBuffer[2];
  dataBuffer[0] = ((data >> 8) & 0xFF);
  dataBuffer[1] = (data & 0xFF);

  Serial.print("16 BIT DATA: ");
  Serial.print(dataBuffer[0], HEX); Serial.print(" ");
  Serial.println(dataBuffer[1], HEX);

  // Begin serial transmission
  SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));

  // Target address
  digitalWrite(chipSelect, LOW);
  delay(1);
  SPI.transfer(SPICMD_SETADDR);
  SPI.transfer(regBuffer, 2);
  digitalWrite(chipSelect, HIGH);

  // Write data
  digitalWrite(chipSelect, LOW);
  delay(1);
  SPI.transfer(SPICMD_WRITEREG);
  SPI.transfer(dataBuffer, 2);
  digitalWrite(chipSelect, HIGH);

  SPI.endTransaction();
}

void writeRegister32(uint16_t reg, uint32_t data)
{
  // Package register address into buffer
  uint8_t regBuffer[2];
  regBuffer[0] = (reg >> 8);
  regBuffer[1] = (reg & 0xFF);

  Serial.print("REG: ");
  Serial.print(regBuffer[0], BIN); Serial.print(" ");
  Serial.println(regBuffer[1], BIN);

  // Package data into buffer
  uint8_t dataBuffer[4];
  dataBuffer[0] = (data >> 24);
  dataBuffer[1] = ((data >> 16) & 0xFF);
  dataBuffer[2] = ((data >> 8) & 0xFF);
  dataBuffer[3] = (data & 0xFF);

  Serial.print("DATA: ");
  Serial.print(dataBuffer[0], BIN); Serial.print(" ");
  Serial.print(dataBuffer[1], BIN); Serial.print(" ");
  Serial.print(dataBuffer[2], BIN); Serial.print(" ");
  Serial.println(dataBuffer[3], BIN);

  // Begin serial transmission
  SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));

  // Target address
  digitalWrite(chipSelect, LOW);
  delay(1);
  SPI.transfer(SPICMD_SETADDR);
  SPI.transfer(regBuffer, 2);
  digitalWrite(chipSelect, HIGH);

  // Write data
  digitalWrite(chipSelect, LOW);
  delay(1);
  SPI.transfer(SPICMD_WRITEREG);
  SPI.transfer(dataBuffer, 4);
  digitalWrite(chipSelect, HIGH);

  SPI.endTransaction();
}

uint16_t readRegister16(uint16_t reg)
{
  // Initialize message buffer
  uint8_t msg[2];

  // Package register address into buffer
  uint8_t regBuffer[2];
  regBuffer[0] = (reg >> 8);
  regBuffer[1] = (reg & 0xFF);
  
  // Begin serial transmission
  SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));

  // Target address
  digitalWrite(chipSelect, LOW);
  delay(1);
  SPI.transfer(SPICMD_SETADDR);
  SPI.transfer(regBuffer, 2);
  digitalWrite(chipSelect, HIGH);

  // Read from address
  digitalWrite(chipSelect, LOW);
  delay(1);
  SPI.transfer(SPICMD_READREG);
  for (int i = 0; i < 2; i++)
  {
    msg[i] = SPI.transfer(0);
  }
  digitalWrite(chipSelect, HIGH);

  SPI.endTransaction();

  uint16_t msg16B = ((msg[0] << 8) | msg[1]);

  return msg16B;
}

uint32_t readRegister32(uint16_t reg)
{
  // Initialize message buffer
  uint8_t msg[4];

  // Package register address into buffer
  uint8_t regBuffer[2];
  regBuffer[0] = (reg >> 8);
  regBuffer[1] = (reg & 0xFF);
  
  // Begin serial transmission
  SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));

  // Target address
  digitalWrite(chipSelect, LOW);
  delay(1);
  SPI.transfer(SPICMD_SETADDR);
  SPI.transfer(regBuffer, 2);
  digitalWrite(chipSelect, HIGH);

  // Read data
  digitalWrite(chipSelect, LOW);
  delay(1);
  SPI.transfer(SPICMD_READREG);
  msg[0] = SPI.transfer(0);
  // msg[1] = SPI.transfer(0);
  // msg[2] = SPI.transfer(0);
  // msg[3] = SPI.transfer(0);
  digitalWrite(chipSelect, HIGH);

  SPI.endTransaction();

  // uint32_t msg32B = ((msg[0] << 24) | (msg[1] << 16) | (msg[2] << 8) | msg[3]);

  return msg[0];
}

void setup()
{
  Serial.begin(9600);
  // Wait for serial connection setup
  while (!Serial) ;

  SPI.begin();
  // SPI.setClockDivider(SPI_CLOCK_DIV8);

  pinMode(chipSelect, OUTPUT);
  pinMode(reset, OUTPUT);

  digitalWrite(chipSelect, HIGH);

  // Toggle pin to reset chip
  digitalWrite(reset, HIGH);
  delay(1);
  digitalWrite(reset, LOW);
  delay(1);
  digitalWrite(reset, HIGH);
  Serial.println("Reset complete");

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
  // writeRegister32(LPTIASW0_ADDR, LPTIASW0_RST);
  // writeRegister32(LPTIASW0_ADDR, 0x1);
  // Serial.print("Write val: ");
  // // Serial.println(0x01161D6E, BIN);
  Serial.println(readRegister16(AD5940_REG_INIT01), HEX);

  Serial.println("Hello world");

  delay(2000);
}