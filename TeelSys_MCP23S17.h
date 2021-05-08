/*!
 * @file TeelSys_MCP23S17.h
 */

#ifndef _TeelSys_MCP23S17_H_
#define _TeelSys_MCP23S17_H_

#include <SPI.h>

/*!
 * @brief MCP23S17 main class
 */
class TeelSys_MCP23S17 {
public:
  void begin(uint8_t address, uint8_t ss);
  void begin(uint8_t ss);

  void pinMode(uint8_t p, uint8_t d);
  void portMode(uint8_t b, uint8_t d);
  void portPolarity(uint8_t b, uint8_t d);
  void digitalWrite(uint8_t p, uint8_t d);
  void pullUp(uint8_t p, uint8_t d);
  uint8_t digitalRead(uint8_t p);

  void writeGPIOAB(uint16_t);
  void writeGPIO(uint8_t b, uint8_t d);
  uint16_t readGPIOAB();
  uint8_t readGPIO(uint8_t b);

  void setupInterrupts(uint8_t mirroring, uint8_t open, uint8_t polarity);
  void setupInterruptPin(uint8_t pin, uint8_t mode);
  void disableInterruptPin(uint8_t pin);
  uint8_t getLastInterruptPin();
  uint8_t getLastInterruptPinValue();

private:
  uint8_t _address;
  uint8_t _ss;
  bool _ss_default;

  uint8_t bitForPin(uint8_t pin);
  uint8_t regForPin(uint8_t pin, uint8_t portAaddr, uint8_t portBaddr);

  uint8_t readRegister(uint8_t addr);
  void writeRegister(uint8_t addr, uint8_t value);

  /**
   * Utility private method to update a register associated with a pin (whether
   * port A/B) reads its value, updates the particular bit, and writes its
   * value.
   */
  void updateRegisterBit(uint8_t p, uint8_t pValue, uint8_t portAaddr,
                         uint8_t portBaddr);
};

//#define MCP23017_ADDRESS 0x20 //!< MCP23S17 Address

// ports
#define MCP23017_PORT_A 0x00 //!< Port A used for port methods
#define MCP23017_PORT_B 0x01 //!< Port B used for port methods

// registers
#define MCP23017_IODIRA 0x00   //!< I/O direction register A
#define MCP23017_IPOLA 0x02    //!< Input polarity port register A
#define MCP23017_GPINTENA 0x04 //!< Interrupt-on-change pins A
#define MCP23017_DEFVALA 0x06  //!< Default value register A
#define MCP23017_INTCONA 0x08  //!< Interrupt-on-change control register A
#define MCP23017_IOCONA 0x0A   //!< I/O expander configuration register A
#define MCP23017_GPPUA 0x0C    //!< GPIO pull-up resistor register A
#define MCP23017_INTFA 0x0E    //!< Interrupt flag register A
#define MCP23017_INTCAPA 0x10  //!< Interrupt captured value for port register A
#define MCP23017_GPIOA 0x12    //!< General purpose I/O port register A
#define MCP23017_OLATA 0x14    //!< Output latch register 0 A

#define MCP23017_IODIRB 0x01   //!< I/O direction register B
#define MCP23017_IPOLB 0x03    //!< Input polarity port register B
#define MCP23017_GPINTENB 0x05 //!< Interrupt-on-change pins B
#define MCP23017_DEFVALB 0x07  //!< Default value register B
#define MCP23017_INTCONB 0x09  //!< Interrupt-on-change control register B
#define MCP23017_IOCONB 0x0B   //!< I/O expander configuration register B
#define MCP23017_GPPUB 0x0D    //!< GPIO pull-up resistor register B
#define MCP23017_INTFB 0x0F    //!< Interrupt flag register B
#define MCP23017_INTCAPB 0x11  //!< Interrupt captured value for port register B
#define MCP23017_GPIOB 0x13    //!< General purpose I/O port register B
#define MCP23017_OLATB 0x15    //!< Output latch register 0 B

#define MCP23017_INT_ERR 255 //!< Interrupt error

// Control byte and configuration register information - Control Byte: "0100 A2 A1 A0 R/W" -- W=0
#define    OPCODEW       (0b01000000)  //!< Opcode for MCP23S17 with LSB (bit0) set to write (0), address OR'd in later, bits 1-3
#define    OPCODER       (0b01000001)  //!< Opcode for MCP23S17 with LSB (bit0) set to read (1), address OR'd in later, bits 1-3
#define    ADDR_ENABLE   (0b00001000)  //!< Configuration register for MCP23S17, the only thing we change is enabling hardware addressing

#endif