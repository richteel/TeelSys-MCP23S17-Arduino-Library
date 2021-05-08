/*!
 * @file TeelSys_MCP23S17.cpp
 *
 * @mainpage TeelSys MCP23S17 Library
 *
 * @section intro_sec Introduction
 *
 * This is a library for the MCP23S17 i2c port expander
 *
 * These displays use I2C to communicate, 2 pins are required to
 * interface
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Richard Teel as a drop-in replacement of the Adafruit_MCP23017 
 * library written by Limor Fried/Ladyada  for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, check license.txt for more information
 * All text above must be included in any redistribution with the exception of 
 * Adafruit Industries. Adafruit Industries may modify this file and code as 
 * they see fit as the base code used was written and developed by Adafruit.
 */

#ifdef __AVR
#include <avr/pgmspace.h>
#elif defined(ESP8266)
#include <pgmspace.h>
#endif
#include "TeelSys_MCP23S17.h"

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


/**
 * Bit number associated to a give Pin
 */
uint8_t TeelSys_MCP23S17::bitForPin(uint8_t pin) { return pin % 8; }

/**
 * Register address, port dependent, for a given PIN
 */
uint8_t TeelSys_MCP23S17::regForPin(uint8_t pin, uint8_t portAaddr,
                                     uint8_t portBaddr) {
  return (pin < 8) ? portAaddr : portBaddr;
}

/**
 * Reads a given register
 */
uint8_t TeelSys_MCP23S17::readRegister(uint8_t regAddr) {
  // read the current GPINTEN
	uint8_t value = 0;												// Initialize a variable to hold the read values to be returned
  ::digitalWrite(_ss, LOW);									// Take slave-select low
  SPI.transfer(OPCODER | (_address << 1));	// Send the MCP23S17 opcode, chip address, and read bit
  SPI.transfer(regAddr);										// Send the register we want to read
  value = SPI.transfer(0x00);								// Send any byte, the function will return the read value
  ::digitalWrite(_ss, HIGH);								// Take slave-select high
  return value;															// Return the constructed word, the format is 0x(register value)
}

/**
 * Writes a given register
 */
void TeelSys_MCP23S17::writeRegister(uint8_t regAddr, uint8_t regValue) {
  // Write the register
  ::digitalWrite(_ss, LOW);									// Take slave-select low
  SPI.transfer(OPCODEW | (_address << 1));	// Send the MCP23S17 opcode, chip address, and write bit
  SPI.transfer(regAddr);										// Send the register we want to write
  SPI.transfer(regValue);										// Send the byte
  ::digitalWrite(_ss, HIGH);								// Take slave-select high
}

/**
 * Helper to update a single bit of an A/B register.
 * - Reads the current register value
 * - Writes the new register value
 */
void TeelSys_MCP23S17::updateRegisterBit(uint8_t pin, uint8_t pValue,
                                          uint8_t portAaddr,
                                          uint8_t portBaddr) {
  uint8_t regValue;
  uint8_t regAddr = regForPin(pin, portAaddr, portBaddr);
  uint8_t bit = bitForPin(pin);
  regValue = readRegister(regAddr);

  // set the value for the particular bit
  bitWrite(regValue, bit, pValue);

  writeRegister(regAddr, regValue);
}

////////////////////////////////////////////////////////////////////////////////


/*!
 * Initializes the MCP23S17 given its HW selected address, see datasheet for
 * Address selection.
 * @param address Selected address (0 to 7)
 * @ss the slave select pin
 */
void TeelSys_MCP23S17::begin(uint8_t address, uint8_t ss) {
  if (address > 7) {
    address = 7;
  }
  _address = address;
  _ss = ss;
  
  _ss_default = false;
  
#ifdef SS
	if(_ss == SS) {	// SS = PIN_SPI_SS
		_ss_default = true;
	}
#endif

	if(!_ss_default){
		::pinMode(_ss, OUTPUT);
		::digitalWrite(_ss, HIGH);
	}
	
	/*
		Default SPI Mode 			= SPI_MODE0
		Clock Polarity (CPOL)	= 0
		Clock Phase (CPHA) 		= 0
		Output Edge 					= Falling
		Data Capture 					= Rising
	*/
	SPI.begin();

  // set defaults!
  // all inputs on port A and B
  writeRegister(MCP23017_IODIRA, 0xff);
  writeRegister(MCP23017_IODIRB, 0xff);

  // Turn off interrupt triggers
  writeRegister(MCP23017_GPINTENA, 0x00);
  writeRegister(MCP23017_GPINTENB, 0x00);

  // Turn off pull up resistors
  writeRegister(MCP23017_GPPUA, 0x00);
  writeRegister(MCP23017_GPPUB, 0x00);
}

/**
 * Initializes the default MCP23017, with 000 for the configurable part of the
 * address
 * @param theWire the I2C object to use, defaults to &Wire
 */
void TeelSys_MCP23S17::begin(uint8_t ss) { begin(0, ss); }

/**
 * Sets the pin mode to either INPUT or OUTPUT
 * @param p Pin to set
 * @param d Mode to set the pin
 */
void TeelSys_MCP23S17::pinMode(uint8_t p, uint8_t d) {
  updateRegisterBit(p, (d == INPUT || d == INPUT_PULLUP), MCP23017_IODIRA,
                    MCP23017_IODIRB);
  if (d == INPUT || d == INPUT_PULLUP) {
    updateRegisterBit(p, (d == INPUT_PULLUP), MCP23017_GPPUA, MCP23017_GPPUB);
  }
}

/**
 * Sets the port mode to either INPUT or OUTPUT
 * @param b Decide which gpio to use. Should be 0 for GPIOA, and 1 for GPIOB.
 * @param d Mode to set the port
 */
void TeelSys_MCP23S17::portMode(uint8_t b, uint8_t d) {
  uint8_t addrIODIR = MCP23017_IODIRA;
  uint8_t addrGPPU = MCP23017_GPPUA;

  if (b == MCP23017_PORT_B) {
    addrIODIR = MCP23017_IODIRB;
    addrGPPU = MCP23017_GPPUB;
  }

  switch (d) {
  case INPUT:
    writeRegister(addrIODIR, 0xff);
    writeRegister(addrGPPU, 0x00);
    break;
  case INPUT_PULLUP:
    writeRegister(addrIODIR, 0xff);
    writeRegister(addrGPPU, 0xff);
    break;
  case OUTPUT:
    writeRegister(addrIODIR, 0x00);
    break;
  }
}

/**
 * Sets the port mode to either non-inverted or inverted
 * @param b Decide which gpio to use. Should be 0 for GPIOA, and 1 for GPIOB.
 * @param d Polarity 0 = non-inverted, 1 = inverted
 */
void TeelSys_MCP23S17::portPolarity(uint8_t b, uint8_t d) {
  uint8_t addrIPOL = MCP23017_IPOLA;

  if (b == MCP23017_PORT_B) {
    addrIPOL = MCP23017_IPOLB;
  }

  if (d == 0) { // non-inverted
    writeRegister(addrIPOL, 0x00);
  } else { // inverted
    writeRegister(addrIPOL, 0xff);
  }
}

/**
 * Reads all 16 pins (port A and B) into a single 16 bits variable.
 * @return Returns the 16 bit variable representing all 16 pins
 */
uint16_t TeelSys_MCP23S17::readGPIOAB() {
  uint16_t ba = 0;
  uint8_t a;

  // read the current GPIO output latches
  ::digitalWrite(_ss, LOW);									// Take slave-select low
  SPI.transfer(OPCODER | (_address << 1));	// Send the MCP23S17 opcode, chip address, and read bit
  SPI.transfer(MCP23017_GPIOA);							// Send the register we want to read
  a = SPI.transfer(0x00);										// Send any byte, the function will return the read value (register address pointer will auto-increment after write)
  ba |= (SPI.transfer(0x00) << 8);					// Read in the "high byte" (portB)
  ::digitalWrite(_ss, HIGH);								// Take slave-select high
  
  ba <<= 8;
  ba |= a;

  return ba;
}

/**
 * Read a single port, A or B, and return its current 8 bit value.
 * @param b Decided what gpio to use. Should be 0 for GPIOA, and 1 for GPIOB.
 * @return Returns the b bit value of the port
 */
uint8_t TeelSys_MCP23S17::readGPIO(uint8_t b) {
	uint16_t value = 0;
  // read the current GPIO output latches
  ::digitalWrite(_ss, LOW);									// Take slave-select low
  SPI.transfer(OPCODER | (_address << 1));  // Send the MCP23S17 opcode, chip address, and read bit
  // Send the register we want to read
  if (b == 0)
    SPI.transfer(MCP23017_GPIOA);
  else {
    SPI.transfer(MCP23017_GPIOB);
  }
  value = SPI.transfer(0x00);               // Send any byte, the function will return the read value
  ::digitalWrite(_ss, HIGH);								// Take slave-select high
  return value;
}

/**
 * Writes all the pins in one go. This method is very useful if you are
 * implementing a multiplexed matrix and want to get a decent refresh rate.
 */
void TeelSys_MCP23S17::writeGPIOAB(uint16_t ba) {
	::digitalWrite(_ss, LOW);									// Take slave-select low
  SPI.transfer(OPCODEW | (_address << 1));  // Send the MCP23S17 opcode, chip address, and write bit
  SPI.transfer(MCP23017_GPIOA);             // Send the register we want to write 
  SPI.transfer((uint8_t) (ba & 0xFF));      // Send the low byte (register address pointer will auto-increment after write)
  SPI.transfer((uint8_t) (ba >> 8));        // Shift the high byte down to the low byte location and send
  ::digitalWrite(_ss, HIGH);								// Take slave-select high
}

/**
 * Write a single port, A or B.
 * @param b Decide which gpio to use. Should be 0 for GPIOA, and 1 for GPIOB.
 * @param d byte of data to send
 */
void TeelSys_MCP23S17::writeGPIO(uint8_t b, uint8_t d) {
	::digitalWrite(_ss, LOW);									// Take slave-select low
  SPI.transfer(OPCODEW | (_address << 1));  // Send the MCP23S17 opcode, chip address, and write bit
  // Send the register we want to write
  if (b == 0)
    SPI.transfer(MCP23017_GPIOA);
  else {
    SPI.transfer(MCP23017_GPIOB);
  }
  SPI.transfer(d);											// Send the byte
  ::digitalWrite(_ss, HIGH);								// Take slave-select high
}

/*!
 * @brief Writes to a pin on the MCP23S17
 * @param pin Pin to write to
 * @param d What to write to the pin
 */
void TeelSys_MCP23S17::digitalWrite(uint8_t pin, uint8_t d) {
  uint8_t gpio;
  uint8_t bit = bitForPin(pin);

  // read the current GPIO output latches
  uint8_t regAddr = regForPin(pin, MCP23017_OLATA, MCP23017_OLATB);
  gpio = readRegister(regAddr);

  // set the pin and direction
  bitWrite(gpio, bit, d);

  // write the new GPIO
  regAddr = regForPin(pin, MCP23017_GPIOA, MCP23017_GPIOB);
  writeRegister(regAddr, gpio);
}

/*!
 * @brief Enables the pull-up resistor on the specified pin
 * @param p Pin to set
 * @param d Value to set the pin
 */
void TeelSys_MCP23S17::pullUp(uint8_t p, uint8_t d) {
  updateRegisterBit(p, d, MCP23017_GPPUA, MCP23017_GPPUB);
}

/*!
 * @brief Reads the specified pin
 * @param pin Pin to read
 * @return Value of the pin
 */
uint8_t TeelSys_MCP23S17::digitalRead(uint8_t pin) {
  uint8_t bit = bitForPin(pin);
  uint8_t regAddr = regForPin(pin, MCP23017_GPIOA, MCP23017_GPIOB);
  return (readRegister(regAddr) >> bit) & 0x1;
}

/**
 * Configures the interrupt system. both port A and B are assigned the same
 * configuration.
 * @param mirroring Mirroring will OR both INTA and INTB pins.
 * @param openDrain Opendrain will set the INT pin to value or open drain.
 * @param polarity polarity will set LOW or HIGH on interrupt.
 * Default values after Power On Reset are: (false, false, LOW)
 * If you are connecting the INTA/B pin to arduino 2/3, you should configure the
 * interupt handling as FALLING with the default configuration.
 */
void TeelSys_MCP23S17::setupInterrupts(uint8_t mirroring, uint8_t openDrain,
                                        uint8_t polarity) {
  // configure the port A
  uint8_t ioconfValue = readRegister(MCP23017_IOCONA);
  bitWrite(ioconfValue, 6, mirroring);
  bitWrite(ioconfValue, 2, openDrain);
  bitWrite(ioconfValue, 1, polarity);
  writeRegister(MCP23017_IOCONA, ioconfValue);

  // Configure the port B
  ioconfValue = readRegister(MCP23017_IOCONB);
  bitWrite(ioconfValue, 6, mirroring);
  bitWrite(ioconfValue, 2, openDrain);
  bitWrite(ioconfValue, 1, polarity);
  writeRegister(MCP23017_IOCONB, ioconfValue);
}

/**
 * Set's up a pin for interrupt. uses arduino MODEs: CHANGE, FALLING, RISING.
 *
 * Note that the interrupt condition finishes when you read the information
 * about the port / value that caused the interrupt or you read the port itself.
 * Check the datasheet can be confusing.
 * @param pin Pin to set
 * @param mode Mode to set the pin
 *
 */
void TeelSys_MCP23S17::setupInterruptPin(uint8_t pin, uint8_t mode) {

  // set the pin interrupt control (0 means change, 1 means compare against
  // given value);
  updateRegisterBit(pin, (mode != CHANGE), MCP23017_INTCONA, MCP23017_INTCONB);
  // if the mode is not CHANGE, we need to set up a default value, different
  // value triggers interrupt

  // In a RISING interrupt the default value is 0, interrupt is triggered when
  // the pin goes to 1. In a FALLING interrupt the default value is 1, interrupt
  // is triggered when pin goes to 0.
  updateRegisterBit(pin, (mode == FALLING), MCP23017_DEFVALA, MCP23017_DEFVALB);

  // enable the pin for interrupt
  updateRegisterBit(pin, HIGH, MCP23017_GPINTENA, MCP23017_GPINTENB);
}

/**
 * Disable a pin for interrupt.
 *
 * @param pin Pin to set
 *
 */
void TeelSys_MCP23S17::disableInterruptPin(uint8_t pin) {
  // disable the pin for interrupt
  updateRegisterBit(pin, LOW, MCP23017_GPINTENA, MCP23017_GPINTENB);
}

/*!
 * @brief Gets the last interrupt pin
 * @return Returns the last interrupt pin
 */
uint8_t TeelSys_MCP23S17::getLastInterruptPin() {
  uint8_t intf;

  // try port A
  intf = readRegister(MCP23017_INTFA);
  for (int i = 0; i < 8; i++)
    if (bitRead(intf, i))
      return i;

  // try port B
  intf = readRegister(MCP23017_INTFB);
  for (int i = 0; i < 8; i++)
    if (bitRead(intf, i))
      return i + 8;

  return MCP23017_INT_ERR;
}
/*!
 * @brief Gets the value of the last interrupt pin
 * @return Returns the value of the last interrupt pin
 */
uint8_t TeelSys_MCP23S17::getLastInterruptPinValue() {
  uint8_t intPin = getLastInterruptPin();
  if (intPin != MCP23017_INT_ERR) {
    uint8_t intcapreg = regForPin(intPin, MCP23017_INTCAPA, MCP23017_INTCAPB);
    uint8_t bit = bitForPin(intPin);
    return (readRegister(intcapreg) >> bit) & (0x01);
  }

  return MCP23017_INT_ERR;
}