/*************************************************** 
 This is a library for the MCP23017 i2c port expander

 These displays use I2C to communicate, 2 pins are required to
 interface
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 Written by Limor Fried/Ladyada for Adafruit Industries.
 BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <stdint.h>
#include "Adafruit_MCP23017.h"
#include <driver/gpio.h>
#include <driver/i2c.h>

//#define m_i2caddr 0x20

// registers
#define MCP23017_IODIRA 0x00
#define MCP23017_IPOLA 0x02
#define MCP23017_GPINTENA 0x04
#define MCP23017_DEFVALA 0x06
#define MCP23017_INTCONA 0x08
#define MCP23017_IOCONA 0x0A
#define MCP23017_GPPUA 0x0C
#define MCP23017_INTFA 0x0E
#define MCP23017_INTCAPA 0x10
#define MCP23017_GPIOA 0x12
#define MCP23017_OLATA 0x14


#define MCP23017_IODIRB 0x01
#define MCP23017_IPOLB 0x03
#define MCP23017_GPINTENB 0x05
#define MCP23017_DEFVALB 0x07
#define MCP23017_INTCONB 0x09
#define MCP23017_IOCONB 0x0B
#define MCP23017_GPPUB 0x0D
#define MCP23017_INTFB 0x0F
#define MCP23017_INTCAPB 0x11
#define MCP23017_GPIOB 0x13
#define MCP23017_OLATB 0x15

#define MCP23017_INT_ERR 255


static int bitRead(uint32_t x, uint8_t n) {
	return ((x & 1<<n) != 0);
}

static uint32_t bitWrite(uint32_t x, uint8_t n, uint8_t b) {
	return ((x & (~(1<< n))) | (b << n));
}

/**
 * Bit number associated to a give Pin
 */
uint8_t Adafruit_MCP23017::bitForPin(uint8_t pin){
	return pin%8;
}

/**
 * Register address, port dependent, for a given PIN
 */
uint8_t Adafruit_MCP23017::regForPin(uint8_t pin, uint8_t portAaddr, uint8_t portBaddr){
	return (pin<8)?portAaddr:portBaddr;
}

/**
 * Reads a given register
 */
uint8_t Adafruit_MCP23017::readRegister(uint8_t addr){
	// read the current GPINTEN
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (m_i2caddr << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
	i2c_master_write_byte(cmd, addr, 1);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(m_port, cmd, 0);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (m_i2caddr << 1) | I2C_MASTER_READ, 1 /* expect ack */);
	uint8_t tmpByte;
	i2c_master_read_byte(cmd, &tmpByte, I2C_MASTER_NACK);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(m_port, cmd, 0);
	i2c_cmd_link_delete(cmd);
	return tmpByte;
}


/**
 * Writes a given register
 */
void Adafruit_MCP23017::writeRegister(uint8_t regAddr, uint8_t regValue){
	// Write the register
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (m_i2caddr << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
	i2c_master_write_byte(cmd, regAddr, 1);
	i2c_master_write_byte(cmd, regValue, 1);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(m_port, cmd, 0);
	i2c_cmd_link_delete(cmd);
}

/**
 * Helper to update a single bit of an A/B register.
 * - Reads the current register value
 * - Writes the new register value
 */
void Adafruit_MCP23017::updateRegisterBit(uint8_t pin, uint8_t pValue, uint8_t portAaddr, uint8_t portBaddr) {
	uint8_t regValue;
	uint8_t regAddr=regForPin(pin,portAaddr,portBaddr);
	uint8_t bit=bitForPin(pin);
	regValue = readRegister(regAddr);

	// set the value for the particular bit
	regValue = bitWrite(regValue,bit,pValue);

	writeRegister(regAddr,regValue);
}

////////////////////////////////////////////////////////////////////////////////


Adafruit_MCP23017::Adafruit_MCP23017(uint8_t addr, i2c_port_t port, gpio_num_t sda, gpio_num_t scl) {
    m_i2caddr = addr;
    m_port = port;
	m_sda = sda;
	m_scl = scl;

	i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = m_sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = m_scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(m_port, &conf);
    i2c_driver_install(m_port, conf.mode, 0, 0, 0);

	writeRegister(MCP23017_IODIRA, 0xFF);
	writeRegister(MCP23017_IODIRB, 0xFF);
}

Adafruit_MCP23017::~Adafruit_MCP23017() {

}

/**
 * Sets the pin mode to either INPUT or OUTPUT
 */
void Adafruit_MCP23017::pinMode(uint8_t p, uint8_t d) {
	updateRegisterBit(p,(d==GPIO_MODE_INPUT),MCP23017_IODIRA,MCP23017_IODIRB);
}

/**
 * Reads all 16 pins (port A and B) into a single 16 bits variable.
 */
uint16_t Adafruit_MCP23017::readGPIOAB() {
	uint16_t ba = 0;
	uint8_t a;

	// read the current GPIO output latches
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (m_i2caddr << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
	i2c_master_write_byte(cmd, MCP23017_GPIOA, 1);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(m_port, cmd, 0);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (m_i2caddr << 1) | I2C_MASTER_READ, 1 /* expect ack */);
	i2c_master_read_byte(cmd, &a, I2C_MASTER_NACK);
	i2c_master_read_byte(cmd, (uint8_t *)&ba, I2C_MASTER_NACK);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(m_port, cmd, 0);
	i2c_cmd_link_delete(cmd);

	ba <<= 8;
	ba |= a;

	return ba;
}

/**
 * Read a single port, A or B, and return its current 8 bit value.
 * Parameter b should be 0 for GPIOA, and 1 for GPIOB.
 */
uint8_t Adafruit_MCP23017::readGPIO(uint8_t b) {

	// read the current GPIO output latches
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (m_i2caddr << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
	if (b == 0) {
		i2c_master_write_byte(cmd, MCP23017_GPIOA, 1);
	} else {
		i2c_master_write_byte(cmd, MCP23017_GPIOB, 1);
	}
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(m_port, cmd, 0);
	i2c_cmd_link_delete(cmd);

	uint8_t byte;
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (m_i2caddr << 1) | I2C_MASTER_READ, 1 /* expect ack */);
	i2c_master_read_byte(cmd, &byte, I2C_MASTER_NACK);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(m_port, cmd, 0);
	i2c_cmd_link_delete(cmd);
	return byte;
}

/**
 * Writes all the pins in one go. This method is very useful if you are implementing a multiplexed matrix and want to get a decent refresh rate.
 */
void Adafruit_MCP23017::writeGPIOAB(uint16_t ba) {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (m_i2caddr << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
	i2c_master_write_byte(cmd, MCP23017_GPIOA, 1);
	i2c_master_write_byte(cmd, ba & 0xFF, 1);
	i2c_master_write_byte(cmd, ba >> 8, 1);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(m_port, cmd, 0);
	i2c_cmd_link_delete(cmd);
}

void Adafruit_MCP23017::digitalWrite(uint8_t pin, uint8_t d) {
	uint8_t gpio;
	uint8_t bit=bitForPin(pin);


	// read the current GPIO output latches
	uint8_t regAddr=regForPin(pin,MCP23017_OLATA,MCP23017_OLATB);
	gpio = readRegister(regAddr);

	// set the pin and direction
	gpio = bitWrite(gpio,bit,d);

	// write the new GPIO
	regAddr=regForPin(pin,MCP23017_GPIOA,MCP23017_GPIOB);
	writeRegister(regAddr,gpio);
}

void Adafruit_MCP23017::pullUp(uint8_t p, uint8_t d) {
	updateRegisterBit(p,d,MCP23017_GPPUA,MCP23017_GPPUB);
}

uint8_t Adafruit_MCP23017::digitalRead(uint8_t pin) {
	uint8_t bit=bitForPin(pin);
	uint8_t regAddr=regForPin(pin,MCP23017_GPIOA,MCP23017_GPIOB);
	return (readRegister(regAddr) >> bit) & 0x1;
}

/**
 * Configures the interrupt system. both port A and B are assigned the same configuration.
 * Mirroring will OR both INTA and INTB pins.
 * Opendrain will set the INT pin to value or open drain.
 * polarity will set LOW or HIGH on interrupt.
 * Default values after Power On Reset are: (false,flase, LOW)
 * If you are connecting the INTA/B pin to arduino 2/3, you should configure the interupt handling as FALLING with
 * the default configuration.
 */
void Adafruit_MCP23017::setupInterrupts(uint8_t mirroring, uint8_t openDrain, uint8_t polarity){
	// configure the port A
	uint8_t ioconfValue=readRegister(MCP23017_IOCONA);
	ioconfValue = bitWrite(ioconfValue,6,mirroring);
	ioconfValue = bitWrite(ioconfValue,2,openDrain);
	ioconfValue = bitWrite(ioconfValue,1,polarity);
	writeRegister(MCP23017_IOCONA,ioconfValue);

	// Configure the port B
	ioconfValue=readRegister(MCP23017_IOCONB);
	ioconfValue = bitWrite(ioconfValue,6,mirroring);
	ioconfValue = bitWrite(ioconfValue,2,openDrain);
	ioconfValue = bitWrite(ioconfValue,1,polarity);
	writeRegister(MCP23017_IOCONB,ioconfValue);
}

uint8_t Adafruit_MCP23017::getLastInterruptPin(){
	uint8_t intf;

	// try port A
	intf=readRegister(MCP23017_INTFA);
	for(int i=0;i<8;i++) if (bitRead(intf,i)) return i;

	// try port B
	intf=readRegister(MCP23017_INTFB);
	for(int i=0;i<8;i++) if (bitRead(intf,i)) return i+8;

	return MCP23017_INT_ERR;

}
uint8_t Adafruit_MCP23017::getLastInterruptPinValue(){
	uint8_t intPin=getLastInterruptPin();
	if(intPin!=MCP23017_INT_ERR){
		uint8_t intcapreg=regForPin(intPin,MCP23017_INTCAPA,MCP23017_INTCAPB);
		uint8_t bit=bitForPin(intPin);
		return (readRegister(intcapreg)>>bit) & (0x01);
	}

	return MCP23017_INT_ERR;
}


