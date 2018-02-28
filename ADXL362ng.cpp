/*
* Library for talking to an ADXL362 motion sensor.
*
* Datasheet: http://www.analog.com/media/en/technical-documentation/data-sheets/ADXL362.pdf
*/

#include <Arduino.h>
#include <SPI.h>
#include <ADXL362ng.h>

// Constructor and general methods {{{
ADXL362ng::ADXL362ng() {
}

// Checks that the connected device is indeed ADXL362, and resets it.
// Needs the CS / Slave Select pin
bool ADXL362ng::begin(uint16_t csPin) {
	this->csPin = csPin;
	SPI.begin();
	SPI.setDataMode(SPI_MODE0);
	pinMode(this->csPin,OUTPUT);
	if(isValidAdxl362()) {
		reset();
		return(true);
	}
	return false;
}

// Check that an ADXL362 sensor is connected
bool ADXL362ng::isValidAdxl362() {
	return
		(getDeviceIdAd()   == 0xAD) &&
		(getDeviceIdMst()  == 0x1D) &&
		(getDevicePartId() == 0xF2);
}

// Checks if the sensor is ready. Returns false if the sensor is not yet
// configured, has experienced a power glitch, or has invalid settings
bool ADXL362ng::ready() {
	return !getStatusErrorUserRegs();
}
// }}}

// Helper(s) for common use cases {{{

// Enables autonomous motion switch in looped wakeup mode, as described in the
// datasheet. Sets the interrupt pin high when activity is detected, low when
// inactivity is detected.
bool ADXL362ng::autonomousMotionSwitch(int interruptPin, int16_t activityThreshold, int8_t activityTime, int16_t inactivityThreshold, int16_t inactivityTime) {
	return
		setActivityThreshold(activityThreshold) &&
		setActivityTime(activityTime) &&
		setInactivityThreshold(inactivityThreshold) &&
		setInactivityTime(inactivityTime) &&
		setInterruptOnAwake(interruptPin) &&
		setLinkLoop(ADXL362_LINKLOOP_LOOP) &&
		setActivityEnable() &&
		setActivityReferenced() &&
		setInactivityEnable() &&
		setInactivityReferenced() &&
		setWakeup() &&
		measure() &&
		ready();
}
// }}}

// Methods for device registers {{{
byte ADXL362ng::getDeviceIdAd() {
	return readRegister(ADXL362_REG_DEVID_AD);
}

byte ADXL362ng::getDeviceIdMst() {
	return readRegister(ADXL362_REG_DEVID_MST);
}

byte ADXL362ng::getDevicePartId() {
	return readRegister(ADXL362_REG_PARTID);
}

byte ADXL362ng::getDeviceRevision() {
	return readRegister(ADXL362_REG_REVID);
}
// }}}

// Methods for getting X/Y/Z axis data {{{

// These methods access only the 8 most significant bits.
// Useful for low power applications where higher precision is not needed.
int ADXL362ng::getXHigh() {
	return (int)readRegister(ADXL362_REG_XDATA);
}
int ADXL362ng::getYHigh() {
	return (int)readRegister(ADXL362_REG_YDATA);
}
int ADXL362ng::getZHigh() {
	return (int)readRegister(ADXL362_REG_ZDATA);
}

// These methods get the full 12 bit data
int16_t ADXL362ng::getX() {
	return (int16_t)readDoubleRegister(ADXL362_REG_XDATA_L) & 0x0fff;
}
int16_t ADXL362ng::getY() {
	return (int16_t)readDoubleRegister(ADXL362_REG_YDATA_L) & 0x0fff;
}
int16_t ADXL362ng::getZ() {
	return (int16_t)readDoubleRegister(ADXL362_REG_ZDATA_L) & 0x0fff;
}
// }}}

// Temperature {{{
int16_t ADXL362ng::getTemperature() {
	return (int16_t)readDoubleRegister(ADXL362_REG_TEMP_L) & 0x0fff;
}
// }}}

// Software reset {{{
void ADXL362ng::reset() {
	writeRegister(ADXL362_REG_SOFTRST,0x52);
	// Must wait approximately 0.5ms
	delay(10);
}
// }}}

// Activity/inactivity thresholds and time {{{

// Actity threshold. 11 bit unsigned
bool ADXL362ng::setActivityThreshold(uint16_t value) {
	return writeDoubleRegister(ADXL362_REG_THRES_ACT_L,value);
}
uint16_t ADXL362ng::getActivityThreshold() {
	return (readDoubleRegister(ADXL362_REG_THRES_ACT_L) & 0x07ff);
}

// Activity time. Sets the number of consequtive samples that must have at least one axis greather than the threshold
bool ADXL362ng::setActivityTime(uint8_t value) {
	return writeRegister(ADXL362_REG_TIME_ACT,value);
}
uint8_t ADXL362ng::getActivityTime() {
	return readRegister(ADXL362_REG_TIME_ACT);
}

// Inactity threshold. 11 bit unsigned
bool ADXL362ng::setInactivityThreshold(uint16_t value) {
	return writeDoubleRegister(ADXL362_REG_THRES_INACT_L,value);
}
uint16_t ADXL362ng::getInactivityThreshold() {
	return (readDoubleRegister(ADXL362_REG_THRES_INACT_L) & 0x07ff);
}

// Inactivity time. Sets the number of consequtive samples that must have all axes lower than the threshold. 16 bit
bool ADXL362ng::setInactivityTime(uint16_t value) {
	return writeDoubleRegister(ADXL362_REG_TIME_INACT_L,value);
}
uint16_t ADXL362ng::getInactivityTime() {
	return (readDoubleRegister(ADXL362_REG_TIME_INACT_L));
}
// }}}

// Activity/inactivity control registers {{{
bool ADXL362ng::setActivityEnable(bool enable) {
	return setRegisterBits(ADXL362_REG_ACT_INACT_CTL,ADXL362_ACT_EN,enable);
}
bool ADXL362ng::getActivityEnable() {
	return getRegisterBits(ADXL362_REG_ACT_INACT_CTL,ADXL362_ACT_EN);
}

bool ADXL362ng::setActivityReferenced(bool enable) {
	return setRegisterBits(ADXL362_REG_ACT_INACT_CTL,ADXL362_ACT_REF,enable);
}
bool ADXL362ng::getActivityReferenced() {
	return getRegisterBits(ADXL362_REG_ACT_INACT_CTL,ADXL362_ACT_REF);
}

bool ADXL362ng::setInactivityEnable(bool enable) {
	return setRegisterBits(ADXL362_REG_ACT_INACT_CTL,ADXL362_INACT_EN,enable);
}
bool ADXL362ng::getInactivityEnable() {
	return getRegisterBits(ADXL362_REG_ACT_INACT_CTL,ADXL362_INACT_EN);
}

bool ADXL362ng::setInactivityReferenced(bool enable) {
	return setRegisterBits(ADXL362_REG_ACT_INACT_CTL,ADXL362_INACT_REF,enable);
}
bool ADXL362ng::getInactivityReferenced() {
	return getRegisterBits(ADXL362_REG_ACT_INACT_CTL,ADXL362_INACT_REF);
}

bool ADXL362ng::setLinkLoop(byte mode) {
	// Modes:
	// ADXL362_LINKLOOP_DEFAULT
	// ADXL362_LINKLOOP_LINKED
	// ADXL362_LINKLOOP_LOOP
	return writeRegister(ADXL362_REG_ACT_INACT_CTL,mode,ADXL362_LINKLOOP_BITMASK);
}
byte ADXL362ng::getLinkLoop() {
	return readRegister(ADXL362_REG_ACT_INACT_CTL) & ADXL362_LINKLOOP_BITMASK;
}
// }}}

// Methods for status register {{{
bool ADXL362ng::getStatusDataReady() {
	return getRegisterBits(ADXL362_REG_STATUS,ADXL362_DATA_RDY);
}
bool ADXL362ng::getStatusFifoReady() {
	return getRegisterBits(ADXL362_REG_STATUS,ADXL362_FIFO_RDY);
}
bool ADXL362ng::getStatusFifoWatermark() {
	return getRegisterBits(ADXL362_REG_STATUS,ADXL362_FIFO_WM);
}
bool ADXL362ng::getStatusFifoOverrun() {
	return getRegisterBits(ADXL362_REG_STATUS,ADXL362_FIFO_OR);
}
bool ADXL362ng::getStatusActivity() {
	return getRegisterBits(ADXL362_REG_STATUS,ADXL362_ACTIVITY);
}
bool ADXL362ng::getStatusInactivity() {
	return getRegisterBits(ADXL362_REG_STATUS,ADXL362_INACTIVITY);
}
bool ADXL362ng::getStatusAwake() {
	return getRegisterBits(ADXL362_REG_STATUS,ADXL362_AWAKE);
}
bool ADXL362ng::getStatusErrorUserRegs() {
	return getRegisterBits(ADXL362_REG_STATUS,ADXL362_ERR_USER_REGS);
}
// }}}

// Methods for INT1/INT2 mapping {{{
/* ADXL362 has two interrupt pins. These can be individually configured to pull
 * high (default) or low when one of several events happen
 */

bool ADXL362ng::setInterruptOnDataReady(int pin, bool enable) {
	return setRegisterBits(intPinToReg(pin),ADXL362_INT_DATA_RDY,enable);
}
bool ADXL362ng::getInterruptOnDataReady(int pin) {
	return getRegisterBits(intPinToReg(pin),ADXL362_INT_DATA_RDY);
}

bool ADXL362ng::setInterruptOnFifoReady(int pin, bool enable) {
	return setRegisterBits(intPinToReg(pin),ADXL362_INT_FIFO_RDY,enable);
}
bool ADXL362ng::getInterruptOnFifoReady(int pin) {
	return getRegisterBits(intPinToReg(pin),ADXL362_INT_FIFO_RDY);
}

bool ADXL362ng::setInterruptOnFifoWatermark(int pin, bool enable) {
	return setRegisterBits(intPinToReg(pin),ADXL362_INT_FIFO_WM,enable);
}
bool ADXL362ng::getInterruptOnFifoWatermark(int pin) {
	return getRegisterBits(intPinToReg(pin),ADXL362_INT_FIFO_WM);
}

bool ADXL362ng::setInterruptOnFifoOverrun(int pin, bool enable) {
	return setRegisterBits(intPinToReg(pin),ADXL362_INT_FIFO_OR,enable);
}
bool ADXL362ng::getInterruptOnFifoOverrun(int pin) {
	return getRegisterBits(intPinToReg(pin),ADXL362_INT_FIFO_OR);
}

bool ADXL362ng::setInterruptOnActivity(int pin, bool enable) {
	return setRegisterBits(intPinToReg(pin),ADXL362_INT_ACTIVITY,enable);
}
bool ADXL362ng::getInterruptOnActivity(int pin) {
	return getRegisterBits(intPinToReg(pin),ADXL362_INT_ACTIVITY);
}

bool ADXL362ng::setInterruptOnInactivity(int pin, bool enable) {
	return setRegisterBits(intPinToReg(pin),ADXL362_INT_INACTIVITY,enable);
}
bool ADXL362ng::getInterruptOnInactivity(int pin) {
	return getRegisterBits(intPinToReg(pin),ADXL362_INT_INACTIVITY);
}

bool ADXL362ng::setInterruptOnAwake(int pin, bool enable) {
	return setRegisterBits(intPinToReg(pin),ADXL362_INT_AWAKE,enable);
}
bool ADXL362ng::getInterruptOnAwake(int pin) {
	return getRegisterBits(intPinToReg(pin),ADXL362_INT_AWAKE);
}

// Use this if you need to interrupt pin pulled low instead of high
bool ADXL362ng::setInterruptActiveLow(int pin, bool enable) {
	return setRegisterBits(intPinToReg(pin),ADXL362_INT_LOW,enable);
}
bool ADXL362ng::getInterruptActiveLow(int pin) {
	return getRegisterBits(intPinToReg(pin),ADXL362_INT_LOW);
}

// Maps from an interrupt pin number (on the ADXL362 module) to the
// corresponding register address. Also accepts a valid INT1/INT2 register
// address
byte ADXL362ng::intPinToReg(int pin) {
	switch(pin) {
		case            1: return ADXL362_REG_INT1;
		case            2: return ADXL362_REG_INT2;
		case ADXL362_REG_INT1: return pin;
		case ADXL362_REG_INT2: return pin;
	}
	return ADXL362_REG_INVALID; // Default to invalid address, which does nothing
}
// }}}

// Filter control register {{{
// TODO TODO TODO
// }}}

// Power control register {{{

bool ADXL362ng::setAutosleep(bool enabled) {
	return setRegisterBits(ADXL362_REG_PWRCTL, ADXL362_AUTOSLEEP, enabled);
}

bool ADXL362ng::getAutosleep() {
	return getRegisterBits(ADXL362_REG_PWRCTL, ADXL362_AUTOSLEEP);
}

bool ADXL362ng::setWakeup(bool enabled) {
	return setRegisterBits(ADXL362_REG_PWRCTL, ADXL362_WAKEUP, enabled);
}

bool ADXL362ng::getWakeup() {
	return getRegisterBits(ADXL362_REG_PWRCTL, ADXL362_WAKEUP);
}

bool ADXL362ng::setMode(byte mode) {
	// Modes:
	// 0x00: Standby
	// 0x02: Measurement mode
	return writeRegister(ADXL362_REG_PWRCTL,mode,ADXL362_MODE_BITMASK);
}
byte ADXL362ng::getMode() {
	return readRegister(ADXL362_REG_PWRCTL) & ADXL362_MODE_BITMASK;
}
bool ADXL362ng::measure() {
	return setMode(ADXL362_MODE_MEASURE);
}
bool ADXL362ng::standby() {
	return setMode(ADXL362_MODE_STANDBY);
}

bool ADXL362ng::setNoise(byte mode) {
	// Low noise modes:
	// 0x00: Normal operation (reset default)
	// 0x10: Low noise mode
	// 0x20: Ultra low noise mode
	return writeRegister(ADXL362_REG_PWRCTL,mode,ADXL362_NOISE_BITMASK);
}

byte ADXL362ng::getNoise() {
	return readRegister(ADXL362_REG_PWRCTL) & ADXL362_NOISE_BITMASK;
}

bool ADXL362ng::setExternalClock(bool enabled) {
	return setRegisterBits(ADXL362_REG_PWRCTL, ADXL362_EXT_CLK, enabled);
}

bool ADXL362ng::getExternalClock() {
	return getRegisterBits(ADXL362_REG_PWRCTL, ADXL362_EXT_CLK);
}
// }}}

// Register helper methods {{{

// Enables or clears bits on the given register address without modifying other bits
bool ADXL362ng::setRegisterBits(byte address, byte bits, bool enable) {
	if(enable)
		// Set the bit(s)
		return writeRegister(address, readRegister(address) | bits);
	else
		// Clear the bit(s)
		return writeRegister(address, readRegister(address) & ~bits);
}

// Returns true if all the given bits are set
bool ADXL362ng::getRegisterBits(byte address, byte bits) {
	return getRegisterBits(address, bits, bits);
}

// Returns true if all the given bits are set, according to a mask
bool ADXL362ng::getRegisterBits(byte address, byte bits, byte mask) {
	return readRegister(address) == (bits & ~mask);
}

// Writes an 8 bit value to the given register address. If a mask is given,
// only bits matching the mask are modified
bool ADXL362ng::writeRegister(byte address, int8_t value, int8_t mask) {
	if(mask!=0xff) {
		value=value & mask;
		value=value | (readRegister(address) & ~mask);
	}
	sSelect();
	SPI.transfer(ADXL362_CMD_WRITE);
	SPI.transfer(address);
	SPI.transfer(value);
	sDeselect();
	return (value==readRegister(address));
}

int8_t ADXL362ng::readRegister(byte address) {
	byte result;
	sSelect();
	SPI.transfer(ADXL362_CMD_READ);
	SPI.transfer(address);
	result = SPI.transfer(0x00);
	sDeselect();
	return result;
}

bool ADXL362ng::writeDoubleRegister(byte startAddress, int16_t value, int16_t mask) {
	if(mask!=(int16_t)0xffff) {
		value=value & mask;
		value=value | (readRegister(startAddress) & ~mask);
	}
	sSelect();
	SPI.transfer(ADXL362_CMD_WRITE);
	SPI.transfer(startAddress);
	SPI.transfer(value);            // LSB
	SPI.transfer((byte)(value>>8)); // MSB
	sDeselect();
	return (value==readDoubleRegister(startAddress));
}

int16_t ADXL362ng::readDoubleRegister(byte startAddress) {
	int16_t result;
	sSelect();
	SPI.transfer(ADXL362_CMD_READ);
	SPI.transfer(startAddress);
	result = SPI.transfer(0x00);
	result+=(SPI.transfer(0x00)<<8);
	sDeselect();
	return result;
}

// Method for debugging. Prints all registers
/*
void ADXL362ng::printRegisters() {
	Serial.println(F("Printing content of all registers"));
	Serial.println();
	Serial.println(F("Register  Value"));
	sSelect();
	SPI.transfer(ADXL362_CMD_READ);
	SPI.transfer(0x00); // Starting address
	for(byte reg=0x00; reg<=0x3F; reg++) {
		byte val=SPI.transfer(0x00); // Read value
		char buf[25] = {0};
		snprintf(buf,25,"0x%02x = 0x%02x (%3d) ",reg,val,val);
		Serial.print(buf);
		switch(reg) {
			case ADXL362_REG_DEVID_AD      : Serial.print(F(" DEVID_AD")); break;
			case ADXL362_REG_DEVID_MST     : Serial.print(F(" DEVID_MST")); break;
			case ADXL362_REG_PARTID        : Serial.print(F(" REG_PARTID")); break;
			case ADXL362_REG_REVID         : Serial.print(F(" REG_REVID")); break;
			case ADXL362_REG_XDATA         : Serial.print(F(" REG_XDATA")); break;
			case ADXL362_REG_YDATA         : Serial.print(F(" REG_YDATA")); break;
			case ADXL362_REG_ZDATA         : Serial.print(F(" REG_ZDATA")); break;
			case ADXL362_REG_STATUS        : Serial.print(F(" REG_STATUS")); break;
			case ADXL362_REG_XDATA_L       : Serial.print(F(" REG_XDATA_L")); break;
			case ADXL362_REG_XDATA_H       : Serial.print(F(" REG_XDATA_H")); break;
			case ADXL362_REG_YDATA_L       : Serial.print(F(" REG_YDATA_L")); break;
			case ADXL362_REG_YDATA_H       : Serial.print(F(" REG_YDATA_H")); break;
			case ADXL362_REG_ZDATA_L       : Serial.print(F(" REG_ZDATA_L")); break;
			case ADXL362_REG_ZDATA_H       : Serial.print(F(" REG_ZDATA_H")); break;
			case ADXL362_REG_TEMP_L        : Serial.print(F(" REG_TEMP_L")); break;
			case ADXL362_REG_TEMP_H        : Serial.print(F(" REG_TEMP_H")); break;
			case ADXL362_REG_SOFTRST       : Serial.print(F(" REG_SOFTRST")); break;
			case ADXL362_REG_THRES_ACT_L   : Serial.print(F(" REG_THRES_ACT_L")); break;
			case ADXL362_REG_THRES_ACT_H   : Serial.print(F(" REG_THRES_ACT_H")); break;
			case ADXL362_REG_TIME_ACT      : Serial.print(F(" REG_TIME_ACT")); break;
			case ADXL362_REG_THRES_INACT_L : Serial.print(F(" REG_THRES_INACT_L")); break;
			case ADXL362_REG_THRES_INACT_H : Serial.print(F(" REG_THRES_INACT_H")); break;
			case ADXL362_REG_TIME_INACT_L  : Serial.print(F(" REG_TIME_INACT_L")); break;
			case ADXL362_REG_TIME_INACT_H  : Serial.print(F(" REG_TIME_INACT_H")); break;
			case ADXL362_REG_ACT_INACT_CTL : Serial.print(F(" REG_ACT_INACT_CTL")); break;
			case ADXL362_REG_INT1          : Serial.print(F(" REG_INT1")); break;
			case ADXL362_REG_INT2          : Serial.print(F(" REG_INT2")); break;
			case ADXL362_REG_FILTER_CTL    : Serial.print(F(" REG_FILTER_CTL")); break;
			case ADXL362_REG_PWRCTL        : Serial.print(F(" REG_PWRCTL")); break;
			case ADXL362_REG_INVALID       : Serial.print(F(" REG_INVALID")); break;
		}
		Serial.println();
	}
	Serial.println();
	sDeselect();
}
*/
// }}}

// SPI selectors {{{
void ADXL362ng::sSelect() {
	digitalWrite(csPin,LOW);
	// Not sure this delay is needed
	delay(1);
}

void ADXL362ng::sDeselect() {
	digitalWrite(csPin,HIGH);
}
// }}}
