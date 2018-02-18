#ifndef ADXL362_H
#define ADXL362_H

// Instructions
#define ADXL362_CMD_WRITE 0x0A
#define ADXL362_CMD_READ  0x0B

// Registers
#define ADXL362_REG_DEVID_AD      0x00 // Device ID (must be 0xAD, Analog Devices)
#define ADXL362_REG_DEVID_MST     0x01 // MEMS device ID (must be 0x1D, ADXL362)
#define ADXL362_REG_PARTID        0x02 // Part ID (must be 0xF2)
#define ADXL362_REG_REVID         0x03 // Silicon revision number, starting at 1
#define ADXL362_REG_XDATA         0x08 // 8 most significant bits of x-axis accelerometer data
#define ADXL362_REG_YDATA         0x09 // 8 most significant bits of y-axis accelerometer data
#define ADXL362_REG_ZDATA         0x0A // 8 most significant bits of z-axis accelerometer data
#define ADXL362_REG_STATUS        0x0B // Status
#define ADXL362_REG_XDATA_L       0x0E // 12-bit X-axis data, LSB
#define ADXL362_REG_XDATA_H       0x0F // 12-bit X-axis data, MSB
#define ADXL362_REG_YDATA_L       0x10 // 12-bit Y-axis data, LSB
#define ADXL362_REG_YDATA_H       0x11 // 12-bit Y-axis data, MSB
#define ADXL362_REG_ZDATA_L       0x12 // 12-bit Z-axis data, LSB
#define ADXL362_REG_ZDATA_H       0x13 // 12-bit Z-axis data, MSB
#define ADXL362_REG_TEMP_L        0x14 // 12-bit temperature, LSB
#define ADXL362_REG_TEMP_H        0x15 // 12-bit temperature, MSB
#define ADXL362_REG_SOFTRST       0x1F // Soft reset (write 0x52 or "R" to reset. Wait 0.5ms)
#define ADXL362_REG_THRES_ACT_L   0x20 // 11-bit unsigned activity threshold, LSB
#define ADXL362_REG_THRES_ACT_H   0x21 // 11-bit unsigned activity threshold, MSB
#define ADXL362_REG_TIME_ACT      0x22 // Activity time
#define ADXL362_REG_THRES_INACT_L 0x23 // 11-bit unsigned inactivity threshold, LSB
#define ADXL362_REG_THRES_INACT_H 0x24 // 11-bit unsigned inactivity threshold, MSB
#define ADXL362_REG_TIME_INACT_L  0x25 // 16-bit unsigned inactivity time, LSB
#define ADXL362_REG_TIME_INACT_H  0x26 // 16-bit unsigned inactivity time, MSB
#define ADXL362_REG_ACT_INACT_CTL 0x27 // Activity/inactivity register
#define ADXL362_REG_INT1          0x2A // INT1 function map register
#define ADXL362_REG_INT2          0x2B // INT2 function map register
#define ADXL362_REG_FILTER_CTL    0x2C // Filter control register
#define ADXL362_REG_PWRCTL        0x2D // Power control register
#define ADXL362_REG_INVALID       0x3F // Invalid address which does nothing

// Values to be used with the registers

// Status register (all RO)
#define ADXL362_DATA_RDY      0x01 // Data ready
#define ADXL362_FIFO_RDY      0x02 // FIFO ready
#define ADXL362_FIFO_WM       0x04 // FIFO watermark
#define ADXL362_FIFO_OR       0x08 // FIFO overrun
#define ADXL362_ACTIVITY      0x10 // Activity
#define ADXL362_INACTIVITY    0x20 // Inactivity
#define ADXL362_AWAKE         0x40 // Accelerometer is awake (default 1)
#define ADXL362_ERR_USER_REGS 0x80 // Power glitch, invalid settings, or not yet configured

// Activity/inactivity control register
#define ADXL362_ACT_EN        0x01
#define ADXL362_ACT_REF       0x02
#define ADXL362_INACT_EN      0x04
#define ADXL362_INACT_REF     0x08
#define ADXL362_LINKLOOP_BITMASK 0x30 // Linkloop consists of 2 bits
#define ADXL362_LINKLOOP_DEFAULT 0x00
#define ADXL362_LINKLOOP_LINKED  0x10
#define ADXL362_LINKLOOP_LOOP    0x30

// Interrupt map register (same for INT1 and INT2, all RW))
#define ADXL362_INT_DATA_RDY   0x01 // Data ready
#define ADXL362_INT_FIFO_RDY   0x02 // FIFO ready
#define ADXL362_INT_FIFO_WM    0x04 // FIFO watermark
#define ADXL362_INT_FIFO_OR    0x08 // FIFO overrun
#define ADXL362_INT_ACTIVITY   0x10 // Activity
#define ADXL362_INT_INACTIVITY 0x20 // Inactivity
#define ADXL362_INT_AWAKE      0x40 // Awake
#define ADXL362_INT_LOW        0x80 // INT pin is active low

// Bit masks for power control register
#define ADXL362_MODE_BITMASK   0x03
#define ADXL362_NOISE_BITMASK  0x30

// Power control register (PWCTL) bits (all RW)
#define ADXL362_MODE_STANDBY   0x00
//                             0x01 (reserved)
#define ADXL362_MODE_MEASURE   0x02
#define ADXL362_AUTOSLEEP      0x04
#define ADXL362_WAKEUP         0x08
#define ADXL362_NOISE_NORMAL   0x00
#define ADXL362_NOISE_LOW      0x10
#define ADXL362_NOISE_ULTRA    0x20 // Ultra low noise
#define ADXL362_EXT_CLK        0x40 // External clock
//                             0x80 (reserved)

class ADXL362ng {
public:
	/* Constructor and general methods */
	ADXL362ng(uint16_t csPin);
	bool begin();
	bool isValidAdxl362();
	bool ready();
	/* Helper(s) for common use cases */
	bool autonomousMotionSwitch(int interruptPin=2, int16_t activityThreshold=250, int8_t activityTime=1, int16_t inactivityThreshold=150, int16_t inactivityTime=30);
	/* Device registers methods */
	byte getDeviceIdAd();
	byte getDeviceIdMst();
	byte getDevicePartId();
	byte getDeviceRevision();
	/* Methods for X/Y/Z registers */
	int getXHigh();
	int getYHigh();
	int getZHigh();
	int16_t getX();
	int16_t getY();
	int16_t getZ();
	/* Temperature */
	int16_t getTemperature();
	/* Software reset */
	void reset();
	/* Activity/inactivity thresholds and time */
	bool setActivityThreshold(uint16_t value);
	uint16_t getActivityThreshold();
	bool setActivityTime(uint8_t value);
	uint8_t getActivityTime();
	bool setInactivityThreshold(uint16_t value);
	uint16_t getInactivityThreshold();
	bool setInactivityTime(uint16_t value);
	uint16_t getInactivityTime();
	/* Activity/inactivity register */
	bool setActivityEnable(bool enable=true);
	bool getActivityEnable();
	bool setActivityReferenced(bool enable=true);
	bool getActivityReferenced();
	bool setInactivityEnable(bool enable=true);
	bool getInactivityEnable();
	bool setInactivityReferenced(bool enable=true);
	bool getInactivityReferenced();
	bool setLinkLoop(byte mode);
	byte getLinkLoop();
	/* Status register methods */
	bool getStatusDataReady();
	bool getStatusFifoReady();
	bool getStatusFifoWatermark();
	bool getStatusFifoOverrun();
	bool getStatusActivity();
	bool getStatusInactivity();
	bool getStatusAwake();
	bool getStatusErrorUserRegs();
	/* Interrupt register methods */
	bool setInterruptOnDataReady(int pin, bool enable=true);
	bool getInterruptOnDataReady(int pin);
	bool setInterruptOnFifoReady(int pin, bool enable=true);
	bool getInterruptOnFifoReady(int pin);
	bool setInterruptOnFifoWatermark(int pin, bool enable=true);
	bool getInterruptOnFifoWatermark(int pin);
	bool setInterruptOnFifoOverrun(int pin, bool enable=true);
	bool getInterruptOnFifoOverrun(int pin);
	bool setInterruptOnActivity(int pin, bool enable=true);
	bool getInterruptOnActivity(int pin);
	bool setInterruptOnInactivity(int pin, bool enable=true);
	bool getInterruptOnInactivity(int pin);
	bool setInterruptOnAwake(int pin, bool enable=true);
	bool getInterruptOnAwake(int pin);
	bool setInterruptActiveLow(int pin, bool enable=true);
	bool getInterruptActiveLow(int pin);
	byte intPinToReg(int pin);
	/* Power control register methods */
	bool setAutosleep(bool enabled=true);
	bool getAutosleep();
	bool setWakeup(bool enabled=true);
	bool getWakeup();
	bool setMode(byte mode);
	bool measure();
	bool standby();
	byte getMode();
	bool setNoise(byte mode);
	byte getNoise();
	bool setExternalClock(bool enabled=true);
	bool getExternalClock();
	/* Debugging */
	void printRegisters();
private:
	uint16_t csPin;
	int8_t readRegister(byte address);
	bool writeDoubleRegister(byte startAddress, int16_t value, int16_t mask=0xffff);
	int16_t readDoubleRegister(byte startAddress);
	bool writeRegister(byte address, int8_t value, int8_t mask=0xff);
	//bool setRegisterMask(byte address, byte bits, byte mask);
	bool getRegisterBits(byte address, byte bits);
	bool getRegisterBits(byte address, byte bits, byte mask);
	bool setRegisterBits(byte address, byte bits, bool enable=true);
	void sSelect();
	void sDeselect();
};

#endif
