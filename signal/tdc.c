/*!
 * @file tdc.c
 *
 * Function implementations to initialize, configure and start the MS1022 Time to Digital Converter.
 * The device is interfaced witht the microcontroller serially through a 4-wire SPI.
 *
 * @note Implementation is based on industry recommended practices.
 *
 * @author Deksios Bekele
 *
 * @date Nov 2, 2020
 *
 */
#include <stdint.h>
#include "tdc.h"
#include "../config/system.h"
#include "../config/spi.h"
#include "../config/pin_io.h"

#ifdef _lk_
volatile uint8_t _intFlag = 0; // whether interrupt pin is LOW

/* Initialize SPI connection (using default hardware SPI pins) and reset the TDC.
 * INT_Pin is the input pin connected to the TDC's INT (intrrupt flag) output */
//GP22(int pinInt, boolean debug, Print& outStream) :
//_pinInt(pinInt),
//_bHardwareInterrupt(0),
//_ISRfunc(pinChangeISR),
//_ISRmode(CHANGE),
//_bDebug(debug),
//_serial(outStream),
//_configRegistersTemp()
//{
//	for (int i = 0; i < CFG_REGISTER_MAX; ++i) {
//		_configRegisters[i] = CFG_DEFAULT_BITMASKS[i];
//	}
//}
//
//~GP22()
//{
//	SPI.end();
//	if (_bHardwareInterrupt) {
//		detachInterrupt(digitalPinToInterrupt(_pinInt));
//	}
//}
/* Setup the interrupt and SPI pins and reset the device */
void init() {
//	pinMode(_pinInt, INPUT_PULLUP);
	//digitalWrite(_pinInt, HIGH);

	attachDefaultInterruptFunc();

//	SPI.setBitOrder(MSBFIRST);
//	SPI.setDataMode(SPI_MODE1); //CPOL=0,CPHA=1
//	SPI.setClockDivider(SPI_CLOCK_DIV2); // 16Mhz clock / 2 = 8Mhz
//	digitalWrite(SS, HIGH);
//	SPI.begin();

	/* Send "power on reset" command */
	sendOpcode(

	);

}

/* Interrupt Service Routine attached to INT0 vector */
void pinChangeISR() {
	_intFlag = !_intFlag;
}

/* Debug ISR with serial prints */
void pinChangeISR_debug() {
//	_intFlag = !_intFlag;
//	if (_intFlag)
//		Serial.print("\\");
//	else
//		Serial.print("/");
}

/* Check whether interrupt pin is set, using hardware or fake */
inline boolean checkInterrupt() {
	if (_bHardwareInterrupt) {
		return _intFlag;
	} else {
		//return (digitalRead(_pinInt) == LOW);
	}
}

void attachDefaultInterruptFunc() {
	/* Choose between hardware interrupts or "faking it" */
//	const int intNum = digitalPinToInterrupt(_pinInt);
//
//	detachInterrupt(intNum);
//
//	if (intNum == NOT_AN_INTERRUPT) {
//		_bHardwareInterrupt = 0;
//		_intFlag = !digitalRead(_pinInt);
//	} else {
//		// clear any pending interrupts
//		EIFR = bit(intNum);
//
//		_intFlag = !digitalRead(_pinInt);
//		_bHardwareInterrupt = 1;
//
//		if (_bDebug) {
//			attachInterrupt(intNum, pinChangeISR_debug, CHANGE);
//		} else {
//			attachInterrupt(intNum, pinChangeISR, CHANGE);
//		}
//	}
//
//	if (_bDebug) {
//		_serial.println(F("attached pinChangeISR_debug"));
//
//	}
}

void attachPreviousInterruptFunc() {
//	const int intNum = digitalPinToInterrupt(_pinInt);
//
//	//TODO: check for intNum == NOT_AN_INTERRUPT
//
//	// clear any pending interrupts
//	EIFR = bit(intNum);
//
//	attachInterrupt(intNum, _ISRfunc, _ISRmode);
//
//	if (_bDebug) {
//		_serial.print(F("attached ISR at 0x"));
//		_serial.println((uint32_t) _ISRfunc, HEX);
//	}
}
void attachInterruptFunc(void (*func)(void), int mode) {
//	_ISRfunc = func;
//	_ISRmode = mode;
//
//	attachPreviousInterruptFunc();
}

/* Test SPI communication to chip by writing to register 1 and reading read
 * register 5. Return true on success, false on error */
boolean testCommunication() {
	boolean ok = 1;
//	unsigned long start_us = micros();

	// Save highest 8 bytes of register 1
	uint8_t initialReg1 = readRegister(READ_REG_1) >> 24;

	// For test data use a scramble of the original
	uint8_t testInput = initialReg1 ^ 0xff + 123;

	// Write the test data, and see if it changes
	writeRegister(1, (uint32_t) (testInput) << 24);
	uint8_t testResult = readRegister(READ_REG_1);

	if (testResult != testInput)
	ok = 0;

	// Restore the original value of register 1 (and check again)
	writeRegister(1, (uint32_t) initialReg1 << 24);
	uint8_t testResult2 = readRegister(READ_REG_1);

	if (testResult2 != initialReg1)
	ok = 0;

//	unsigned long duration_us = micros() - start_us;
//
//	if (_bDebug) {
//		_serial.print(F("Test took "));
//		_serial.print(duration_us);
//		_serial.println(F(" us"));
//	}

	return ok;
}

/* Send a stand-alone opcode
 * (can't be followed by reading bytes, as SS is set low then high) */
void sendOpcode(eOpcode opcode) {
	__ATOMIZE();
//	digitalWriteFast(SS, LOW);
//	SPI.transfer(opcode);
//	digitalWriteFast(SS, HIGH);
	__END_ATOMIC();
}

/* Writes the opcode, then the n lowest bytes in data, over SPI.
 * n can be 0 to 4 */
void writeNBytes(uint8_t opcode, uint32_t data, uint8_t n) {
	__ATOMIZE();
//	digitalWriteFast(SS, LOW);
//
//	//transfer the opcode
//	SPI.transfer(opcode);
//
//	//read the remaning uint8_t
//	for (int shift = (n - 1) * 8; shift >= 0; shift -= 8) {
//		SPI.transfer((uint8_t) (data >> shift));
//	}

	//digitalWriteFast(SS, HIGH);
	__END_ATOMIC();

}

/* Writes the opcode, then reads and returns n bytes
 * n can be 1 to 4
 * Note that this doesn't work for reading IDs (7 bytes)
 *  -- only the lowest 4 bytes are returned.
 */
uint32_t readNBytes(uint8_t opcode, uint8_t n) {
	uint8_t buf;
	uint32_t res = 0;

	__disable_interrupt();
	__no_operation();
	__ATOMIZE();
	//digitalWriteFast(SS, LOW);

	//transfer the opcode
	//SPI.transfer(opcode);

	//read the remaning bytes
//	for (int shift = (n - 1) * 8; shift >= 0; shift -= 8) {
//		buf = SPI.transfer(0x00);
//		res |= ((uint32_t) buf << shift);
//	}

	//digitalWriteFast(SS, HIGH);
	__END_ATOMIC();

	return res;
}

/* Write data into the register at address */
void writeRegister(uint8_t address, reg_t data) {
	writeNBytes(OPCODE_WRITE_ADDRESS + address, data, 4);
	/* Save local copy of register */
	_configRegisters[address] = data;

}

/* Read the register at address */
uint32_t readRegister(eReadRegister reg) {
	return readNBytes(OPCODE_READ_ADDRESS | READ_REGISTER_ADDRS[reg],
			READ_REGISTER_LENGTHS[reg] / 8);
}

/* Convenience function
 * Read the register & convert to float from 16:16 fixed-point format */
float readResult(int address) {
	return fixedPoint16ToFloat(readRegister((eReadRegister) address));
}

/* Read only 2 bytes, for an uncalibrated result in meas. mode 1
 * Return the float representation
 */
int16_t readUncalibratedResult(int address) {
	return readNBytes(OPCODE_READ_ADDRESS | address, 2);
}

/* Wait for the INT pin to go low.
 Return the number of microseconds passed (0 if low on first read),
 or -1 if the timeout reached. timeout_us=0 --> no timeout*/
long waitForInterrupt(long timeout_us) {
	/*TODO - Fix later*/
	//unsigned long start_time = micros();
	unsigned long start_time;
	unsigned long waited_us = 0;
	while (!checkInterrupt()) {
		//waited_us = micros() - start_time;
		if (timeout_us && waited_us > timeout_us) {
			return -1;
		}
	}
	return waited_us;
}

/* Read the status register for the ALU pointer, ALU_OP_PTR.
 *
 * Note ALU_OP_PTR points to the next "empty" register, where the following
 * calculation will be stored. 0 means no measurements yet performed.
 * "After a measurement ALU_OP_PTR minus 1 will point to the ALU result,"
 */
int getALUPointer() {
	return ((readRegister(READ_STAT) & STAT_ALU_OP_PTR * 0x7) / STAT_ALU_OP_PTR);
}

/* Get CLOCK_FACTOR (1, 2, or 3) based on the local register cache */
int8_t getClockFactor() {
	int DIV_CLKHS = (_configRegisters[0] & CFG0_DIV_CLKHS_0 * 0x3)
	/ CFG0_DIV_CLKHS_0;
	return (DIV_CLKHS < 3 ? (1 << DIV_CLKHS) : (1 << 2));
}

/* Return the expected number of cycles during CAL_RESONATOR procedure
 * This depends on DIV_CLKHS which is obtained from local register cache
 * It also depends on ANZ_PER_CALRES which is assumed constant, as set in getResonatorCycles()
 */
float getResonatorCyclesTheoretical() {
	// Need DIV_CLKHS for theoretical result.
	const int CLOCK_FACTOR = getClockFactor();

	// Theoretical result
	return 2.0 / REF_CLK_FREQ_HZ * (1 << DEFAULT_ANZ_PER_CALRES )
	* HS_CLK_FREQ_HZ / CLOCK_FACTOR;

}

/* For calibrating the ceramic resonator.
 * Return the measured cycles, or 0 for failure
 */
float getResonatorCycles() {
	float res_meas = 0;

	// Need EN_AUTOCALC=0 for this. Save a temp copy of the register and disable
	// Also need SEL_TIMO_MB2=3 (discovered by experimentation, not in datasheet)

	writeRegister(3,
			_configRegistersTemp[3] & ~CFG3_EN_AUTOCALC_MB2
			| CFG3_SEL_TIMO_MB2_0 * 0x3);

	// Measure actual value
	sendOpcode(OPCODE_INIT);
	sendOpcode(OPCODE_START_CAL_RESONATOR);

	int w = waitForInterrupt(1000000);

	// dirty debug: this just helps it run,
	// for some reason, while measurements are already running
	//delay(10);

	if (w == -1) {
		//resonation calibration failed
	} else {
		// Read measured value (convert from 16:16 fixed point format)
		res_meas = readResult(READ_RES_0);
	}

	return res_meas;
}

//float getResonatorCorrectionFactor() {
//return float(_calibration.resonator_theor_cycles) / _calibration.resonator_meas_cycles;
//}

float getCycleTime_ns() {
	return (_calibration.Tref_theor_ns * _calibration.clock_factor
			* _calibration.resonator_theor_cycles
			/ _calibration.resonator_meas_cycles / _calibration.tdc_cal_cycles);
}

/* Get the TDC calibration Cal2-Cal1
 * Note a regular measurement is needed for this to work
 * i.e. something must be happening on the Start & Stop pins
 * On success, return the positive calibration result
 * On failure, return a negative eCalibrationResult error code
 */
int16_t getCalCycles() {
	int16_t res = E_CAL_OK;

	sendOpcode(OPCODE_INIT);

// Turn off NO_CAL_AUTO and EN_FAST_INIT
	writeRegister(0, _configRegisters[0] & ~(CFG0_NO_CAL_AUTO));
	writeRegister(1, CFG_KEEP_DEFAULT_BITMASKS[1] | CFG1_HITIN1_0 * 1// Wait for 1 hit on channel 1
			| CFG1_HITIN2_0 * 0// Wait for 0 hits on channel 2
			| CFG1_HIT1_0 * 1// Calculate 1st Stop Ch1 - Start
			| CFG1_HIT2_0 * 0);

// Set INT on hits
	writeRegister(2, CFG2_EN_INT_HITS);

// Set INT on hits
	writeRegister(2, CFG_KEEP_DEFAULT_BITMASKS[2] | CFG2_EN_INT_HITS);

	sendOpcode(OPCODE_INIT);
	sendOpcode(OPCODE_START_CAL_TDC);

// Calibration data are addressed only after the next regular measurement
// Wait for interrupt meaning hits arrived
	int w = waitForInterrupt(1000000);
	if (w == -1) {
		res = E_CAL_FAIL_NO_HITS;
	}

	if (res == E_CAL_OK) {

		// Set INT on timeout or ALU
		writeRegister(2,
				CFG_KEEP_DEFAULT_BITMASKS[2] | CFG2_EN_INT_ALU
				| CFG2_EN_INT_TDC_TIMEOUT);

		sendOpcode(OPCODE_INIT);

		int w2 = waitForInterrupt(1000000);
		if (w2 == -1) {
			res = E_CAL_FAIL_NO_MEASUREMENT;
		}
	}

	if (readRegister(READ_STAT) & STAT_TIMEOUT_TDC) {
		res = E_CAL_FAIL_TDC_TIMEOUT;
	}

	if (res == E_CAL_OK) {

		/* Read calibration (Cal2-Cal1), and calculate real time  */
		writeRegister(1, CFG_KEEP_DEFAULT_BITMASKS[1] | CFG1_HIT1_0 * 7 // Cal2
				| CFG1_HIT2_0 * 6// Cal1
		);

		int w2 = waitForInterrupt(500000);
		if (w2 == -1) {
			res = E_CAL_FAIL_WAIT;
		} else {
			/* Looks like success */
			res = readUncalibratedResult(getALUPointer() - 1);

			/* Check that value is not garbage to avoid conflict with error codes */
			if (res <= 0) {
				res = E_CAL_FAIL_GARBAGE;
			}

		}

	}

	return res;

}

/* Updates the calibration structure
 * Returns eCalibrationResult with E_OK for success or an error code for failure
 * The _calibration struct is only updated if E_OK returned
 */
eCalibrationResult updateCalibration() {
	eCalibrationResult res = E_CAL_OK;

	tempSaveRegisters();

	attachDefaultInterruptFunc();

// Turn off NO_CAL_AUTO and EN_FAST_INIT
	writeRegister(0,
			(_configRegisters[0] & ~(CFG0_NO_CAL_AUTO)
					& ~(CFG0_ANZ_PER_CALRES_0 * 3))
			| CFG0_ANZ_PER_CALRES_0 * DEFAULT_ANZ_PER_CALRES);

	writeRegister(1, CFG_KEEP_DEFAULT_BITMASKS[1] | CFG1_HITIN1_0 * 1// Wait for 1 hit on channel 1
			| CFG1_HITIN2_0 * 0// Wait for 0 hits on channel 2
			| CFG1_HIT1_0 * 1// Calculate 1st Stop Ch1 - Start
			| CFG1_HIT2_0 * 0);

// Calibrate resonator to 32 kHz clock
	const float res_theor = getResonatorCyclesTheoretical();

	const float res_meas = getResonatorCycles();

// Check failure
	if (res_theor == 0 || res_meas == 0) {
		res = E_CAL_FAIL_RESONATOR;
	}

// Calculate correction factor
	float corrFact = res_theor / res_meas;

// Check reasonable correction factor
	if (corrFact < 0.5 || corrFact > 1.5) {
		res = E_CAL_FAIL_RESONATOR;
	}

// Get TDC calibration cycles
	const int16_t cal = getCalCycles();
// Use of negative return value for error codes (TODO)
	if (cal < 0) {
		res = (eCalibrationResult) cal;
	}

	if (res == E_CAL_OK) {
		_calibration.Tref_theor_ns = HS_CLK_PERIOD_NS;
		_calibration.clock_factor = getClockFactor();
		_calibration.resonator_theor_cycles = res_theor;
		_calibration.resonator_meas_cycles = res_meas;
		_calibration.tdc_cal_cycles = cal;
	}

// Write back original register values
	attachPreviousInterruptFunc();
	tempRestoreRegisters();
	sendOpcode(OPCODE_INIT);

	return res;
}

void tempSaveRegisters() {
	for (int i = 0; i < CFG_REGISTER_MAX; ++i) {
		_configRegistersTemp[i] = _configRegisters[i];
	}
}

void tempRestoreRegisters() {
	for (int i = 0; i < CFG_REGISTER_MAX; ++i) {
		writeRegister(i, _configRegistersTemp[i]);
	}
}

#else

void init(int slaveSelectPin) {
	// Set the internal variable for the SPI slave select.
	_ssPin = slaveSelectPin;
	// Precalculate the conversion factor based on default settings.
	updateConversionFactors();
}

void end() {
	SPI.end();
}

void begin() {
	//Start up SPI
	SPI.begin(_ssPin);
	//Run the SPI clock at 14 MHz (GP22's max is apparently 20 MHz)
	SPI.setClockDivider(_ssPin, 6);
	//Clock polarity = 0, clock phase = 1 (MODE1?)
	SPI.setDataMode(_ssPin, SPI_MODE1);
	//The GP22 sends the most significant bit first
	SPI.setBitOrder(_ssPin, MSBFIRST);
	//Power-on-reset command
	SPI.transfer(_ssPin, 0x50);
	//Transfer the GP22 config registers across
	updateConfig();
}

//Initilise measurement
void measure() {
	SPI.transfer(_ssPin, 0x70);
}

void readStatus() {
	// Get the TDC status from it's stat register
	_status = transfer2B(0xB4, 0x00, 0x00);
}
boolean timedOut() {
	return (_status & 0x0600) > 0 ? TRUE : FALSE;
}
uint8_t getMeasuredHits(Channel channel) {
	switch (channel) {
	case CH1:
		return (_status & 0x0038) >> 3;
	case CH2:
		return (_status & 0x01C0) >> 6;
	}
}
uint8_t getReadPointer() {
	return _status & 0x0007;
}

//Function to read from result registers (as a signed int, as MM1 uses 2's comp)
int32_t readResult(uint8_t resultRegister) {
	// Make sure that we are only reading one of the 4 possibilities
	if (resultRegister < 4 && resultRegister >= 0) {
		// The first read code is 0xB0, so add the register to get the required read code.
		uint8_t readCode = 0xB0 + resultRegister;
		return transfer4B(readCode, 0, 0, 0, 0);
	} else {
		// No such register, return 0;
		return 0;
	}
}

// These are the functions designed to make tranfers quick enough to work
// by sending the opcode and immediatly following with data (using SPI_CONTINUE).
uint8_t transfer1B(uint8_t opcode, uint8_t byte1) {
	Fou rByte
	data = {0};
	SPI.transfer(_ssPin, opcode, SPI_CONTINUE);
	data.bit8[0] = SPI.transfer(_ssPin, byte1);
	return data.bit8[0];
}
uint16_t transfer2B(uint8_t opcode, uint8_t byte1, uint8_t byte2) {
	FourByte data = { 0 };
	SPI.transfer(_ssPin, opcode, SPI_CONTINUE);
	data.bit8[1] = SPI.transfer(_ssPin, byte1, SPI_CONTINUE);
	data.bit8[0] = SPI.transfer(_ssPin, byte2);
	return data.bit16[0];
}
uint32_t transfer4B(uint8_t opcode, uint8_t byte1, uint8_t byte2, uint8_t byte3,
		uint8_t byte4) {
	FourByte data = { 0 };
	SPI.transfer(_ssPin, opcode, SPI_CONTINUE);
	data.bit8[3] = SPI.transfer(_ssPin, byte1, SPI_CONTINUE);
	data.bit8[2] = SPI.transfer(_ssPin, byte2, SPI_CONTINUE);
	data.bit8[1] = SPI.transfer(_ssPin, byte3, SPI_CONTINUE);
	data.bit8[0] = SPI.transfer(_ssPin, byte4);
	return data.bit32;
}

boolean testComms() {
	// The comms can be tested by reading read register 5, which contains the highest 8 bits of config reg 1.
	int test = transfer1B(0xB5, 0);
	// Now test the result is the same as the config register (assuming the registers have been written!).
	if (test == _config[1][0]) {
		return TRUE;
	} else {
		return FALSE;
	}
}

float measConv(int32_t input) {
	// Use the precalculated conversion factor.
	return ((float) input) * _conversionFactorRead;
}

void updateConversionFactors() {
	// This number takes cycles to calculate, so precalculate it.
	// It only needs calculating at startup and on changing the clock settings.

	// Input is a Q16.16 number representation,
	// thus conversion is via multiplication by 2^(-16).
	// The input in also multiples of the clock (4MHz).
	// Output is in microseconds.

	float qConvRead = pow(2.0, -16);    //Q conversion factor
	float qConvDelay = pow(2.0, -5);
	float tRef = (1.0) / (4000000.0); //4MHz clock
	float timeBase = 1000000.0;   //Microseconds
	float N = (float)getClkPreDiv(); // The Clock predivider correction

	_conversionFactorRead = tRef * qConvRead * timeBase * N;
	_conversionFactorDelay = tRef * qConvDelay * timeBase * N;
}

void updateConfig() {
	//Transfer the configuration registers

	// The first config register is 0x80 and the last is 0x86
	// I know, this is a bit cheeky, but I just really wanted to try it...
	for (uint8_t i = 0; i < 7; i++)
		transfer4B((0x80 + i), _config[i][0], _config[i][1], _config[i][2],
				_config[i][3]);
}

void getConfig(uint32_t * arrayToFill) {
	// Fill the array with the config registers, combined into 32 bits

	for (uint8_t i = 0; i < 7; i++)
		arrayToFill[i] = (_config[i][0] << 24) + (_config[i][1] << 16)
				+ (_config[i][2] << 8) + _config[i][3];
}

//// The config setting/getting functions

// Measurement mode selection
void setMeasurementMode(uint8_t mode) {
	uint8_t configPiece = _config[0][2];

	// Reg 0, bit 11, called MESSB2 selects which measurement mode
	// to use. MESSB2 = 0 is mode 1, MESSB2 = 1 is mode 2.

	if (mode == 1) {
		// Set MESSB2 = 0 (Measurement Mode 1)
		bitClear(configPiece, 3);

		// In measurement mode 1, only double res is available.
		// Thus, if the current settings are for quad res (from MM2),
		// change it to double res instead.
		if (isQuadRes())
			setDoubleRes();
	} else if (mode == 2) {
		// Set MESSB2 = 1 (Measurement Mode 2)
		bitSet(configPiece, 3);
	}

	_config[0][2] = configPiece;
}
uint8_t getMeasurementMode() {
	return (_config[0][2] & B00001000) > 0;
}

// This is for the measurement mode 1 clock pre-divider
void setClkPreDiv(uint8_t div) {
	uint8_t configPiece = _config[0][1];

	// The only valid divisions are 1, 2 and 4
	if (div == 1 || div == 2 || div == 4) {
		// Start by clearing the bits (same as setting div to 1)
		bitClear(configPiece, 4);
		bitClear(configPiece, 5);

		// Now set the bits as required for div = 2 or 4
		if (div == 2)
			bitSet(configPiece, 4);
		else if (div == 4)
			bitSet(configPiece, 5);
	}

	_config[0][1] = configPiece;

	// As the clock settings have been changed...
	updateConversionFactors();
}
uint8_t getClkPreDiv() {
	uint8_t divRaw = (_config[0][1] & B00110000) >> 4;

	switch (divRaw) {
	case 1:
		// This means that the div is set do 2
		return 2;
	case 2:
		// This means 4, so does 3 so fall through
	case 3:
		// If we are here, the div is 4
		return 4;
	default:
		// If in doubt (or it was zero), div is 1
		return 1;
	}
}

// The hits of Ch1 are stored in bits 16-18 in register 1
void setExpectedHits(Channel channel, uint8_t hits) {
	// First lets get the bit of the config register we want to modify
	uint8_t configPiece = _config[1][1];

	// Now, we need to set and clear bits as necessary
	// The minimum number of hits is 0, the max is 4.
	if (hits >= 0 & hits <= 4) {
		if (channel == CH1) {
			bitClear(configPiece, 0);
			bitClear(configPiece, 1);
			bitClear(configPiece, 2);

			configPiece += hits;
		} else if (channel == CH2) {
			bitClear(configPiece, 3);
			bitClear(configPiece, 4);
			bitClear(configPiece, 5);

			configPiece += (hits << 3);
		}
	}

	// Now that the peice of the config that needed to be changed has been, lets put it back
	_config[1][1] = configPiece;
	// It is up to the user to update the GP22s registers
	// (in case they want to chain together setting modifications).
	// Also, so this can be called before the begin function is called.
}
uint8_t getExpectedHits(Channel channel) {
	if (channel == CH1) {
		return _config[1][1] & B00000111;
	} else if (channel == CH2) {
		return (_config[1][1] & B00111000) >> 3;
	}
}

void updateALUInstruction(ALUInstruction instruction) {
	// First, update the config registers
	defineHit1Op(instruction.hit1Op);
	defineHit2Op(instruction.hit2Op);
	// Now, we only want to update the relevent config register,
	// as this is quicker than doing everything...
	// The config register with the operators is Reg 1, so update that one!
	transfer4B((0x81), _config[1][0], _config[1][1], _config[1][2],
			_config[1][3]);
}

// Define HIT operators for ALU processing
void defineHit1Op(uint8_t op) {
	uint8_t configPiece = _config[1][0];

	// Clear the first 4 bits of the byte
	for (int i = 0; i < 4; i++)
		bitWrite(configPiece, i, 0);

	// Now write the operator into the first four bits
	configPiece += op;

	// Then write to the config
	_config[1][0] = configPiece;
}
void defineHit2Op(uint8_t op) {
	uint8_t configPiece = _config[1][0];

	// Clear the second 4 bits of the byte
	for (int i = 4; i < 8; i++)
		bitWrite(configPiece, i, 0);

	// Now write the op into the top 4 bits
	configPiece += (op << 4);

	// Then write to the config
	_config[1][0] = configPiece;
}
uint8_t getHit1Op() {
	return _config[1][0] & B00001111;
}
uint8_t getHit2Op() {
	return (_config[1][0] & B11110000) >> 4;
}

// Define the edge sensitivities of the inputs
void setEdgeSensitivity(uint8_t start, uint8_t stop1, uint8_t stop2) {
	uint8_t reg0p2 = _config[0][2];
	uint8_t reg2p0 = _config[2][0];

	// Deal with the start, which can only be rising or falling, not both.
	if (start == 0 || start == 1) {
		bitWrite(reg0p2, 0, start);
	}

	// Stop 1 and 2 can be rising, falling or both.
	if (stop1 == 0 || stop1 == 1) {
		// Deal with rising or falling
		bitWrite(reg0p2, 1, stop1);
	} else if (stop1 == 2) {
		// Deal with both, i.e. set to rising sensitivity
		// and make the stop trigger on both edges.
		bitClear(reg0p2, 1);
		bitSet(reg2p0, 3);
	}
	// Repeat for stop2
	if (stop2 == 0 || stop2 == 1) {
		bitWrite(reg0p2, 2, stop2);
	} else if (stop2 == 2) {
		bitClear(reg0p2, 2);
		bitSet(reg2p0, 4);
	}

	_config[0][2] = reg0p2;
	_config[2][0] = reg2p0;
}

void setSingleRes() {
	uint8_t configPiece = _config[6][2];

	bitClear(configPiece, 4);
	bitClear(configPiece, 5);

	_config[6][2] = configPiece;
}
boolean isSingleRes() {
	return !isDoubleRes() && !isQuadRes() ? TRUE : FALSE;
}
void setDoubleRes() {
	uint8_t configPiece = _config[6][2];

	bitSet(configPiece, 4);
	bitClear(configPiece, 5);

	_config[6][2] = configPiece;
}
boolean isDoubleRes() {
	return (_config[6][2] & B00010000) > 0 ? TRUE : FALSE;
}
void setQuadRes() {
	// Quad res is only available in measurement mode 2.
	if (getMeasurementMode() == 2) {
		uint8_t configPiece = _config[6][2];

		bitSet(configPiece, 5);
		bitClear(configPiece, 4);

		_config[6][2] = configPiece;
	}
}
boolean isQuadRes() {
	return (_config[6][2] & B00100000) > 0 ? TRUE : FALSE;
}
void setAutoCalcOn(boolean on) {
	uint8_t configPiece = _config[3][0];

	if (on)
		bitSet(configPiece, 7);
	else
		bitClear(configPiece, 7);

	_config[3][0] = configPiece;
}
boolean isAutoCalcOn() {
	return (_config[3][0] & B10000000) > 0 ? TRUE : FALSE;
}

void setFirstWaveMode(boolean on) {
	// First wave on/off is bit 30 of reg 3
	uint8_t configPiece = _config[3][0];

	bitSet(configPiece, 6);

	_config[3][0] = configPiece;
}
boolean isFirstWaveMode() {
	return (_config[3][0] & B01000000) > 0 ? TRUE : FALSE;
}

void setFirstWaveDelays(uint8_t stop1, uint8_t stop2, uint8_t stop3) {
	// Grab the relevant bytes from Reg 3 to modify
	FourByte configPiece = { 0 };
	configPiece.bit8[3] = _config[3][0];
	configPiece.bit8[2] = _config[3][1];
	configPiece.bit8[1] = _config[3][2];
	configPiece.bit8[0] = _config[3][3];

	// DELREL1 is bits 8-13
	// DELREL2 is bits 14-19
	// DELREL3 is bits 20-25

	/// First sort out DELREL3
	// To make things easier, combine the top two bytes
	uint16_t topBytes = configPiece.bit16[1];
	// Now DELREL3 is bits 4-9 of topBytes
	// First clear bits 4-9.
	topBytes = topBytes ^ (topBytes & 0x03F0);
	// Now add the setting to those bits
	topBytes = topBytes + ((uint16_t) stop3 << 4);
	// Now we can write this back
	configPiece.bit16[1] = topBytes;

	/// Now DELREL2
	// This is tricky as it is spread accross the top and bottom half.
	// So, we grab the middle bytes.
	uint16_t middleBytes = ((uint16_t) configPiece.bit8[2] << 8)
			+ configPiece.bit8[1];
	// Now DELREL2 is in bits 6-11 of middleBytes
	// So clear the bits that need to be modified
	middleBytes = middleBytes ^ (middleBytes & 0x0FC0);
	// Now add the new settings
	middleBytes = middleBytes + ((uint16_t) stop2 << 6);
	// Now we can put the bytes back
	configPiece.bit8[1] = (uint8_t) middleBytes;
	configPiece.bit8[2] = (uint8_t) (middleBytes >> 8);

	// Now DELREL1
	// This should be easy, as it's all in one byte
	uint8_t byte1 = configPiece.bit8[1];
	// So DELREL1 is in bits 0-5 of byte1
	byte1 = byte1 ^ (byte1 & B00111111);
	byte1 = byte1 + stop1;
	// Now we can write this back
	configPiece.bit8[1] = byte1;
}

void setPulseWidthMeasOn(boolean on) {
	// DIS_PW, disable pusle width measurement is contained in bit 16, Reg 4
	uint8_t configPiece = _config[4][1];

	// bit 16 = 0 => Pulse width measurement enabled
	// bit 16 = 1 => disabled
	if (on)
		bitClear(configPiece, 0);
	else
		bitSet(configPiece, 0);

	_config[4][1] = configPiece;
}
boolean isPulseWidthMeasOn() {
	return (_config[4][1] & B00000001) == 0 ? TRUE : FALSE;
}

void setFirstWaveRisingEdge(boolean on) {
	uint8_t configPiece = _config[4][2];

	// bit 15 = 0 => Rising (positive) edge sensitive
	// bit 15 = 1 => Falling (negative) edge sensitive
	if (on)
		bitClear(configPiece, 7);
	else
		bitSet(configPiece, 7);

	_config[4][2] = configPiece;
}
boolean isFirstWaveRisingEdge() {
	return (_config[4][2] & B10000000) > 0 ? TRUE : FALSE;
}

void setFirstWaveOffset(int8_t offset) {
	// There are three seperate settings that need to be configured.
	// First is the OFFS setting, in bits 8-12 of REG4.
	// The OFFS setting is a number between -16 and +15 in twos complement.
	// The next two settings are for adding on an addition +- 20 mV.
	/// First lets grab the byte in question
	uint8_t configPiece = _config[4][2];

	// Now we need to check if we need the extra ranges
	if (offset > 15) {
		// We are greater than the offset allows, so we need the extra +20 range
		offset -= 20;
		// Offset is now what it was, minus what the extra range gives
		bitSet(configPiece, 6);
		bitClear(configPiece, 5);
	} else if (offset < -16) {
		// The offset is less than it can be, so we need the extra -20 range
		offset += 20;
		// Now offset is what it was, add the extra range amount
		bitSet(configPiece, 5);
		bitClear(configPiece, 6);
	} else {
		// We seem to have an offset that is within range, so turn off the extra ranges
		bitClear(configPiece, 5);
		bitClear(configPiece, 6);
	}

	// Now we need to load the offset into bits 0-4 of the config byte.
	// It needs to be loaded as twos complement.
	// First lets clear the relevent bits
	configPiece = configPiece ^ (configPiece & B00011111);
	if ((offset > 0) && (offset <= 15)) {
		// If the number is positive, then we can just add it normally
		configPiece += offset;
	} else if ((offset < 0) && (offset >= -16)) {
		// This means we are in the negative part, so this is tricky.
		// Need to do a 5 bit 2s complement conversion.
		// First start with 5 bits all 1.
		uint8_t twosComp = 31;
		// Now add one to the offset, and then add it to the 1s
		twosComp = twosComp + (offset + 1);
		// Now bits 0-4 should contain the correct 5 bit 2s complement number
		configPiece += twosComp;
	}

	// So now that the config is set, put it back in place
	_config[4][2] = configPiece;
}
int8_t getFirstWaveOffset() {
	// First grab the relevant byte
	uint8_t configPiece = _config[4][2];
	// Next we need to grab the twos complement offset number
	uint8_t twosComp = configPiece & B00011111;
	// Prepare a variable to store the offset
	int8_t offset = 0;
	// Now parse the twos complement number
	if (twosComp > 15) {
		// If this number is greater than 15, then it must be negative
		offset = -1 * ((~twosComp) + 1);
	} else {
		// If it is less than that, then it is positive and nothing needs to be done
		offset = twosComp;
	}
	// Now we need to deal with any of the range additions
	if ((configPiece & B00100000) > 0) {
		// OFFSRNG1 is enabled, so take 20 from the offset
		offset -= 20;
	} else if ((configPiece & B01000000) > 0) {
		// OFFSRNG2 is enabled, so add 20 to the offset
		offset += 20;
	}

	return offset;
}
#endif
