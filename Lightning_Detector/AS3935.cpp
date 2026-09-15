/*
  This is a library for the ASM AS3935 Franklin Lightning Detector
  By: Elias Santistevan
  SparkFun Electronics
  Date: January, 2019
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!

  Modifié pour:
  - supprimer l'I2C et forcer l'utilisation du bus SPI hardware (gain de place)
*/

#include "AS3935.h"

// Default constructor, to be used with SPI
AS3935::AS3935()
{
}

bool AS3935::begin(uint8_t user_CSPin, uint32_t spiPortSpeed)
{
    // Startup time requires 2ms for the LCO and 2ms more for the RC oscillators
    // which occurs only after the LCO settles. See "Timing" under "Electrical
    // Characteristics" in the datasheet.
    delay(4);
    _spiPortSpeed = spiPortSpeed; // Make sure it's not 500kHz or it will cause feedback with antekknna.
    _cs = user_CSPin;
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH); // Deselect the Lightning Detector.

    mySpiSettings = SPISettings(spiPortSpeed, MSBFIRST, SPI_MODE1);

    return true;
}
// REG0x00, bit[0], manufacturer default: 0.
// The product consumes 1-2uA while powered down. If the board is powered down
// the the TRCO will need to be recalibrated: REG0x08[5] = 1, wait 2 ms, REG0x08[5] = 0.
// SPI and I-squared-C remain active when the chip is powered down.
void AS3935::powerDown()
{
    _writeRegister(AFE_GAIN, POWER_MASK, 1, 0);
}

// REG0x3A bit[7].
// This register holds the state of the timer RC oscillator (TRCO),
// after it has been calibrated. The TRCO in this case needs to be recalibrated
// after power down. The following function wakes the IC, sends the "Direct Command" to
// CALIB_RCO register REG0x3D, waits 2ms and then checks that it has been successfully
// calibrated. Note that I-squared-C and SPI are active during power down.
bool AS3935::wakeUp()
{

    _writeRegister(AFE_GAIN, POWER_MASK, 0, 0); // Set the power down bit to zero to wake it up

    if (calibrateOsc())
        return true;
    else
        return false;
}

// REG0x00, bits [5:1], manufacturer default: 10010 (INDOOR).
// This function changes toggles the chip's settings for Indoors and Outdoors.
void AS3935::setIndoorOutdoor(uint8_t _setting)
{
    if (((_setting != INDOOR) && (_setting != OUTDOOR)))
        return;

    _writeRegister(AFE_GAIN, GAIN_MASK, _setting, 1);
}

// REG0x00, bits [5:1], manufacturer default: 10010 (INDOOR).
// This function returns the indoor/outdoor settting.
uint8_t AS3935::readIndoorOutdoor()
{

    uint8_t regVal = readRegister(AFE_GAIN);
    return ((regVal &= ~GAIN_MASK) >> 1);
}

// REG0x01, bits[3:0], manufacturer default: 0010 (2).
// This setting determines the threshold for events that trigger the
// IRQ Pin.
void AS3935::watchdogThreshold(uint8_t _sensitivity)
{
    if (_sensitivity > 10) // 10 is the max sensitivity setting
        return;

    _writeRegister(THRESHOLD, THRESH_MASK, _sensitivity, 0);
}

// REG0x01, bits[3:0], manufacturer default: 0010 (2).
// This function returns the threshold for events that trigger the
// IRQ Pin.
uint8_t AS3935::readWatchdogThreshold()
{

    uint8_t regVal = readRegister(THRESHOLD);
    return (regVal &= (~THRESH_MASK));
}

// REG0x01, bits [6:4], manufacturer default: 010 (2).
// The noise floor level is compared to a known reference voltage. If this
// level is exceeded the chip will issue an interrupt to the IRQ pin,
// broadcasting that it can not operate properly due to noise (INT_NH).
// Check datasheet for specific noise level tolerances when setting this register.
void AS3935::setNoiseLevel(uint8_t _floor)
{
    if (_floor > 7)
        return;

    _writeRegister(THRESHOLD, NOISE_FLOOR_MASK, _floor, 4);
}

// REG0x01, bits [6:4], manufacturer default: 010 (2).
// This function will return the set noise level threshold: default is 2.
uint8_t AS3935::readNoiseLevel()
{

    uint8_t regVal = readRegister(THRESHOLD);
    return (regVal & ~NOISE_FLOOR_MASK) >> 4;
}

// REG0x02, bits [3:0], manufacturer default: 0010 (2).
// This setting, like the watchdog threshold, can help determine between false
// events and actual lightning. The shape of the spike is analyzed during the
// chip's signal validation routine. Increasing this value increases robustness
// at the cost of sensitivity to distant events.
void AS3935::spikeRejection(uint8_t _spSensitivity)
{
    if (_spSensitivity > 15)
        return;

    _writeRegister(LIGHTNING_REG, SPIKE_MASK, _spSensitivity, 0);
}

// REG0x02, bits [3:0], manufacturer default: 0010 (2).
// This function returns the value of the spike rejection register. This value
// helps to differentiate between events and acutal lightning, by analyzing the
// shape of the spike during  chip's signal validation routine.
// Increasing this value increases robustness at the cost of sensitivity to distant events.
uint8_t AS3935::readSpikeRejection()
{

    uint8_t regVal = readRegister(LIGHTNING_REG);
    return (regVal &= ~SPIKE_MASK);
}
// REG0x02, bits [5:4], manufacturer default: 0 (single lightning strike).
// The number of lightning events before IRQ is set high. 15 minutes is The
// window of time before the number of detected lightning events is reset.
// The number of lightning strikes can be set to 1,5,9, or 16.
void AS3935::lightningThreshold(uint8_t _strikes)
{

    uint8_t bits;

    if (_strikes == 1)
        bits = 0;
    else if (_strikes == 5)
        bits = 1;
    else if (_strikes == 9)
        bits = 2;
    else if (_strikes == 16)
        bits = 3;
    else
        return;

    _writeRegister(LIGHTNING_REG, LIGHT_MASK, bits, 4);
}

// REG0x02, bits [5:4], manufacturer default: 0 (single lightning strike).
// This function will return the number of lightning strikes must strike within
// a 15 minute window before it triggers an event on the IRQ pin. Default is 1.
uint8_t AS3935::readLightningThreshold()
{
    uint8_t regVal = readRegister(LIGHTNING_REG);

    regVal &= ~LIGHT_MASK;
    regVal >>= 4; // Front of the line.

    if (regVal == 0)
        return 1;
    else if (regVal == 1)
        return 5;
    else if (regVal == 2)
        return 9;
    else if (regVal == 3)
        return 16;
    else
        return regVal;
}

// REG0x02, bit [6], manufacturer default: 1.
// This register clears the number of lightning strikes that has been read in
// the last 15 minute block.
void AS3935::clearStatistics(bool _clearStat)
{
    if (_clearStat != true)
        return;
    // Write high, then low, then high to clear.
    _writeRegister(LIGHTNING_REG, STAT_MASK, 1, 6);
    _writeRegister(LIGHTNING_REG, STAT_MASK, 0, 6);
    _writeRegister(LIGHTNING_REG, STAT_MASK, 1, 6);
}

// REG0x03, bits [3:0], manufacturer default: 0.
// When there is an event that exceeds the watchdog threshold, the register is written
// with the type of event. This consists of two messages: INT_D (disturber detected) and
// INT_L (Lightning detected). A third interrupt INT_NH (noise level too HIGH)
// indicates that the noise level has been exceeded and will persist until the
// noise has ended. Events are active HIGH. There is a one second window of time to
// read the interrupt register after lightning is detected, and 1.5 after
// disturber.
uint8_t AS3935::readInterruptReg()
{
    // A 2ms delay is added to allow for the memory register to be populated
    // after the interrupt pin goes HIGH. See "Interrupt Management" in
    // datasheet.
    delay(2);

    uint8_t _interValue;
    _interValue = readRegister(INT_MASK_ANT);
    _interValue &= INT_MASK;

    return (_interValue);
}

// REG0x03, bit [5], manufacturere default: 0.
// This setting will change whether or not disturbers trigger the IRQ Pin.
void AS3935::maskDisturber(bool _state)
{
    _writeRegister(INT_MASK_ANT, DISTURB_MASK, _state, 5);
}

// REG0x03, bit [5], manufacturere default: 0.
// This setting will return whether or not disturbers trigger the IRQ Pin.
uint8_t AS3935::readMaskDisturber()
{

    uint8_t regVal = readRegister(INT_MASK_ANT);
    return (regVal &= ~DISTURB_MASK) >> 5;
}

// REG0x03, bit [7:6], manufacturer default: 0 (16 division ratio).
// The antenna is designed to resonate at 500kHz and so can be tuned with the
// following setting. The accuracy of the antenna must be within 3.5 percent of
// that value for proper signal validation and distance estimation.
void AS3935::changeDivRatio(uint8_t _divisionRatio)
{

    uint8_t bits;

    if (_divisionRatio == 16)
        bits = 0;
    else if (_divisionRatio == 32)
        bits = 1;
    else if (_divisionRatio == 64)
        bits = 2;
    else if (_divisionRatio == 128)
        bits = 3;
    else
        return;

    _writeRegister(INT_MASK_ANT, DIV_MASK, bits, 6);
}

// REG0x03, bit [7:6], manufacturer default: 0 (16 division ratio).
// This function returns the current division ratio of the resonance frequency.
// The antenna resonance frequency should be within 3.5 percent of 500kHz, and
// so when modifying the resonance frequency with the internal capacitors
// (tuneCap()) it's important to keep in mind that the displayed frequency on
// the IRQ pin is divided by this number.
uint8_t AS3935::readDivRatio()
{

    uint8_t regVal = readRegister(INT_MASK_ANT);
    regVal &= ~DIV_MASK;
    regVal >>= 6; // Front of the line.

    if (regVal == 0)
        return 16;
    else if (regVal == 1)
        return 32;
    else if (regVal == 2)
        return 64;
    else if (regVal == 3)
        return 128;
    else
        return UNKNOWN_ERROR;
}

// REG0x07, bit [5:0], manufacturer default: 0.
// This register holds the distance to the front of the storm and not the
// distance to a lightning strike.
uint8_t AS3935::distanceToStorm()
{

    uint8_t _dist = readRegister(DISTANCE);
    _dist &= DISTANCE_MASK;
    return (_dist);
}

// REG0x08, bits [5,6,7], manufacturer default: 0.
// This will send the frequency of the oscillators to the IRQ pin.
//  _osc 1, bit[5] = TRCO - System RCO at 32.768kHz
//  _osc 2, bit[6] = SRCO - Timer RCO Oscillators 1.1MHz
//  _osc 3, bit[7] = LCO - Frequency of the Antenna
//void AS3935::displayOscillator(bool _state, uint8_t _osc)
void AS3935::displayOscillator(uint8_t _osc)
{
    if (_osc > 3)
        return;

    _writeRegister(FREQ_DISP_IRQ, OSC_MASK, _osc, 5);
/*
    if (_state == true)
    {
        if (_osc == 1)
            _writeRegister(FREQ_DISP_IRQ, OSC_MASK, 1, 5);
        if (_osc == 2)
            _writeRegister(FREQ_DISP_IRQ, OSC_MASK, 1, 6);
        if (_osc == 3)
            _writeRegister(FREQ_DISP_IRQ, OSC_MASK, 1, 7);
    }

    if (_state == false)
    {
        if (_osc == 1)
            _writeRegister(FREQ_DISP_IRQ, OSC_MASK, 0, 5); // Demonstrative
        if (_osc == 2)
            _writeRegister(FREQ_DISP_IRQ, OSC_MASK, 0, 6);
        if (_osc == 3)
            _writeRegister(FREQ_DISP_IRQ, OSC_MASK, 0, 7);
    }
*/
}

// REG0x08, bits [3:0], manufacturer default: 0.
// This setting will add capacitance to the series RLC antenna on the product
// to help tune its resonance. The datasheet specifies being within 3.5 percent
// of 500kHz to get optimal lightning detection and distance sensing.
// It's possible to add up to 120pF in steps of 8pF to the antenna.
void AS3935::tuneCap(uint8_t farad)
{
    if (farad > 15)
        return;

    _writeRegister(FREQ_DISP_IRQ, CAP_MASK, farad, 0);
}

// REG0x08, bits [3:0], manufacturer default: 0.
// This setting will return the capacitance of the internal capacitors. It will
// return a value from one to 15. It need to be multiplied by 8 to have value in pF.
uint8_t AS3935::readTuneCap()
{

    uint8_t regVal = readRegister(FREQ_DISP_IRQ);
    return (regVal &= ~CAP_MASK);
}

// LSB =  REG0x04, bits[7:0]
// MSB =  REG0x05, bits[7:0]
// MMSB = REG0x06, bits[4:0]
// This returns a 20 bit value that is the 'energy' of the lightning strike.
// According to the datasheet this is only a pure value that doesn't have any
// physical meaning.
uint32_t AS3935::lightningEnergy()
{

    uint32_t _pureLight = readRegister(ENERGY_LIGHT_MMSB);
    _pureLight &= ENERGY_MASK;
    _pureLight <<= 8;
    _pureLight |= readRegister(ENERGY_LIGHT_MSB);
    _pureLight <<= 8;
    _pureLight |= readRegister(ENERGY_LIGHT_LSB);
    return _pureLight;
}

// REG0x3D, bits[7:0]
// This function calibrates both internal oscillators The oscillators are tuned
// based on the resonance frequency of the antenna and so it should be trimmed
// before the calibration is done.
//
// Returns true if calibration succedded.
bool AS3935::calibrateOsc()
{
    // Send command to calibrate the oscillators
    _writeRegister(CALIB_RCO, WIPE_ALL, DIRECT_COMMAND, 0);

    // This procedure is specified in the datasheet
    displayOscillator(2);
    delay(2);
    displayOscillator(0);

    // Check they were calibrated successfully.
    uint8_t regValSrco = readRegister(CALIB_SRCO);
    uint8_t regValTrco = readRegister(CALIB_TRCO);

    regValSrco &= CALIB_MASK;
    regValSrco >>= 6;
    regValTrco &= CALIB_MASK;
    regValTrco >>= 6;

    return (!regValSrco && !regValTrco);
}

// REG0x3C, bits[7:0]
// This function resets all settings to their default values.
void AS3935::resetSettings()
{
    _writeRegister(RESET_LIGHT, WIPE_ALL, DIRECT_COMMAND, 0);
}

// This function handles all I2C write commands. It takes the register to write
// to, then will mask the part of the register that coincides with the
// given register, and then write the given bits to the register starting at
// the given start position._spiPort->_i2cWrite
void AS3935::_writeRegister(uint8_t _wReg, uint8_t _mask, uint8_t _bits, uint8_t _startPosition)
{
    _spiWrite = readRegister(_wReg);       // Get the current value of the register
    _spiWrite &= _mask;                     // Mask the position we want to write to
    _spiWrite |= (_bits << _startPosition); // Write the given bits to the variable
    writeRegister(_wReg, _spiWrite);
    // SPI.beginTransaction(mySpiSettings);
    // digitalWrite(_cs, LOW);        // Start communication
    // SPI.transfer(_wReg);     // Start write command at given register
    // SPI.transfer(_spiWrite); // Write to register
    // digitalWrite(_cs, HIGH);       // End communcation
    // SPI.endTransaction();
}

void AS3935::writeRegister(uint8_t _reg, uint8_t _value)
{
  _reg = _reg & 0x3f;
  // _reg |= 0x00;    // Register OR'ed with SPI write command.
  SPI.beginTransaction(mySpiSettings);
  digitalWrite(_cs, LOW); // Start communication
  SPI.transfer(_reg);    // Start write command at given register
  SPI.transfer(_value);      // Write to register
  digitalWrite(_cs, HIGH);// End communcation
  SPI.endTransaction();
}

// This function reads the given register.
uint8_t AS3935::readRegister(uint8_t _reg)
{
    SPI.beginTransaction(mySpiSettings);
    digitalWrite(_cs, LOW);           // Start communication.
    SPI.transfer(_reg |= SPI_READ_M); // Register OR'ed with SPI read command.
    _regValue = SPI.transfer(0);      // Get data from register.
    // According to datsheet, the chip select must be written HIGH, LOW, HIGH
    // to correctly end the READ command.
    digitalWrite(_cs, HIGH);
    digitalWrite(_cs, LOW);
    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
    return (_regValue);
}

