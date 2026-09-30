/******************************************************************************
Walrus.cpp
Library for Walrus pressure and temperature sensor, made by Northern Widget LLC.
Based off of the TP-Downhole
Bobby Schulz @ Northern Widget LLC
5/9/2018
Hardware info located at:
https://github.com/NorthernWidget-Skunkworks/Project-Walrus

Distributed as-is; no warranty is given.
******************************************************************************/

#include <Wire.h> // Wire library is used for I2C
#include "Walrus_I2C.h"

Walrus::Walrus()
{
}

bool Walrus::begin(uint8_t Address_)
{
    //Page 0 gates: Schema 1, the name "Walrus", firmware patch >= WALRUS_FW_MIN_PATCH.
    return _dev.begin(Address_, "Walrus", WALRUS_FW_MIN_PATCH);
}

bool Walrus::updateMeasurements(uint8_t component)
{
    bool doMS = component & MS5803, doMCP = component & MCP9808;
    if(doMS) { _pressureReadings.reset(); _tempMS5803Readings.reset(); }
    if(doMCP) _tempExtReadings.reset();
    if(doMS && doMCP && _pressureCfg.n <= 1 && _temperatureCfg.n <= 1) {
        //One reading of everything: both chips in one trigger, one 24-byte read
        //covering Blocks 1 to 3, so nothing can straddle a rewrite.
        _dev.resetBatch();
        uint8_t d[WALRUS_DATA_BYTES];
        if(_dev.takeReading(ALL) && readPage2(d)) {
            readMS5803(d);
            readMCP9808(d + 8);
        }
    }
    else {
        //Per chip group: N readings each, appended to the arrays; a chip that
        //reports absent (not answering / self-test failed) stops its batch.
        if(doMS) _dev.takeReadings(MS5803, _pressureCfg.n, [this] { return updatePressure(); });
        if(doMCP) _dev.takeReadings(MCP9808, _temperatureCfg.n, [this] { return updateTemperature(); });
    }
    summarise(component);
    bool ok = true;
    if(doMS) ok = ok && _pressureReadings.count() > 0;
    if(doMCP) ok = ok && _tempExtReadings.count() > 0;
    return ok;
}

bool Walrus::updatePressure()
{
    //Blocks 1 to 3 in one transaction: the device rewrites its data atomically,
    //so a single read cannot straddle a reading, where two reads could.
    uint8_t d[WALRUS_DATA_BYTES];
    if(!_dev.takeReading(MS5803) || !readPage2(d)) return false;
    return readMS5803(d);
}

bool Walrus::updateTemperature()
{
    uint8_t d[2];
    if(!_dev.takeReading(MCP9808) || !_dev.readData(TEMP_EXT, d, 2)) return false;
    return readMCP9808(d);
}

bool Walrus::readPage2(uint8_t (&d)[WALRUS_DATA_BYTES])
{
    //Blocks 1 and 2 first, then Block 3: sixteen bytes and eight, because the
    //Walrus cannot clock out more than sixteen at a time.
    if(!_dev.readData(PRES_REG, d, WALRUS_READ_MAX)) return false;
    return _dev.readData(ADC_REG, d + WALRUS_READ_MAX, WALRUS_DATA_BYTES - WALRUS_READ_MAX);
}

bool Walrus::readMS5803(const uint8_t (&d)[WALRUS_DATA_BYTES])
{
    if(_dev.faulted(0)) return false;           //MS5803: pressure int32 uBar, temperature int16 0.01 C
    int32_t p = (int32_t)((uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24));
    _pressureReadings.append(p);
    _tempMS5803Readings.append((int16_t)(d[4] | (d[5] << 8)));
    //Block 3 at 0x58 is 16 bytes past 0x48: the conversions the rest came from.
    _pressureAdc    = (uint32_t)d[16] | ((uint32_t)d[17] << 8) | ((uint32_t)d[18] << 16) | ((uint32_t)d[19] << 24);
    _temperatureAdc = (uint32_t)d[20] | ((uint32_t)d[21] << 8) | ((uint32_t)d[22] << 16) | ((uint32_t)d[23] << 24);
    return true;
}

bool Walrus::readMCP9808(uint8_t* d)
{
    if(_dev.faulted(1)) return false;           //MCP9808: external temperature int16 0.01 C
    _tempExtReadings.append((int16_t)(d[0] | (d[1] << 8)));
    return true;
}

void Walrus::summarise(uint8_t component)
{
    //Means over the readings taken, scaled from the register units; NW_ERROR when none.
    if(component & MS5803) {
        _pressure   = nwScaled(_pressureReadings.mean(),   1000.0);
        _tempMS5803 = nwScaled(_tempMS5803Readings.mean(), 100.0);
    }
    if(component & MCP9808) {
        _tempExt = nwScaled(_tempExtReadings.mean(), 100.0);
    }
}

uint16_t Walrus::setPressureReadings(uint16_t n)    { return _pressureCfg.set(n, WALRUS_PRESSURE_CAPACITY); }
uint16_t Walrus::setTemperatureReadings(uint16_t n) { return _temperatureCfg.set(n, WALRUS_TEMPERATURE_CAPACITY); }
void     Walrus::setPressureStats(bool enable)      { _pressureCfg.stats = enable; }
void     Walrus::setTemperatureStats(bool enable)   { _temperatureCfg.stats = enable; }
uint16_t Walrus::getPressureCount()               { return _pressureReadings.count(); }
uint16_t Walrus::getTemperatureCount()            { return _tempExtReadings.count(); }

//Statistics are computed from the arrays each call (NW_Readings), in the
//register units, then scaled: uBar -> mBar, 0.01 C -> C. NW_ERROR when empty.
float Walrus::getPressureMean()   { return nwScaled(_pressureReadings.mean(),   1000.0); }
float Walrus::getPressureStd()    { return nwScaled(_pressureReadings.std(),    1000.0); }
float Walrus::getPressureSterr()  { return nwScaled(_pressureReadings.sterr(),  1000.0); }
float Walrus::getPressureMedian() { return nwScaled(_pressureReadings.median(), 1000.0); }
float Walrus::getTemperatureMean()   { return nwScaled(_tempExtReadings.mean(),   100.0); }
float Walrus::getTemperatureStd()    { return nwScaled(_tempExtReadings.std(),    100.0); }
float Walrus::getTemperatureSterr()  { return nwScaled(_tempExtReadings.sterr(),  100.0); }
float Walrus::getTemperatureMedian() { return nwScaled(_tempExtReadings.median(), 100.0); }
float Walrus::getMS5803TemperatureMean()   { return nwScaled(_tempMS5803Readings.mean(),   100.0); }
float Walrus::getMS5803TemperatureStd()    { return nwScaled(_tempMS5803Readings.std(),    100.0); }
float Walrus::getMS5803TemperatureSterr()  { return nwScaled(_tempMS5803Readings.sterr(),  100.0); }
float Walrus::getMS5803TemperatureMedian() { return nwScaled(_tempMS5803Readings.median(), 100.0); }

uint32_t Walrus::getPressureADC()    { return _pressureAdc; }
uint32_t Walrus::getTemperatureADC() { return _temperatureAdc; }
void     Walrus::setADCColumns(bool enable) { _adcColumns = enable; }

float Walrus::getTemperature()       { return _tempExt; }      //the medium: the measurement
float Walrus::getMS5803Temperature() { return _tempMS5803; }   //the die: what compensates the pressure

float Walrus::getPressure()
{
    return _pressure;
}

bool Walrus::newData()  //Checks for updated/valid data
{
    return _dev.ready();
}

bool    Walrus::ready()            { return _dev.ready(); }
bool    Walrus::newReading()       { return _dev.newReading(); }
bool    Walrus::requestReading()   { return _dev.requestReading(0x03); }
bool    Walrus::faulted(uint8_t chip) { return _dev.faulted(chip); }
bool    Walrus::anyFault()         { return _dev.anyFault(); }
uint8_t Walrus::reportChip()        { return _dev.reportChip(); }
uint8_t Walrus::reportKind()        { return _dev.reportKind(); }
String  Walrus::beginFailure()     { return _dev.beginFailure(); }
uint8_t Walrus::getHardwareMajor() { return _dev.hardwareMajor(); }
uint8_t Walrus::getHardwareMinor() { return _dev.hardwareMinor(); }
uint8_t Walrus::getFirmwareVersion() { return _dev.firmwareVersion(); }

size_t Walrus::printReport(Print& out)
{
    //The chip names are Walrus's own (the spec's chip table); NW_Report prints the rest.
    static const char* const chips[] = {"MS5803", "MCP9808"};
    return _dev.report().print(out, chips, 2);
}

size_t Walrus::printStatus(Print& out, bool boot)
{
    static const char* const chips[] = {"MS5803", "MCP9808"};
    return _dev.printSnapshot(out, chips, 2, boot, WALRUS_LIBRARY_VERSION, WALRUS_LIBRARY_COMMIT);
}

bool    Walrus::reportIsFault()   { return _dev.report().isFault(); }
uint8_t Walrus::bootReportKind()  { return _dev.bootReport().kind(); }
void    Walrus::clearBootReport() { _dev.clearBootReport(); }

String Walrus::reportNote()
{
    //One word for a data-table note: the chip, then the kind ("MS5803NotAnswering").
    static const char* const chips[] = {"MS5803", "MCP9808"};
    return _dev.report().note(chips, 2);
}

String Walrus::getHeader()
{
    String h = "Pressure [mBar],"; //return header string
    if(_pressureCfg.columns()) h += "Pressure std [mBar],Pressure sterr [mBar],";
    h += "Temp DH [C],";
    if(_temperatureCfg.columns()) h += "Temp DH std [C],Temp DH sterr [C],";
    h += "Temp DHt [C],";
    if(_pressureCfg.columns()) h += "Temp DHt std [C],Temp DHt sterr [C],";
    if(_adcColumns) h += "MS5803 D1 [1],MS5803 D2 [1],";
    return h;
}

String Walrus::getString()
{
    updateMeasurements();                           //NW_ERROR (-9999) where a reading failed
    String s = String(getPressure()) + ",";
    if(_pressureCfg.columns()) s += String(getPressureStd()) + "," + String(getPressureSterr()) + ",";
    s += String(getTemperature()) + ",";
    if(_temperatureCfg.columns()) s += String(getTemperatureStd()) + "," + String(getTemperatureSterr()) + ",";
    s += String(getMS5803Temperature()) + ",";
    if(_pressureCfg.columns()) s += String(getMS5803TemperatureStd()) + "," + String(getMS5803TemperatureSterr()) + ",";
    if(_adcColumns) s += String(getPressureADC()) + "," + String(getTemperatureADC()) + ",";
    return s;
}

//The reading interface: one reading per logReading(), printed as it is taken.
void Walrus::beginReadings(uint8_t component, uint16_t n)
{
    _component = component;
    if(component & MS5803) { _pressureReadings.reset(); _tempMS5803Readings.reset(); }
    if(component & MCP9808) _tempExtReadings.reset();
    _dev.beginBatch(n);
}

void Walrus::endReadings()
{
    //No cleanup required currently
}

size_t Walrus::printHeader(Print& out)
{
    size_t n = 0;
    if(_component & MS5803) n += out.print("Pressure [mBar],Temp DHt [C],");
    if(_component & MCP9808) n += out.print("Temp DH [C],");
    return n;
}

size_t Walrus::printReading(Print& out)
{
    size_t n = 0;
    if(_component & MS5803) { n += out.print(_pressure); n += out.print(','); n += out.print(_tempMS5803); n += out.print(','); }
    if(_component & MCP9808) { n += out.print(_tempExt); n += out.print(','); }
    return n;
}

size_t Walrus::logReading(Print& out)
{
    //One acquisition per chip group selected, then the values just taken.
    if(_component & MS5803) {
        _pressure = _tempMS5803 = NW_ERROR;
        if(updatePressure()) {
            _pressure = _pressureReadings.last() / 1000.0;
            _tempMS5803 = _tempMS5803Readings.last() / 100.0;
        }
    }
    if(_component & MCP9808) {
        _tempExt = NW_ERROR;
        if(updateTemperature()) _tempExt = _tempExtReadings.last() / 100.0;
    }
    return printReading(out);
}
