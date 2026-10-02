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

//The column names, from NW_Core's generated table. They are aliased here only
//to keep the lines below readable: the strings themselves are never typed, and
//which register each one names is recorded in the Walrus appendix of
//NW-Device-Specification, beside the register map.
//
//A summary row carries a mean, a per-reading row carries one reading, and the
//vocabulary says which with its prefix operator. Both forms are generated from
//the same CSV row, so the pair cannot drift apart.
#define HDR_PRESSURE         NW_HDR_SUBMERSIBLE_SENSOR_WALRUS_FLUID__PRESSURE
#define HDR_PRESSURE_MEAN    NW_HDR_MEAN_OF_SUBMERSIBLE_SENSOR_WALRUS_FLUID__PRESSURE
#define HDR_PRESSURE_STD     NW_HDR_STD_OF_SUBMERSIBLE_SENSOR_WALRUS_FLUID__PRESSURE
#define HDR_PRESSURE_STERR   NW_HDR_STERR_OF_SUBMERSIBLE_SENSOR_WALRUS_FLUID__PRESSURE
#define HDR_TEMP_EXT         NW_HDR_SUBMERSIBLE_SENSOR_WALRUS_FLUID__TEMPERATURE
#define HDR_TEMP_EXT_MEAN    NW_HDR_MEAN_OF_SUBMERSIBLE_SENSOR_WALRUS_FLUID__TEMPERATURE
#define HDR_TEMP_EXT_STD     NW_HDR_STD_OF_SUBMERSIBLE_SENSOR_WALRUS_FLUID__TEMPERATURE
#define HDR_TEMP_EXT_STERR   NW_HDR_STERR_OF_SUBMERSIBLE_SENSOR_WALRUS_FLUID__TEMPERATURE
#define HDR_TEMP_MS5803      NW_HDR_SUBMERSIBLE_SENSOR_WALRUS_PRESSURE_SENSOR__TEMPERATURE
#define HDR_TEMP_MS5803_MEAN NW_HDR_MEAN_OF_SUBMERSIBLE_SENSOR_WALRUS_PRESSURE_SENSOR__TEMPERATURE
#define HDR_TEMP_MS5803_STD  NW_HDR_STD_OF_SUBMERSIBLE_SENSOR_WALRUS_PRESSURE_SENSOR__TEMPERATURE
#define HDR_TEMP_MS5803_STERR NW_HDR_STERR_OF_SUBMERSIBLE_SENSOR_WALRUS_PRESSURE_SENSOR__TEMPERATURE
#define HDR_D1               NW_HDR_SUBMERSIBLE_SENSOR_WALRUS_PRESSURE_SENSOR_PRESSURE__ADC_OUTPUT
#define HDR_D2               NW_HDR_SUBMERSIBLE_SENSOR_WALRUS_PRESSURE_SENSOR_TEMPERATURE__ADC_OUTPUT

Walrus::Walrus()
{
}

bool Walrus::begin(uint8_t Address_)
{
    //Page 0 gates: Schema 1, the name "Walrus", firmware patch >= WALRUS_FW_MIN_PATCH.
    if(!_dev.begin(Address_, "Walrus", WALRUS_FW_MIN_PATCH)) return false;

    //Which MS5803 is fitted, from Page 1. The columns are chosen from it here,
    //once, because a file's header must mean the same thing for its whole life.
    _modelRead = _dev.readBytes(MS5803_MODEL_REG, &_model, 1);
    return true;
}

bool Walrus::updateMeasurements(uint8_t component)
{
    bool doMS = component & MS5803, doMCP = component & MCP9808;
    //The conversions are reading state like the arrays beside them. A reading
    //that does not complete must leave none of the last one behind: with no
    //model on Page 1 the conversions are the value column, where a stale word
    //reads as a measurement rather than as a gap.
    if(doMS) {
        _pressureReadings.reset();
        _tempMS5803Readings.reset();
        clearADC();
    }
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

uint8_t  Walrus::getMS5803Model()    { return _model; }

bool Walrus::modelKnown()
{
    //A sensor that never answered begin() has told us nothing, and relabelling
    //a file's columns because it was unplugged would be worse than assuming the
    //usual. Only a Page 1 we actually read, naming no MS5803, switches them.
    if(!_modelRead) return true;

    //The six parts a Walrus can carry, by the bar figure in the order code.
    return _model == 1 || _model == 2 || _model == 5
        || _model == 7 || _model == 14 || _model == 30;
}

void Walrus::clearADC()
{
    _pressureAdc = WALRUS_ADC_NOT_READ;
    _temperatureAdc = WALRUS_ADC_NOT_READ;
}

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

//The summary interface: the columns a logger writes, streamed. getHeader() and
//getString() are the same column set collected into a String, which keeps one
//definition of it. See LIBRARY-DESIGN.md section 14.
size_t Walrus::printDataHeader(Print& out)
{
    //The summary row: every value is a mean over the readings taken, which is
    //why these carry the mean_of_ operator. With no model on Page 1 the device
    //converts nothing, so the MS5803's two columns carry its conversions
    //instead and say so. The MCP9808 is unaffected: it needs no model.
    size_t n = 0;
    n += out.print(modelKnown() ? F(HDR_PRESSURE_MEAN) : F(HDR_D1));
    n += out.print(',');
    if(modelKnown() && _pressureCfg.columns()) {
        n += out.print(F(HDR_PRESSURE_STD));
        n += out.print(',');
        n += out.print(F(HDR_PRESSURE_STERR));
        n += out.print(',');
    }
    n += out.print(F(HDR_TEMP_EXT_MEAN));
    n += out.print(',');
    if(_temperatureCfg.columns()) {
        n += out.print(F(HDR_TEMP_EXT_STD));
        n += out.print(',');
        n += out.print(F(HDR_TEMP_EXT_STERR));
        n += out.print(',');
    }
    n += out.print(modelKnown() ? F(HDR_TEMP_MS5803_MEAN) : F(HDR_D2));
    n += out.print(',');
    if(modelKnown() && _pressureCfg.columns()) {
        n += out.print(F(HDR_TEMP_MS5803_STD));
        n += out.print(',');
        n += out.print(F(HDR_TEMP_MS5803_STERR));
        n += out.print(',');
    }
    if(modelKnown() && _adcColumns) {
        n += out.print(F(HDR_D1));
        n += out.print(',');
        n += out.print(F(HDR_D2));
        n += out.print(',');
    }
    return n;
}

size_t Walrus::printDataRow(Print& out)
{
    //The values the last updateMeasurements() left, in printDataHeader()'s
    //order. This takes no reading: the caller has already acquired, and a row
    //written to two sinks must not acquire twice.
    size_t n = 0;
    if(modelKnown()) n += out.print(getPressure());
    else n += out.print(getPressureADC());
    n += out.print(',');
    if(modelKnown() && _pressureCfg.columns()) {
        n += out.print(getPressureStd());
        n += out.print(',');
        n += out.print(getPressureSterr());
        n += out.print(',');
    }
    n += out.print(getTemperature());
    n += out.print(',');
    if(_temperatureCfg.columns()) {
        n += out.print(getTemperatureStd());
        n += out.print(',');
        n += out.print(getTemperatureSterr());
        n += out.print(',');
    }
    if(modelKnown()) n += out.print(getMS5803Temperature());
    else n += out.print(getTemperatureADC());
    n += out.print(',');
    if(modelKnown() && _pressureCfg.columns()) {
        n += out.print(getMS5803TemperatureStd());
        n += out.print(',');
        n += out.print(getMS5803TemperatureSterr());
        n += out.print(',');
    }
    if(modelKnown() && _adcColumns) {
        n += out.print(getPressureADC());
        n += out.print(',');
        n += out.print(getTemperatureADC());
        n += out.print(',');
    }
    return n;
}

String Walrus::getHeader()
{
    String h;
    NW_StringPrint p(h);
    printDataHeader(p);
    return h;
}

String Walrus::getString()
{
    updateMeasurements();                           //NW_ERROR (-9999) where a reading failed
    String s;
    NW_StringPrint p(s);
    printDataRow(p);
    return s;
}

//The reading interface: one reading per logReading(), printed as it is taken.
void Walrus::beginReadings(uint8_t component, uint16_t n)
{
    _component = component;
    if(component & MS5803) {
        _pressureReadings.reset();
        _tempMS5803Readings.reset();
        clearADC();
    }
    if(component & MCP9808) _tempExtReadings.reset();
    _dev.beginBatch(n);
}

void Walrus::endReadings()
{
    //No cleanup required currently
}

size_t Walrus::printHeader(Print& out)
{
    //The columns printReading() writes, in the order getHeader() and the
    //specification's binding table give: pressure, the medium's temperature,
    //then the MS5803's own. No statistics columns, because one reading has
    //none. With no model on Page 1 the MS5803's two columns carry its
    //conversions instead, exactly as they do in getHeader().
    size_t n = 0;
    if(_component & MS5803) {
        n += out.print(modelKnown() ? F(HDR_PRESSURE) : F(HDR_D1));
        n += out.print(',');
    }
    if(_component & MCP9808) {
        n += out.print(F(HDR_TEMP_EXT));
        n += out.print(',');
    }
    if(_component & MS5803) {
        n += out.print(modelKnown() ? F(HDR_TEMP_MS5803) : F(HDR_D2));
        n += out.print(',');
    }
    return n;
}

size_t Walrus::printReading(Print& out)
{
    size_t n = 0;
    if(_component & MS5803) {
        if(modelKnown()) n += out.print(_pressure);
        else n += out.print(_pressureAdc);
        n += out.print(',');
    }
    if(_component & MCP9808) {
        n += out.print(_tempExt);
        n += out.print(',');
    }
    if(_component & MS5803) {
        if(modelKnown()) n += out.print(_tempMS5803);
        else n += out.print(_temperatureAdc);
        n += out.print(',');
    }
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
