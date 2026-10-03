# Walrus_Library

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.4572367.svg)](https://doi.org/10.5281/zenodo.4572367)

Library for the Northern Widget [Walrus](https://github.com/NorthernWidget/Project-Walrus) submersible pressure and temperature sensor. Based off of the TP-Downhole.

The Walrus is an encapsulated submersible pressure and temperature sensor intended for water-level or barometric monitoring.

**Installation:** included in [NorthernWidget-libraries](https://github.com/NorthernWidget/NorthernWidget-libraries).

```cpp
#include <Walrus_I2C.h>

Walrus sensor;

void setup() {
    Serial.begin(9600);
    if (!sensor.begin()) sensor.printNote(Serial, true);  // NotAnswering, NotSchema1, WrongName, OldFirmware
    sensor.printDataHeader(Serial);   // straight to the port: no row is built in RAM
    Serial.println();
}

void loop() {
    sensor.updateMeasurements();     // take the readings
    sensor.printDataRow(Serial);     // print what they left
    Serial.println();
    delay(1000);
}
```

`updateMeasurements()` takes one reading of both chips through the [NW-Device-Specification](https://github.com/NorthernWidget/NW-Device-Specification) handshake: it triggers the device and waits for its reading counter. `printDataRow(out)` then prints pressure [mBar], the MCP9808 water temperature and the MS5803 temperature [°C], with `-9999.00` where a reading failed. The getters return the stored reading: `getPressure()`, `getTemperature(0)` for the MCP9808 and `getTemperature(1)` for the MS5803.

One `updateMeasurements()` can take several readings of each chip. `setPressureReadings(n)` sets how many the MS5803 takes, for pressure and its temperature. `setTemperatureReadings(n)` does the same for the MCP9808, for the water temperature. Each chip stores up to `WALRUS_PRESSURE_CAPACITY` or `WALRUS_TEMPERATURE_CAPACITY` readings: 16 by default, and you can define a larger one before the include. The values printed are then the means. For the spread, `getPressureMean()`, `getPressureStd()`, `getPressureSterr()` and `getPressureMedian()` read the stored readings. The same four exist for `getTemperature…(0)` and `(1)`, and `getPressureCount()` and `getTemperatureCount()` say how many readings each chip holds. `setPressureStats(true)` or `setTemperatureStats(true)` adds std and sterr columns to `printDataHeader()` and `printDataRow()`. To read one chip alone, call `updateMeasurements(Walrus::MS5803)` or `updateMeasurements(Walrus::MCP9808)`. To log one row per reading to a file, call `beginReadings(component, n)`, `printHeader(out)`, then `logReading(out)` n times and `endReadings()`. Here `out` is any `Print`, such as an SdFat `File` or `Serial`.

Faults are there if you want them. `faulted(chip)` takes 0 for the MS5803 and 1 for the MCP9808. `anyFault()`, `reportChip()` and `reportKind()` summarise the last reading. `printReport(Serial)` prints the report as text, and `printNote(out)` prints it as one word, such as `MS5803NotAnswering`, for a logger's note column. `begin()` refuses a device that is not Schema 1, is not a Walrus, or runs firmware older than patch `WALRUS_FW_MIN_PATCH`. `printNote(out, true)` tells you which gate refused. `printStatus(out)` prints one status line for a logger's status file (name, serial, hardware version, firmware patch and build commit from Page 0, this library's `WALRUS_LIBRARY_VERSION` and `WALRUS_LIBRARY_COMMIT`, the last report as code and note, Pages 0 to 2 in hex), for the logger to write with its timestamp whenever `reportKind()` is not zero. The two commits come from the NW-Build wrapper and are blank in an IDE build. Pages renumbered 2026-09-23 (spec 4c3b18d): calibration is Page 1 at 0x20, data Page 2 at 0x40. This version needs the [NW_Core](https://github.com/NorthernWidget/NW_Core) library and Walrus firmware patch 1 or later.

**Full API reference:** https://docs.northernwidget.com/Walrus_Library/
