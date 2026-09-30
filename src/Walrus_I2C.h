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

#ifndef Walrus_I2C_h
#define Walrus_I2C_h

#include <Arduino.h>
#include <NW_Core.h>   // NW_Core: NW_Device (Schema 1 protocol), NW_Report

/// Lowest firmware patch (Page 0 byte 0x0A) this library accepts: patch 2,
/// the first to serve the MS5803's own conversions in Page 2 Block 3.
/// brought the Block 0 handshake (trigger, reading counter, faults).
#define WALRUS_FW_MIN_PATCH 2

// Build identity: this library's version (held equal to library.properties by
// NW-Tests/version_check.py) and its build commit, set by the NW-Build wrapper from
// git and blank in an Arduino IDE build. Both go into a logger's status file.
#define WALRUS_LIBRARY_VERSION "0.2.0"
#ifndef WALRUS_LIBRARY_COMMIT
#define WALRUS_LIBRARY_COMMIT ""
#endif

// Readings per updateMeasurements() are kept in static arrays of this
// capacity (one per chip group; no heap); set<Field>Readings(n) clamps to it.
// Override before the include to trade RAM for a longer batch.
#ifndef WALRUS_PRESSURE_CAPACITY
  #define WALRUS_PRESSURE_CAPACITY 16   // MS5803: pressure and its temperature
#endif
#ifndef WALRUS_TEMPERATURE_CAPACITY
  #define WALRUS_TEMPERATURE_CAPACITY 16   // MCP9808: external temperature
#endif

#define PRES_REG    0x48  // Schema 1 Page 2 Block 1: pressure, int32, µBar
#define TEMP_MS5803 0x4C  // Schema 1 Page 2 Block 1: MS5803 temperature, int16, 0.01 °C
#define TEMP_EXT    0x50  // Schema 1 Page 2 Block 2: external temperature (MCP9808), int16, 0.01 °C
#define ADC_REG     0x58  // Schema 1 Page 2 Block 3: MS5803 D1 and D2, uint32 each, ADC counts
/// Bytes of Page 2 a reading spans: Block 1 through Block 3, 0x48 to 0x5F.
#define WALRUS_DATA_BYTES 24
/// Most the Walrus can clock out in one read. Its USI peripheral has a 16-byte
/// transmit buffer (ATTinyCore USI_TWI_Slave.h, TWI_TX_BUFFER_SIZE), so a
/// longer request is answered with 0xFF past the sixteenth byte and a reading
/// must be fetched in two transactions.
#define WALRUS_READ_MAX 16

/**
 * @class Walrus: .
 * @brief Class to interface with the Walrus submersible temperature and
 * pressure sensor
 * @details The Walrus is an encapsulated submersible
 * pressure and temperature sensor intended for water-level or barometric
 * monitoring.
 *
 * \verbatim [![DOI](https://zenodo.org/badge/219609527.svg)](https://zenodo.org/badge/latestdoi/219609527) \endverbatim
 */
class Walrus : public NW_Sensor
{
    public:
        /** @brief Default I2C address: NW-Device-Specification Schema 1 'W' (0x57). */
        static constexpr uint8_t DEFAULT_ADDRESS = 0x57;
        /** @brief Chip groups a reading can cover (the spec's chip table: 0 MS5803, 1 MCP9808). */
        enum Component : uint8_t {
            MS5803  = 0x01,  ///< pressure and the MS5803's own temperature
            MCP9808 = 0x02,  ///< external (water) temperature
            ALL     = 0x03
        };
        /**
         * @brief Instantiate Walrus object
         */
        Walrus();  // Constructor
        /**
         * @brief Begin communications with the Walrus using a prescribed
         * address.
         * @details Refuses the device unless Page 0 says Schema 1, the name
         * "Walrus", and a firmware patch of at least WALRUS_FW_MIN_PATCH;
         * beginFailure() says which gate refused.
         * @param Address_: I2C address of Walrus
         * @return true if the device answered and passed the three gates
         */
        bool begin(uint8_t Address_ = DEFAULT_ADDRESS);
        /**
         * @brief Take the configured number of readings of the selected chips
         * and store them for the getters and the statistics.
         * @details Each reading triggers the device and waits for its reading
         * counter to advance (NW-Device-Specification handshake); N > 1 is
         * declared to the device as a batch first. The single-value getters
         * return the mean of the readings taken; a chip the device reports
         * faulted leaves its values at NW_ERROR (-9999). With one reading of
         * ALL, both chips are read in a single transaction.
         * @param component Walrus::ALL (default), Walrus::MS5803 or Walrus::MCP9808.
         * @return true if every selected chip gave at least one valid reading
         */
        bool updateMeasurements(uint8_t component = ALL);
        /** @brief Take ONE reading of the MS5803 (pressure and its temperature) and append it to the readings. */
        bool updatePressure();
        /** @brief Take ONE reading of the MCP9808 (external temperature) and append it to the readings. */
        bool updateTemperature();
        /**
         * @brief Set how many MS5803 readings updateMeasurements() takes
         * (statistics are computed over them). Clamped to WALRUS_PRESSURE_CAPACITY.
         * @return The number actually set.
         */
        uint16_t setPressureReadings(uint16_t n);
        /** @brief Set how many MCP9808 readings updateMeasurements() takes. Clamped to WALRUS_TEMPERATURE_CAPACITY. */
        uint16_t setTemperatureReadings(uint16_t n);
        /** @brief Enable or disable pressure and MS5803-temperature std and sterr columns in getString()/getHeader(). */
        void setPressureStats(bool enable);
        /** @brief Enable or disable external-temperature std and sterr columns in getString()/getHeader(). */
        void setTemperatureStats(bool enable);
        /** @brief Number of valid MS5803 readings stored by the last updateMeasurements(). */
        uint16_t getPressureCount();
        /** @brief Number of valid MCP9808 readings stored by the last updateMeasurements(). */
        uint16_t getTemperatureCount();
        /**
         * @brief Return calculated temperature from Walrus.
         * @details This calculated temperature can be from either
         * the MS5803 sensor, which primarily measures pressure, or from the
         * dedicated MCP9808 sensor. Values are those stored by the last
         * updateMeasurements() (NW_ERROR before the first).
         *
         * This is the MCP9808, exposed to whatever the unit is installed in:
         * the measurement, which is why it is the unqualified name. It is also
         * what setTemperatureReadings(), setTemperatureStats() and
         * getTemperatureCount() have always meant.
         */
        float getTemperature();
        /**
         * @brief The MS5803's own die temperature [C].
         * @details A diagnostic, not a measurement of the medium: it is what
         * compensates the pressure, and it reads the inside of the pressure
         * sensor. Named for the part because only the part tells the Walrus's
         * two thermometers apart, which is the chip-naming rule in
         * NW-Device-Specification.
         */
        float getMS5803Temperature();
        /**
         * @brief Return calculated pressure from sensor [mBar].
         * @details This is the MS5803 sensor, which can come in a variety
         * of different pressure ranges and sensitivities. The mean of the
         * readings stored by the last updateMeasurements().
         */
        float getPressure();
        /**
         * @brief The MS5803's digital pressure value, D1, in its own counts.
         * @details The conversion the compensated pressure was computed from,
         * served whole on every reading. A reading can be checked after the
         * fact with it, and a controller that knows the variant can compensate
         * for itself. The last reading rather than a burst mean: counts carry
         * no statistics, and the compensation is non-linear in D2, so a mean of
         * the counts is not the counts of the mean. Zero before the first
         * reading. Firmware patch 2 and above.
         */
        uint32_t getPressureADC();
        /** @brief The MS5803's digital temperature value, D2, in its own counts; see getPressureADC(). */
        uint32_t getTemperatureADC();
        /** @brief Include the D1 and D2 columns in getString() and getHeader(). Off by default. */
        void setADCColumns(bool enable);

        // --- Statistics getters ---
        // Computed two-pass in 32-bit float over the readings stored by the last
        // updateMeasurements() (NW_Readings). Adequate for N up to the array
        // capacities; at N in the thousands the sum of squared deviations would
        // want double precision, which the AVR lacks.
        /** @brief Pressure mean [mBar] over the stored readings (NW_ERROR when none). */
        float getPressureMean();
        /** @brief Pressure standard deviation [mBar]. */
        float getPressureStd();
        /** @brief Pressure standard error [mBar]. */
        float getPressureSterr();
        /** @brief Pressure median [mBar] (mean of the middle pair for even N). */
        float getPressureMedian();
        /** @brief Medium temperature mean [C] over the stored readings (NW_ERROR when none). */
        float getTemperatureMean();
        /** @brief Medium temperature standard deviation [C]. */
        float getTemperatureStd();
        /** @brief Medium temperature standard error [C]. */
        float getTemperatureSterr();
        /** @brief Medium temperature median [C]. */
        float getTemperatureMedian();
        /** @brief MS5803 die temperature mean [C]. */
        float getMS5803TemperatureMean();
        /** @brief MS5803 die temperature standard deviation [C]. */
        float getMS5803TemperatureStd();
        /** @brief MS5803 die temperature standard error [C]. */
        float getMS5803TemperatureSterr();
        /** @brief MS5803 die temperature median [C]. */
        float getMS5803TemperatureMedian();
        /**
         * @brief Return header
         * @details "Pressure [mBar],Temp DH [C],Temp DHt [C]," with std and
         * sterr columns after a value when its statistics are enabled and
         * more than one reading is configured.
         */
        String getHeader();
        /**
         * @brief Take a reading (updateMeasurements()) and return it as a string
         * @details String(getPressure()) + "," + String(getTemperature())
         + "," + String(getMS5803Temperature()) + ","; statistics columns as
         * getHeader() describes.
         */
        String getString();

        // --- Reading interface (NW standard) ---
        /**
         * @brief Print the header matching printReading(): column names with
         * units, each followed by a comma, for the chips selected by
         * beginReadings(). No statistics columns: one reading has none.
         * @param out Any Print destination (SdFat File, Serial, ...).
         * @return Bytes written.
         */
        size_t printHeader(Print& out);
        /**
         * @brief Print the stored reading of the selected chips, each value
         * followed by a comma. Does not acquire: call updateMeasurements()
         * first, or use logReading(). Writes: pressure [mBar], MS5803
         * temperature [C] for MS5803; external temperature [C] for MCP9808.
         * @return Bytes written.
         */
        size_t printReading(Print& out);
        /**
         * @brief Take ONE reading of the selected chips and print it: the
         * one-reading primitive for collecting many readings to a file.
         * @return Bytes written.
         */
        size_t logReading(Print& out);
        /**
         * @brief Begin a run of readings, selecting which chips they cover.
         * @param component Walrus::ALL, Walrus::MS5803 or Walrus::MCP9808.
         * @param n How many readings the run will take (the number of
         * logReading() calls to follow); with n > 1 the device is told in
         * advance (readings-requested word). Nothing on Walrus is powered per
         * batch, so the word only satisfies the protocol.
         */
        void beginReadings(uint8_t component = ALL, uint16_t n = 0);
        /** @brief End a run of readings. */
        void endReadings();
        /**
        * @brief Checks for updated data. Returns `true` if the device's ready
        * bit is set; otherwise returns `false`.
        * @deprecated Use ready(); the handshake is newReading() after requestReading().
        */
        bool newData();

        // --- Handshake (NW-Device-Specification Block 0) ---
        /** @brief Status ready bit: the data registers hold a complete reading. */
        bool ready();
        /** @brief The reading counter has advanced since the last request. */
        bool newReading();
        /** @brief Trigger a reading of both chips without waiting for it. */
        bool requestReading();

        // --- Faults (status byte, live; Report register, latched) ---
        /** @brief True if the given chip (0 = MS5803, 1 = MCP9808) was faulted in the last reading. */
        bool faulted(uint8_t chip);
        /** @brief True if any chip was faulted in the last reading (status pan-fault bit). */
        bool anyFault();
        /** @brief Chip index of the report (0 MS5803, 1 MCP9808, 7 the unit); meaningful when reportKind() != 0. */
        uint8_t reportChip();
        /** @brief Kind of the report, per the spec's table (1 not answering, 6 restarted since configured, ...). */
        uint8_t reportKind();
        /** @brief Print the report as text, e.g. "MS5803: not answering"; "none" when there is no fault. */
        size_t printReport(Print& out);
        /** @brief The report as one word for a note column: "MS5803NotAnswering", "UnitRestarted"; "UnitNone" when none. */
        String reportNote();
        /** @brief Print one status line for a logger's status file: name, serial, versions, the last report, Pages 0-2 in hex; no newline, not answering. */
        size_t printStatus(Print& out, bool boot = false) override;
        // --- NW_Sensor: the logger's view (Margay::watch) ---
        const char* name() const override { return "Walrus"; }
        bool reportIsFault() override;
        uint8_t bootReportKind() override;
        void clearBootReport() override;
        /** @brief Why the last begin() refused, as one word: "NotAnswering", "NotSchema1", "WrongName", "OldFirmware"; "None" after success. */
        String beginFailure();
        uint8_t getHardwareMajor();
        uint8_t getHardwareMinor();
        uint8_t getFirmwareVersion();
    private:
        NW_Device _dev;
        float _pressure = NW_ERROR;   //Mean of the last updateMeasurements() [mBar]
        float _tempExt = NW_ERROR;    //MCP9808 [C]
        float _tempMS5803 = NW_ERROR; //MS5803 [C]
        uint32_t _pressureAdc = 0;    //MS5803 D1, counts, last reading
        uint32_t _temperatureAdc = 0; //MS5803 D2, counts, last reading
        bool _adcColumns = false;     //the counts are a diagnostic: off unless asked for
        // Readings as the device serves them (raw register units), one array per
        // field; statistics come from these and are scaled on the way out.
        NW_Readings<int32_t, WALRUS_PRESSURE_CAPACITY>    _pressureReadings;   //uBar
        NW_Readings<int16_t, WALRUS_PRESSURE_CAPACITY>    _tempMS5803Readings; //0.01 C
        NW_Readings<int16_t, WALRUS_TEMPERATURE_CAPACITY> _tempExtReadings;    //0.01 C
        NW_ReadingsConfig _pressureCfg;    //Readings per updateMeasurements() and stats columns, MS5803 group
        NW_ReadingsConfig _temperatureCfg; //MCP9808 group
        uint8_t _component = ALL;     //Selection of the current beginReadings() run
        //Append one served MS5803 reading unless faulted. Takes a reference to
        //an array of exactly WALRUS_DATA_BYTES, not a pointer: it reads Block 3
        //at offset 16, and a caller that passed a shorter buffer once read past
        //the end of it. The size is now the compiler's business.
        bool readMS5803(const uint8_t (&d)[WALRUS_DATA_BYTES]);
        //Fill a reading's 24 bytes, in two transactions because of the buffer
        //above. Both are checked against the same captured reading by
        //NW_Device::readData(), so they cannot straddle a rewrite.
        bool readPage2(uint8_t (&d)[WALRUS_DATA_BYTES]);
        bool readMCP9808(uint8_t* d); //Append one served MCP9808 reading (2 bytes from 0x50) unless faulted
        void summarise(uint8_t component); //Means into the single-value fields, NW_ERROR when no reading
};

/** @deprecated Use Walrus::DEFAULT_ADDRESS. Every NW library defined this same macro
 *  with a different value, so a sketch including two of them got the last one.
 *  Removed at the next major version. */
#define ADR_DEFAULT Walrus::DEFAULT_ADDRESS

#endif
