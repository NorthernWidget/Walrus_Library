// Output-regression test for Walrus_Library: compiles src/Walrus_I2C.cpp
// against the NW_Core stubs and prints getHeader()/getString()/getters for
// fixed register images. run.sh diffs the result against baseline.txt.
#include "Arduino.h"
#include "Wire.h"
TwoWire Wire;
#include "../../src/Walrus_I2C.cpp"

#include "NW_TestSupport.h"

// Build a Schema 1 register image: Page 0 as NW-Provision writes it (with the
// firmware's patch at 0x0A), Page 2 with a complete reading (Walrus appendix:
// pressure int32 uBar at 0x48, MS5803 temperature int16 0.01 C at 0x4C,
// external temperature int16 0.01 C at 0x50).
static void loadImage(int32_t pressure, int16_t tMS5803, int16_t tExt, uint8_t fwPatch = 1, uint8_t schema = 0x01) {
  uint8_t* r = Wire.image;
  nwLoadPage0(r, "Walrus", 0x57, 2, fwPatch, schema);               // Page 0 and Block 0, HW 0.2
  for (int i = 0; i < 4; i++) r[0x48 + i] = (pressure >> (8 * i)) & 0xFF;
  r[0x4C] = tMS5803 & 0xFF; r[0x4D] = (tMS5803 >> 8) & 0xFF;
  r[0x50] = tExt & 0xFF;    r[0x51] = (tExt >> 8) & 0xFF;
}

static void report(const char* name, Walrus& s) {
  printf("[%s]\n", name);
  printf("header: %s\n", s.getHeader().c_str());
  printf("string: %s\n", s.getString().c_str());
  printf("getters: pressure=%.4f tExt=%.4f tMS5803=%.4f default=%.4f newData=%d\n",
         s.getPressure(), s.getTemperature(), s.getMS5803Temperature(), s.getTemperature(), s.newData());
}

int main() {
  Wire.deviceAddress = 0x57;
  installFirmwareEmulation();

  // 1. A complete reading: 1013.250 mBar, 21.37 C in the MS5803, 4.05 C in the water.
  loadImage(1013250, 2137, 405);
  { Walrus s; s.begin(); report("complete reading", s); }

  // 2. Negative temperatures and a depth reading (2.5 m of water over 1 bar).
  loadImage(1250000, -1234, -50);
  { Walrus s; s.begin(); report("negative temperatures", s); }

  // 3. Device absent: not answering at the address.
  loadImage(1013250, 2137, 405); Wire.present = false;
  { Walrus s; s.begin(); report("device absent", s); }
  Wire.present = true;

  // 4. Device present but never ready (status bit 0 clear, counter frozen).
  loadImage(1013250, 2137, 405); Wire.image[0x40] = 0x00; Wire.onWrite = nullptr;
  { Walrus s; s.begin(); report("never ready", s); }
  installFirmwareEmulation();

  // 5. begin() gates: wrong name, wrong schema, firmware too old, and the versions it reports.
  loadImage(1013250, 2137, 405); Wire.image[0x01] = 'X';
  { Walrus s; bool ok = s.begin(); printf("[wrong name] begin=%d failure=%s\n", ok, s.beginFailure().c_str()); }
  loadImage(1013250, 2137, 405, 1, 0x00);
  { Walrus s; bool ok = s.begin(); printf("[schema 0x00] begin=%d failure=%s\n", ok, s.beginFailure().c_str()); }
  loadImage(1013250, 2137, 405, 0);
  { Walrus s; bool ok = s.begin(); printf("[fw patch 0 < min %d] begin=%d fw=%u failure=%s\n", WALRUS_FW_MIN_PATCH, ok, s.getFirmwareVersion(), s.beginFailure().c_str()); }
  loadImage(1013250, 2137, 405);
  { Walrus s; bool ok = s.begin(); printf("[versions] begin=%d hw=%u.%u fw=%u failure=%s\n", ok, s.getHardwareMajor(), s.getHardwareMinor(), s.getFirmwareVersion(), s.beginFailure().c_str()); }

  // 6. Faults: the MS5803 does not acknowledge (status bit 1, pan-fault, latched 0x01);
  //    the MCP9808 value survives. Then a unit reset code with a clean status.
  loadImage(1013250, 2137, 405);
  { Walrus s; s.begin(); char pb[48];
    onReading = [](TwoWire& w) { w.image[0x40] = 0x83; w.image[0x47] = 0x01; };
    bool ok = s.updateMeasurements(); BufferPrint bp(pb, sizeof pb); s.printReport(bp);
    printf("[MS5803 no ack] update=%d faulted(0)=%d faulted(1)=%d any=%d chip=%u kind=%u text='%s' note='%s'\n",
           ok, s.faulted(0), s.faulted(1), s.anyFault(), s.reportChip(), s.reportKind(), pb, s.reportNote().c_str());
    printf("[MS5803 no ack] string: %s\n", s.getString().c_str());
    onReading = [](TwoWire& w) { w.image[0x40] = 0x01; w.image[0x47] = 0xE6; };
    ok = s.updateMeasurements(); BufferPrint bp2(pb, sizeof pb); s.printReport(bp2);
    printf("[unit reset] update=%d any=%d chip=%u kind=%u text='%s' note='%s'\n", ok, s.anyFault(), s.reportChip(), s.reportKind(), pb, s.reportNote().c_str());
    onReading = nullptr; }

  // 7. Handshake pieces and the cost of one row.
  loadImage(1013250, 2137, 405);
  { Walrus s; s.begin(); unsigned t0 = Wire.transactions;
    bool req = s.requestReading(); bool nr = s.newReading(); bool rd = s.ready();
    printf("[handshake] requestReading=%d newReading=%d ready=%d\n", req, nr, rd);
    s.getString(); printf("[cost] requestFrom calls for one getString(): %u\n", Wire.transactions - t0 - 2); }

  // 8. N readings with statistics: pressure steps through five values, the MCP9808
  //    through three; the batch word reaches the device; getString() grows its columns.
  loadImage(1013250, 2137, 405);
  { Walrus s; s.begin(); int k = 0;
    onReading = [&](TwoWire& w) { k++;
      int32_t p = 1013000 + 100 * (k % 5); for (int i = 0; i < 4; i++) w.image[0x48 + i] = (p >> (8 * i)) & 0xFF;
      int16_t t = 400 + 10 * (k % 3); w.image[0x50] = t & 0xFF; w.image[0x51] = (t >> 8) & 0xFF; };
    printf("[N] setPressureReadings(5)=%u setTemperatureReadings(3)=%u setPressureReadings(99)=%u\n",
           s.setPressureReadings(5), s.setTemperatureReadings(3), s.setPressureReadings(99));
    s.setPressureReadings(5); s.setPressureStats(true); s.setTemperatureStats(true);
    lastRequest = 0; unsigned t0 = Wire.transactions; bool ok = s.updateMeasurements();
    printf("[N=5,3] update=%d pressureCount=%u temperatureCount=%u lastRequest=%u requestFrom=%u\n",
           ok, s.getPressureCount(), s.getTemperatureCount(), lastRequest, Wire.transactions - t0);
    printf("[N=5,3] pressure mean=%.4f std=%.4f sterr=%.4f median=%.4f | tExt mean=%.4f std=%.4f median=%.4f | tMS5803 mean=%.4f std=%.4f\n",
           s.getPressureMean(), s.getPressureStd(), s.getPressureSterr(), s.getPressureMedian(),
           s.getTemperatureMean(), s.getTemperatureStd(), s.getTemperatureMedian(), s.getMS5803TemperatureMean(), s.getMS5803TemperatureStd());
    printf("[N=5,3] header: %s\n", s.getHeader().c_str());
    printf("[N=5,3] string: %s\n", s.getString().c_str());
    // One chip group only: the MCP9808 readings are left untouched by an MS5803 update.
    ok = s.updateMeasurements(Walrus::MS5803);
    printf("[MS5803 only] update=%d pressureCount=%u temperatureCount=%u tExt=%.4f\n", ok, s.getPressureCount(), s.getTemperatureCount(), s.getTemperature());
    onReading = nullptr; }

  // 9. Reading interface: header, three logged readings of ALL, then MCP9808 alone; the
  //    batch word for the run reaches the device.
  loadImage(1013250, 2137, 405);
  { Walrus s; s.begin(); int k = 0; char pb[96];
    onReading = [&](TwoWire& w) { k++; int32_t p = 1013000 + 50 * k; for (int i = 0; i < 4; i++) w.image[0x48 + i] = (p >> (8 * i)) & 0xFF; };
    lastRequest = 0; s.beginReadings(Walrus::ALL, 3);
    BufferPrint bh(pb, sizeof pb); s.printHeader(bh); printf("[run ALL] header: %s lastRequest=%u\n", pb, lastRequest);
    for (int i = 0; i < 3; i++) { BufferPrint bp(pb, sizeof pb); size_t n = s.logReading(bp); printf("[run ALL] row %d (%zu bytes): %s\n", i, n, pb); }
    s.endReadings();
    printf("[run ALL] pressure count=%u mean=%.4f median=%.4f\n", s.getPressureCount(), s.getPressureMean(), s.getPressureMedian());
    s.beginReadings(Walrus::MCP9808);
    BufferPrint bh2(pb, sizeof pb); s.printHeader(bh2); printf("[run MCP9808] header: %s\n", pb);
    BufferPrint bp2(pb, sizeof pb); s.logReading(bp2); s.endReadings(); printf("[run MCP9808] row: %s\n", pb);
    onReading = nullptr; }

  // 10. A dead MS5803 (not answering on the first reading) stops its batch of 10.
  loadImage(1013250, 2137, 405);
  { Walrus s; s.begin(); int k = 0;
    onReading = [&](TwoWire& w) { k++; w.image[0x40] = 0x83; w.image[0x47] = 0x01; };
    s.setPressureReadings(10); bool ok = s.updateMeasurements(Walrus::MS5803);
    printf("[dead MS5803] N=10: update=%d readings taken=%d pressureCount=%u pressure=%.2f note='%s'\n", ok, k, s.getPressureCount(), s.getPressure(), s.reportNote().c_str());
    onReading = nullptr; }

  // The status line for a logger's status file.
  loadImage(101325, 2137, 1850);
  { Walrus s; s.begin(); s.updateMeasurements(); char sb[320]; BufferPrint sp(sb, sizeof sb); size_t k = s.printStatus(sp); printf("[status] %zu bytes: %s\n", k, sb); }

  fprintf(stderr, "bus transactions total: %u\n", Wire.transactions);   // metric, not output
  return 0;
}
