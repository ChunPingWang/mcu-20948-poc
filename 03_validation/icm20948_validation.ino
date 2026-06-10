// ICM-20948 Validation Test Suite
//
// Runs a sequence of functional tests and prints PASS/FAIL to Serial.
// Keep the sensor stationary and flat (Z-axis pointing up) during the test.
//
// Tests:
//   T1: WHO_AM_I identity check
//   T2: AK09916 identity check
//   T3: Accel static Z-axis (flat) ~= 1.0g
//   T4: Accel XY-axes static  ~= 0.0g
//   T5: Gyro static noise within ±2.0 dps
//   T6: Magnetometer Earth field 20–80 μT
//   T7: Temperature sanity 10–50 °C
//   T8: Output data rate ~50 Hz (with 20ms delay)

#include <Wire.h>

#define ICM_ADDR    0x68
#define AK_ADDR     0x0C

// ── Register definitions (same as lowlevel sketch) ─
#define WHO_AM_I        0x00
#define USER_CTRL       0x03
#define PWR_MGMT_1      0x06
#define PWR_MGMT_2      0x07
#define INT_PIN_CFG     0x0F
#define ACCEL_XOUT_H    0x2D
#define TEMP_OUT_H      0x39
#define REG_BANK_SEL    0x7F
#define GYRO_CONFIG_1   0x01
#define ACCEL_CONFIG    0x14
#define AK_WIA2         0x01
#define AK_ST1          0x10
#define AK_HXL          0x11
#define AK_CNTL2        0x31
#define AK_CNTL3        0x32

#define ACCEL_SCALE  16384.0f
#define GYRO_SCALE     131.0f
#define MAG_SCALE        0.15f
#define TEMP_SCALE     333.87f
#define TEMP_OFFSET     21.0f

// ── Low-level I/O (duplicated from lowlevel sketch) ─

static void bankSel(uint8_t bank) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(REG_BANK_SEL);
  Wire.write((bank & 0x03) << 4);
  Wire.endTransmission();
}
static void icmWr(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}
static uint8_t icmRd(uint8_t reg) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ICM_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}
static void icmRdBurst(uint8_t reg, uint8_t *b, uint8_t n) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ICM_ADDR, n);
  for (uint8_t i = 0; i < n; i++) b[i] = Wire.available() ? Wire.read() : 0;
}
static void akWr(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(AK_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}
static uint8_t akRd(uint8_t reg) {
  Wire.beginTransmission(AK_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)AK_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}
static void akRdBurst(uint8_t reg, uint8_t *b, uint8_t n) {
  Wire.beginTransmission(AK_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)AK_ADDR, n);
  for (uint8_t i = 0; i < n; i++) b[i] = Wire.available() ? Wire.read() : 0;
}

// ── Sensor init (minimal, for validation) ──────────

static bool sensorInit() {
  bankSel(0);
  uint8_t id = icmRd(WHO_AM_I);
  if (id != 0xEA) return false;
  icmWr(PWR_MGMT_1, 0x80); delay(50);
  icmWr(PWR_MGMT_1, 0x01); delay(30);
  icmWr(PWR_MGMT_2, 0x00);
  bankSel(2);
  icmWr(GYRO_CONFIG_1, 0x19);
  icmWr(ACCEL_CONFIG,  0x19);
  bankSel(0);
  icmWr(INT_PIN_CFG, 0x02);
  return true;
}

static bool magInit() {
  if (akRd(AK_WIA2) != 0x09) return false;
  akWr(AK_CNTL3, 0x01); delay(10);
  akWr(AK_CNTL2, 0x08); delay(10);
  return true;
}

// ── Sample helpers ─────────────────────────────────

struct Sample {
  float ax, ay, az, gx, gy, gz, mx, my, mz, temp;
  bool magOk;
};

static void takeSample(Sample &s) {
  bankSel(0);
  uint8_t raw[12];
  icmRdBurst(ACCEL_XOUT_H, raw, 12);
  int16_t ax = (int16_t)((raw[0]  << 8)|raw[1]);
  int16_t ay = (int16_t)((raw[2]  << 8)|raw[3]);
  int16_t az = (int16_t)((raw[4]  << 8)|raw[5]);
  int16_t gx = (int16_t)((raw[6]  << 8)|raw[7]);
  int16_t gy = (int16_t)((raw[8]  << 8)|raw[9]);
  int16_t gz = (int16_t)((raw[10] << 8)|raw[11]);
  uint8_t tp[2]; icmRdBurst(TEMP_OUT_H, tp, 2);
  int16_t t  = (int16_t)((tp[0]<<8)|tp[1]);
  s.ax = ax/ACCEL_SCALE; s.ay = ay/ACCEL_SCALE; s.az = az/ACCEL_SCALE;
  s.gx = gx/GYRO_SCALE;  s.gy = gy/GYRO_SCALE;  s.gz = gz/GYRO_SCALE;
  s.temp = t/TEMP_SCALE + TEMP_OFFSET;

  uint8_t st1 = akRd(AK_ST1);
  if (st1 & 0x01) {
    uint8_t m[8]; akRdBurst(AK_HXL, m, 8);
    if (!(m[7] & 0x08)) {
      s.mx = (int16_t)((m[1]<<8)|m[0]) * MAG_SCALE;
      s.my = (int16_t)((m[3]<<8)|m[2]) * MAG_SCALE;
      s.mz = (int16_t)((m[5]<<8)|m[4]) * MAG_SCALE;
      s.magOk = true; return;
    }
  }
  s.magOk = false;
}

static void avgSamples(uint8_t n, uint32_t intervalMs, Sample &avg) {
  Sample s; memset(&avg, 0, sizeof(avg));
  uint8_t magCount = 0;
  for (uint8_t i = 0; i < n; i++) {
    takeSample(s);
    avg.ax += s.ax; avg.ay += s.ay; avg.az += s.az;
    avg.gx += s.gx; avg.gy += s.gy; avg.gz += s.gz;
    avg.temp += s.temp;
    if (s.magOk) { avg.mx += s.mx; avg.my += s.my; avg.mz += s.mz; magCount++; }
    delay(intervalMs);
  }
  avg.ax /= n; avg.ay /= n; avg.az /= n;
  avg.gx /= n; avg.gy /= n; avg.gz /= n;
  avg.temp /= n;
  if (magCount) { avg.mx /= magCount; avg.my /= magCount; avg.mz /= magCount; avg.magOk = true; }
}

// ── Test framework ──────────────────────────────────

uint8_t passCount = 0, failCount = 0;

static void result(const __FlashStringHelper *name, bool ok,
                   const char *detail = nullptr) {
  Serial.print(ok ? F("  [PASS] ") : F("  [FAIL] "));
  Serial.print(name);
  if (detail) { Serial.print(F(" — ")); Serial.print(detail); }
  Serial.println();
  ok ? passCount++ : failCount++;
}

// ── Individual Tests ────────────────────────────────

void testWhoAmI() {
  Serial.println(F("T1: WHO_AM_I identity"));
  bankSel(0);
  uint8_t id = icmRd(WHO_AM_I);
  char buf[24]; snprintf(buf, sizeof(buf), "0x%02X (want 0xEA)", id);
  result(F("ICM-20948 WHO_AM_I"), id == 0xEA, buf);
}

void testMagId() {
  Serial.println(F("T2: AK09916 identity"));
  uint8_t wia = akRd(AK_WIA2);
  char buf[24]; snprintf(buf, sizeof(buf), "0x%02X (want 0x09)", wia);
  result(F("AK09916 WIA2"), wia == 0x09, buf);
}

void testStaticAccel() {
  Serial.println(F("T3+T4: Static accel (keep flat, Z-up)"));
  Serial.println(F("  Sampling 50 points..."));
  Sample avg; avgSamples(50, 20, avg);

  char buf[40];
  snprintf(buf, sizeof(buf), "az=%.3fg (want 0.9..1.1)", avg.az);
  result(F("Accel Z ~1g"), avg.az >= 0.90f && avg.az <= 1.10f, buf);

  snprintf(buf, sizeof(buf), "ax=%.3fg (want <0.1)", fabsf(avg.ax));
  result(F("Accel X ~0g"), fabsf(avg.ax) < 0.10f, buf);

  snprintf(buf, sizeof(buf), "ay=%.3fg (want <0.1)", fabsf(avg.ay));
  result(F("Accel Y ~0g"), fabsf(avg.ay) < 0.10f, buf);
}

void testGyroNoise() {
  Serial.println(F("T5: Gyro static noise (keep still)"));
  Serial.println(F("  Sampling 50 points..."));

  float maxG = 0;
  for (uint8_t i = 0; i < 50; i++) {
    Sample s; takeSample(s);
    float m = max(fabsf(s.gx), max(fabsf(s.gy), fabsf(s.gz)));
    if (m > maxG) maxG = m;
    delay(20);
  }
  char buf[40]; snprintf(buf, sizeof(buf), "peak=%.2f dps (want <2.0)", maxG);
  result(F("Gyro static peak < 2 dps"), maxG < 2.0f, buf);
}

void testMagField() {
  Serial.println(F("T6: Magnetometer Earth field (20–80 μT)"));
  Sample avg; avgSamples(20, 15, avg);
  if (!avg.magOk) { result(F("Magnetometer data"), false, "no DRDY"); return; }

  float total = sqrtf(avg.mx*avg.mx + avg.my*avg.my + avg.mz*avg.mz);
  char buf[48]; snprintf(buf, sizeof(buf), "|B|=%.1f uT (want 20..80)", total);
  result(F("Mag field magnitude"), total >= 20.0f && total <= 80.0f, buf);
}

void testTemperature() {
  Serial.println(F("T7: Temperature sanity (10–50 °C)"));
  Sample s; takeSample(s);
  char buf[32]; snprintf(buf, sizeof(buf), "%.1f C", s.temp);
  result(F("Temperature in range"), s.temp >= 10.0f && s.temp <= 50.0f, buf);
}

void testDataRate() {
  Serial.println(F("T8: Output data rate (~50 Hz with 20ms delay)"));
  const uint8_t N = 20;
  uint32_t t0 = micros();
  for (uint8_t i = 0; i < N; i++) {
    Sample s; takeSample(s);
    delay(20);
  }
  uint32_t elapsed = micros() - t0;
  float hz = N * 1e6f / elapsed;
  char buf[40]; snprintf(buf, sizeof(buf), "%.1f Hz (want 45..55)", hz);
  result(F("ODR 45–55 Hz"), hz >= 45.0f && hz <= 55.0f, buf);
}

// ─── Setup & Run ────────────────────────────────────

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Wire.begin();
  Wire.setClock(400000);

  Serial.println(F("===================================="));
  Serial.println(F("  ICM-20948 Validation Test Suite"));
  Serial.println(F("===================================="));
  Serial.println(F("Place sensor FLAT (Z-axis up) and STILL."));
  Serial.println(F("Starting in 3s..."));
  delay(3000);

  if (!sensorInit()) {
    Serial.println(F("[HALT] Cannot initialize ICM-20948"));
    while (1);
  }
  bool hasMag = magInit();
  if (!hasMag) Serial.println(F("[WARN] Magnetometer not found — T2/T6 will fail"));

  Serial.println();
  testWhoAmI();
  if (hasMag) testMagId(); else result(F("AK09916 WIA2"), false, "init failed");
  testStaticAccel();
  testGyroNoise();
  if (hasMag) testMagField(); else result(F("Mag field magnitude"), false, "no mag");
  testTemperature();
  testDataRate();

  Serial.println();
  Serial.println(F("===================================="));
  Serial.print(F("  Result: "));
  Serial.print(passCount); Serial.print(F(" PASS  /  "));
  Serial.print(failCount); Serial.println(F(" FAIL"));
  Serial.println(F("===================================="));
}

void loop() {}
