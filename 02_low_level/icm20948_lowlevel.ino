// ICM-20948 PoC — Direct Register Access (No External Library)
//
// Demonstrates full register-level I2C communication with:
//   - ICM-20948 (Accel + Gyro + Temp)
//   - AK09916 internal magnetometer (via I2C bypass mode)
//
// Wiring: same as icm20948_basic.ino

#include <Wire.h>

// ── I2C Addresses ─────────────────────────────────
#define ICM_ADDR    0x68   // AD0=GND → 0x68; AD0=VCC → 0x69
#define AK_ADDR     0x0C   // AK09916 magnetometer (always 0x0C)

// ── Bank 0 Registers ──────────────────────────────
#define WHO_AM_I        0x00  // Expected: 0xEA
#define USER_CTRL       0x03
#define PWR_MGMT_1      0x06  // bit7: reset, bit6: sleep, [2:0]: clksel
#define PWR_MGMT_2      0x07  // bit5-3: disable accel axes, bit2-0: disable gyro axes
#define INT_PIN_CFG     0x0F  // bit1: BYPASS_EN
#define ACCEL_XOUT_H    0x2D  // 14 bytes: AX AY AZ GX GY GZ TP (all H:L)
#define TEMP_OUT_H      0x39
#define REG_BANK_SEL    0x7F  // bits[5:4]: USER_BANK

// ── Bank 2 Registers ──────────────────────────────
#define GYRO_CONFIG_1       0x01  // [5:3]DLPFCFG [2:1]FS_SEL [0]FCHOICE
#define ACCEL_SMPLRT_DIV_1  0x10  // [3:0] upper bits of 12-bit divider
#define ACCEL_SMPLRT_DIV_2  0x11  // [7:0] lower bits
#define ACCEL_CONFIG        0x14  // [5:3]DLPFCFG [2:1]FS_SEL [0]FCHOICE

// ── AK09916 Registers ─────────────────────────────
#define AK_WIA2     0x01  // Expected: 0x09
#define AK_ST1      0x10  // bit0: DRDY (data ready)
#define AK_HXL      0x11  // 6 bytes: HX HY HZ (L:H byte order!) + ST2
#define AK_ST2      0x18  // bit3: HOFL (overflow)
#define AK_CNTL2    0x31  // [4:0]: MODE  0x08=100Hz continuous
#define AK_CNTL3    0x32  // bit0: SRST (soft reset)

// ── Scale Factors ────────────────────────────────
#define ACCEL_SCALE  16384.0f   // LSB/g   for ±2g
#define GYRO_SCALE     131.0f   // LSB/dps for ±250dps
#define MAG_SCALE        0.15f  // μT/LSB
#define TEMP_SCALE     333.87f
#define TEMP_OFFSET     21.0f

// ─── Register I/O ────────────────────────────────

static void bankSel(uint8_t bank) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(REG_BANK_SEL);
  Wire.write((bank & 0x03) << 4);
  Wire.endTransmission();
}

static void icmWr(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static uint8_t icmRd(uint8_t reg) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ICM_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

static void icmRdBurst(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ICM_ADDR, len);
  for (uint8_t i = 0; i < len; i++)
    buf[i] = Wire.available() ? Wire.read() : 0;
}

static void akWr(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(AK_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static uint8_t akRd(uint8_t reg) {
  Wire.beginTransmission(AK_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)AK_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

static void akRdBurst(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(AK_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)AK_ADDR, len);
  for (uint8_t i = 0; i < len; i++)
    buf[i] = Wire.available() ? Wire.read() : 0;
}

// ─── Sensor Init ────────────────────────────────

bool initICM20948() {
  bankSel(0);

  uint8_t id = icmRd(WHO_AM_I);
  if (id != 0xEA) {
    Serial.print(F("[ERR] WHO_AM_I=0x")); Serial.print(id, HEX);
    Serial.println(F(", expected 0xEA"));
    return false;
  }

  icmWr(PWR_MGMT_1, 0x80);   // soft reset (bit7)
  delay(50);
  icmWr(PWR_MGMT_1, 0x01);   // wake, auto clock select
  delay(30);
  icmWr(PWR_MGMT_2, 0x00);   // enable all accel + gyro axes

  // Bank 2: configure DLPF and full-scale range
  bankSel(2);
  // GYRO_CONFIG_1: DLPFCFG=3 (~51Hz BW), FS_SEL=0 (±250dps), FCHOICE=1
  icmWr(GYRO_CONFIG_1, 0x19);
  // ACCEL_CONFIG:  DLPFCFG=3 (~50Hz BW), FS_SEL=0 (±2g),     FCHOICE=1
  icmWr(ACCEL_CONFIG, 0x19);
  // Sample rate divider: ODR = 1.125 kHz / (1+0) — max rate
  icmWr(ACCEL_SMPLRT_DIV_1, 0x00);
  icmWr(ACCEL_SMPLRT_DIV_2, 0x00);

  // Bank 0: enable I2C bypass so Arduino can talk directly to AK09916
  bankSel(0);
  icmWr(INT_PIN_CFG, 0x02);   // BYPASS_EN

  return true;
}

bool initMagnetometer() {
  uint8_t wia = akRd(AK_WIA2);
  if (wia != 0x09) {
    Serial.print(F("[ERR] AK09916 WIA2=0x")); Serial.print(wia, HEX);
    Serial.println(F(", expected 0x09"));
    return false;
  }

  akWr(AK_CNTL3, 0x01);   // soft reset
  delay(10);
  akWr(AK_CNTL2, 0x08);   // continuous mode 4 (100 Hz)
  delay(10);
  return true;
}

// ─── Data Reading ───────────────────────────────

struct IMUData {
  float ax, ay, az;   // g
  float gx, gy, gz;   // dps
  float mx, my, mz;   // μT
  float temp;         // °C
  bool  magValid;
};

void readIMU(IMUData &d) {
  bankSel(0);

  // Burst: ACCEL_XOUT_H(0x2D) → 12 bytes accel+gyro, then 2 temp at 0x39
  uint8_t raw[12];
  icmRdBurst(ACCEL_XOUT_H, raw, 12);

  int16_t ax = (int16_t)((raw[0]  << 8) | raw[1]);
  int16_t ay = (int16_t)((raw[2]  << 8) | raw[3]);
  int16_t az = (int16_t)((raw[4]  << 8) | raw[5]);
  int16_t gx = (int16_t)((raw[6]  << 8) | raw[7]);
  int16_t gy = (int16_t)((raw[8]  << 8) | raw[9]);
  int16_t gz = (int16_t)((raw[10] << 8) | raw[11]);

  uint8_t tp[2];
  icmRdBurst(TEMP_OUT_H, tp, 2);
  int16_t temp = (int16_t)((tp[0] << 8) | tp[1]);

  d.ax   = ax   / ACCEL_SCALE;
  d.ay   = ay   / ACCEL_SCALE;
  d.az   = az   / ACCEL_SCALE;
  d.gx   = gx   / GYRO_SCALE;
  d.gy   = gy   / GYRO_SCALE;
  d.gz   = gz   / GYRO_SCALE;
  d.temp = (temp / TEMP_SCALE) + TEMP_OFFSET;
}

void readMag(IMUData &d) {
  uint8_t st1 = akRd(AK_ST1);
  if (!(st1 & 0x01)) { d.magValid = false; return; }

  // Read 8 bytes: HXL HXH HYL HYH HZL HZH dummy ST2
  uint8_t raw[8];
  akRdBurst(AK_HXL, raw, 8);

  if (raw[7] & 0x08) { d.magValid = false; return; }  // HOFL

  // AK09916 is little-endian (L byte first)
  int16_t mx = (int16_t)((raw[1] << 8) | raw[0]);
  int16_t my = (int16_t)((raw[3] << 8) | raw[2]);
  int16_t mz = (int16_t)((raw[5] << 8) | raw[4]);

  d.mx = mx * MAG_SCALE;
  d.my = my * MAG_SCALE;
  d.mz = mz * MAG_SCALE;
  d.magValid = true;
}

// ─── Setup & Loop ───────────────────────────────

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin();
  Wire.setClock(400000);

  Serial.println(F("=== ICM-20948 PoC (Low-Level Registers) ==="));

  if (!initICM20948()) { Serial.println(F("[HALT]")); while (1); }
  Serial.println(F("[OK] ICM-20948"));

  if (!initMagnetometer()) Serial.println(F("[WARN] Magnetometer skipped"));
  else                     Serial.println(F("[OK] AK09916 magnetometer"));

  Serial.println(F("ax(g),ay(g),az(g),gx(dps),gy(dps),gz(dps),mx(uT),my(uT),mz(uT),temp(C)"));
}

IMUData data;

void loop() {
  readIMU(data);
  readMag(data);

  Serial.print(data.ax, 3);  Serial.print(',');
  Serial.print(data.ay, 3);  Serial.print(',');
  Serial.print(data.az, 3);  Serial.print(',');
  Serial.print(data.gx, 2);  Serial.print(',');
  Serial.print(data.gy, 2);  Serial.print(',');
  Serial.print(data.gz, 2);  Serial.print(',');
  if (data.magValid) {
    Serial.print(data.mx, 2); Serial.print(',');
    Serial.print(data.my, 2); Serial.print(',');
    Serial.print(data.mz, 2);
  } else {
    Serial.print(F("N/A,N/A,N/A"));
  }
  Serial.print(',');
  Serial.println(data.temp, 1);

  delay(20);  // 50 Hz
}
