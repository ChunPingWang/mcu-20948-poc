// ICM-20948 PoC — SparkFun Library Version
//
// Wiring (I2C, 3.3V breakout with level shifter):
//   Arduino 3.3V → VCC     Arduino GND → GND
//   Arduino A4   → SDA     Arduino A5  → SCL
//   Arduino D2   → INT     AD0 → GND (addr 0x68)
//
// Library: "SparkFun ICM-20948" — install via Arduino Library Manager

#include "ICM_20948.h"

#define WIRE_PORT   Wire
#define AD0_VAL     0       // 0 = AD0 tied to GND (I2C addr 0x68)
#define INT_PIN     2

ICM_20948_I2C imu;
volatile bool dataReady = false;

void IRAM_ATTR onDataReady() { dataReady = true; }

// ─── Setup ────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  pinMode(INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), onDataReady, RISING);

  Serial.println(F("=== ICM-20948 PoC (Library) ==="));

  bool ok = false;
  for (uint8_t attempt = 0; attempt < 5 && !ok; attempt++) {
    imu.begin(WIRE_PORT, AD0_VAL);
    if (imu.status == ICM_20948_Stat_Ok) {
      ok = true;
    } else {
      Serial.print(F("Init attempt ")); Serial.print(attempt + 1);
      Serial.print(F(" failed: ")); Serial.println(imu.statusString());
      delay(500);
    }
  }

  if (!ok) {
    Serial.println(F("[HALT] Cannot reach ICM-20948. Check wiring."));
    while (1);
  }

  configureSensor();
  Serial.println(F("[OK] ICM-20948 ready"));
  printHeader();
}

void configureSensor() {
  // Gyro: ±250 dps, DLPF ~51 Hz
  ICM_20948_smplrt_t rate;
  rate.g = 0;  // ODR = 1.1 kHz / (1+0) = 1.1 kHz, decimated in loop
  rate.a = 0;
  imu.setSampleRate(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, rate);

  ICM_20948_dlpcfg_t dlp;
  dlp.a = acc_d50bw4_n68bw8;   // ~50 Hz BW
  dlp.g = gyr_d51bw2_n73bw3;
  imu.setDLPFcfg(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, dlp);

  imu.enableDLPF(ICM_20948_Internal_Acc, true);
  imu.enableDLPF(ICM_20948_Internal_Gyr, true);

  imu.setFullScale(ICM_20948_Internal_Acc, ICM_20948_fss_a_2g);
  imu.setFullScale(ICM_20948_Internal_Gyr, ICM_20948_fss_g_250dps);

  // Magnetometer continuous mode 4 (100 Hz)
  imu.startupMagnetometer();
}

void printHeader() {
  Serial.println(F("ax(g),ay(g),az(g),gx(dps),gy(dps),gz(dps),mx(uT),my(uT),mz(uT),temp(C)"));
}

// ─── Loop ────────────────────────────────────────

void loop() {
  if (imu.dataReady()) {
    imu.getAGMT();
    printCSV();
  }
  delay(20);  // ~50 Hz output
}

void printCSV() {
  Serial.print(imu.accX() / 1000.0f, 3); Serial.print(',');
  Serial.print(imu.accY() / 1000.0f, 3); Serial.print(',');
  Serial.print(imu.accZ() / 1000.0f, 3); Serial.print(',');
  Serial.print(imu.gyrX(), 2);           Serial.print(',');
  Serial.print(imu.gyrY(), 2);           Serial.print(',');
  Serial.print(imu.gyrZ(), 2);           Serial.print(',');
  Serial.print(imu.magX(), 2);           Serial.print(',');
  Serial.print(imu.magY(), 2);           Serial.print(',');
  Serial.print(imu.magZ(), 2);           Serial.print(',');
  Serial.println(imu.temp(), 1);
}
