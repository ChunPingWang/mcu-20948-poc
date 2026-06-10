# Arduino Uno × ICM-20948 PoC

[![GitHub](https://img.shields.io/badge/GitHub-mcu--20948--poc-blue?logo=github)](https://github.com/ChunPingWang/mcu-20948-poc)

Arduino Uno 整合 TDK ICM-20948 九軸 IMU（加速度計 + 陀螺儀 + 磁力計）的概念驗證專案。  
提供三個層次的實作：函式庫版、暫存器直接操作版，以及自動化驗證測試套件。

---

## 硬體需求

| 元件 | 規格 | 備註 |
|------|------|------|
| Arduino Uno | Rev3 | 5V 主控板 |
| ICM-20948 分接板 | SparkFun / CJMCU | **必須含** 3.3V LDO 與電位轉換電路 |
| 4.7 kΩ 電阻 × 2 | ¼W | I2C pull-up（若板上無） |
| 100 nF 電容 × 2 | 陶瓷 | VCC 去耦電容 |

> ⚠️ ICM-20948 工作電壓 1.71–3.6 V，Arduino Uno 為 5 V。  
> 務必使用含電位轉換電路的分接板，裸模組直接接 5 V 邏輯將損壞感測器。

---

## 接線（I2C 模式）

```
Arduino Uno          ICM-20948 Breakout
───────────────────────────────────────
3.3V  ────────────── VCC
GND   ────────────── GND
A4 (SDA) ──[4.7kΩ]── SDA  (pull-up to 3.3V)
A5 (SCL) ──[4.7kΩ]── SCL  (pull-up to 3.3V)
D2       ────────────INT   (可選，資料就緒中斷)
                     AD0 ── GND  → I2C 位址 0x68
                     AD0 ── VCC  → I2C 位址 0x69
```

去耦電容接於 VCC 與 GND 之間，盡量靠近感測器引腳。

---

## 專案結構

```
.
├── 01_basic_library/
│   └── icm20948_basic.ino       # SparkFun 函式庫版（推薦入門）
├── 02_low_level/
│   └── icm20948_lowlevel.ino    # 直接操作 I2C 暫存器（無需外部函式庫）
└── 03_validation/
    └── icm20948_validation.ino  # 8 項自動化驗證測試
```

---

## 使用說明

### 01 — 函式庫版（快速上手）

1. 開啟 Arduino IDE → Library Manager，搜尋並安裝 **SparkFun ICM-20948**。
2. 燒錄 `01_basic_library/icm20948_basic.ino`。
3. 開啟 Serial Monitor（115200 baud）或 Serial Plotter，觀察 CSV 輸出。

輸出格式：
```
ax(g), ay(g), az(g), gx(dps), gy(dps), gz(dps), mx(uT), my(uT), mz(uT), temp(C)
```

感測器設定：
- 加速度計：±2 g，DLPF ≈ 50 Hz
- 陀螺儀：±250 dps，DLPF ≈ 51 Hz
- 磁力計：AK09916 連續模式 4（100 Hz）
- 輸出率：50 Hz

---

### 02 — 暫存器直接操作版

無需安裝任何外部函式庫。直接操作 ICM-20948 User Bank 0/2 暫存器，  
磁力計（AK09916）透過 **I2C Bypass 模式**直接存取。

主要流程：
1. `bankSel(0)` → 讀取 `WHO_AM_I`（期望值 `0xEA`）
2. Soft reset → Wake → 啟用所有軸
3. `bankSel(2)` → 設定 DLPF / Full-Scale
4. `INT_PIN_CFG |= 0x02`（BYPASS_EN）→ Arduino 直接與 AK09916（`0x0C`）通訊
5. 主迴圈以 50 Hz 讀取 burst 暫存器並輸出 CSV

---

### 03 — 驗證測試套件

燒錄前將感測器**水平靜置，Z 軸朝上**。  
燒錄後開啟 Serial Monitor（115200 baud），程式 3 秒後自動執行。

| 測試 | 項目 | 通過條件 |
|:----:|------|----------|
| T1 | WHO_AM_I 識別 | `0xEA` |
| T2 | AK09916 識別 | `0x09` |
| T3 | 靜態加速度 Z 軸 | `0.90 g ≤ az ≤ 1.10 g` |
| T4 | 靜態加速度 XY 軸 | `\|ax\|, \|ay\| < 0.10 g` |
| T5 | 靜態陀螺儀雜訊 | 峰值 `< 2.0 dps` |
| T6 | 地磁場大小 | `20 ≤ \|B\| ≤ 80 μT` |
| T7 | 溫度合理性 | `10–50 °C` |
| T8 | 輸出資料率 | `45–55 Hz` |

全部通過後輸出：
```
====================================
  Result: 8 PASS  /  0 FAIL
====================================
```

---

## 常見問題

| 現象 | 可能原因 | 解法 |
|------|----------|------|
| `WHO_AM_I = 0xFF` | 接線錯誤 / 電源不足 | 確認 SDA、SCL 未接反，量測 VCC |
| 加速度固定為 0 | PWR_MGMT_1 未清除 sleep bit | 確認初始化呼叫 `icmWr(PWR_MGMT_1, 0x01)` |
| 磁力計無資料（T6 失敗）| I2C Bypass 未開啟 | 確認 `INT_PIN_CFG = 0x02` 已寫入 |
| 資料跳動劇烈 | DLPF 未啟用 | 確認 `GYRO_CONFIG_1 / ACCEL_CONFIG` 的 `FCHOICE = 1` |
| 編譯錯誤（01_basic_library）| 未安裝 SparkFun 函式庫 | Library Manager 安裝 **SparkFun ICM-20948** |

---

## 關鍵暫存器速查（ICM-20948）

| 暫存器 | Bank | 位址 | 說明 |
|--------|:----:|:----:|------|
| `WHO_AM_I` | 0 | `0x00` | 識別碼，值 `0xEA` |
| `PWR_MGMT_1` | 0 | `0x06` | 重置 / 喚醒 / 時脈選擇 |
| `INT_PIN_CFG` | 0 | `0x0F` | bit1: BYPASS_EN |
| `ACCEL_XOUT_H` | 0 | `0x2D` | 加速度 burst 起點（12 bytes accel+gyro） |
| `TEMP_OUT_H` | 0 | `0x39` | 溫度 |
| `REG_BANK_SEL` | — | `0x7F` | bits[5:4]: 切換 Bank 0–3 |
| `GYRO_CONFIG_1` | 2 | `0x01` | DLPF / FS_SEL / FCHOICE |
| `ACCEL_CONFIG` | 2 | `0x14` | DLPF / FS_SEL / FCHOICE |

---

## 相關專案

- [pico-20948-poc](https://github.com/ChunPingWang/pico-20948-poc) — Raspberry Pi Pico 版本（MicroPython，3.3V 直接連接，無需電位轉換）

---

## 授權

MIT
