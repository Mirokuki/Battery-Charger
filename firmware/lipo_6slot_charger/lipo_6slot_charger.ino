/**
 * 6-Slot 1S LiPo / LiHV Smart USB-C PD Charger Firmware
 * Target MCU: ESP32-C3-WROOM-02-N4
 *
 * Features:
 *  - Controls 6x TI BQ25188YBGR I2C 1S Charger ICs via TCA9548A I2C Mux (0x70)
 *  - Per-slot programmable charge rate: 50mA - 500mA in 10mA steps (150mA default for 150mAh 1C)
 *  - Per-slot programmable termination voltage: 4.20V (LiPo), 4.35V (LiHV), or 3.80V (Storage)
 *  - Per-slot Auto-Start charging when a battery (>2.80V) is detected via CD74HC4051 ADC Mux
 *  - 16x ADC oversampling & rolling median filter to eliminate 600kHz buck switching ripple
 *  - GeeekPi 0.96" SSD1306 128x64 OLED display dashboard showing live status for all 6 slots
 *  - Tactile UP / DOWN buttons to cycle through slots and adjust charge currents / modes
 *  - BLE GATT Server streaming live telemetry notifications & accepting remote control commands
 *  - AW9523B 16-channel constant-current I2C LED driver controlling 6x Bi-Color Red/Green 0603 LEDs
 *  - Native USB Serial/JTAG on ESP32-C3 GPIO18 (D-) and GPIO19 (D+) wired directly to J1 USB-C receptacle
 *
 * ESP32-C3 Arduino IDE Board Setup:
 *  - Board: "ESP32C3 Dev Module"
 *  - USB CDC On Boot: "Enabled" (enables single-cable firmware flashing and Serial Monitor over J1)
 *  - Flash Mode: "QIO 80MHz"
 *  - Flash Size: "4MB (32Mb)"
 *  - Partition Scheme: "Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)"
 */

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLESecurity.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// --- Pin Definitions (Matches KiCad Schematic) ---
#define PIN_MUX_A        0   // GPIO0 -> CD74HC4051 S0 (Pin 11)
#define PIN_BTN_SELECT   1   // GPIO1 <- SW2 (SELECT)
#define PIN_BTN_UP       2   // GPIO2 <- SW1 (UP)
#define PIN_BTN_DOWN     3   // GPIO3 <- SW3 (DOWN)
#define PIN_ADC_MUX_OUT  4   // GPIO4 (ADC1_CH4) <- CD74HC4051 COM (Pin 3)
#define PIN_MUX_B        5   // GPIO5 -> CD74HC4051 S1 (Pin 10)
#define PIN_MUX_C        6   // GPIO6 -> CD74HC4051 S2 (Pin 9)
// GPIO7 is now No Connect (U7 level shifter removed, AW9523B is on I2C)
#define PIN_I2C_SDA      8   // GPIO8 -> Master I2C SDA (TCA9548A + OLED)
#define PIN_I2C_SCL      9   // GPIO9 -> Master I2C SCL (TCA9548A + OLED), J4 Pin 3 (BOOT jumper)

// --- I2C Addresses ---
#define TCA9548A_ADDR    0x70
#define BQ25188_ADDR     0x6A
#define OLED_I2C_ADDR    0x3C

// --- OLED Display Dimensions ---
#define SCREEN_WIDTH     128
#define SCREEN_HEIGHT    64
#define OLED_RESET       -1

// --- BQ25188 Register Map ---
#define BQ_REG_STAT0       0x00  // [6:5] CHG_STAT, [1] THERMREG_ACTIVE
#define BQ_REG_STAT1       0x01
#define BQ_REG_FLAG0       0x02
#define BQ_REG_VBAT_CTRL   0x03  // [6:0] VBAT_REG: 3500mV + code * 10mV
#define BQ_REG_ICHG_CTRL   0x04  // [7] CHG_DIS, [6:0] ICHG
#define BQ_REG_CHARGECTRL0 0x05
#define BQ_REG_CHARGECTRL1 0x06
#define BQ_REG_IC_CTRL     0x07  // [7] TS_EN, [3:2] SAFETY_TIMER, [1:0] WATCHDOG_SEL
#define BQ_REG_TMR_ILIM    0x08  // [2:0] ILIM
#define BQ_REG_SHIP_RST    0x09
#define BQ_REG_SYS_REG     0x0A
#define BQ_REG_TS_CONTROL  0x0B
#define BQ_REG_MASK_ID     0x0C

#define BQ_IC_CTRL_VALUE   0x84  // TS_EN=1, 6h safety timer, 160s watchdog
#define BQ_TMR_ILIM_VALUE  0x4E  // ILIM=665mA

#define BQ_CHG_STAT(stat0) (((stat0) >> 5) & 0x03)
#define BQ_CHG_NOT_CHARGING 0x00
#define BQ_STAT1_SAFETY_TMR_FAULT 0x04
#define BQ_FLAG0_FAULT_MASK 0x05
#define BQ_CHG_CC           0x01
#define BQ_CHG_CV           0x02
#define BQ_CHG_DONE_OR_DIS  0x03

#define NUM_SLOTS           6

// 9V @ 2.5A input = 22.5W. 6 slots * 500mA = 3.0A at 4.8V (14.4W), giving 36% margin.
#define TOTAL_CHARGE_LIMIT_MA 3000

#define BATTERY_PRESENT_MV  2800  // Below this a pack is too low to auto-start
#define BATTERY_MAX_MV      4500  // Above this is treated as a fault
#define BATTERY_GONE_MV     1200  // Disconnect threshold when unpowered
#define STORAGE_MV          3800
#define I2C_FAIL_LIMIT      3

// Physical pin mapping matched to layout:
// Left Bank (Slots 1-3): TCA9548A CH 0, 1, 2 | CD74HC4051 Y 4, 5, 6
// Right Bank (Slots 4-6): TCA9548A CH 4, 5, 6 | CD74HC4051 Y 0, 1, 2
const uint8_t I2C_MUX_CHANNEL[NUM_SLOTS] = {0, 1, 2, 4, 5, 6};
const uint8_t ADC_MUX_CHANNEL[NUM_SLOTS] = {4, 5, 6, 0, 1, 2};

// Down the right bank (slots 4, 5, 6), then up the left bank (slots 3, 2, 1)
// --- AW9523B 16-Channel Constant-Current I2C LED Driver ---
#define AW9523_ADDR      0x5B
#define AW_REG_P0_MODE   0x11  // 0 = LED Mode, 1 = GPIO Mode
#define AW_REG_P1_MODE   0x12  // 0 = LED Mode, 1 = GPIO Mode
#define AW_REG_GCR       0x13  // Global Current Control: 0=37mA, 1=27.75mA, 2=18.5mA, 3=9.25mA
#define AW_REG_SW_RESET  0x7F

// AW9523 Dimming Register Mapping: {Green_Dim_Reg, Red_Dim_Reg}
// Slot 1 (Left Bank Row 1): P1_0 (G), P1_1 (R) -> Regs 0x28, 0x29
// Slot 2 (Left Bank Row 2): P1_2 (G), P1_3 (R) -> Regs 0x2A, 0x2B
// Slot 3 (Left Bank Row 3): P0_0 (G), P0_1 (R) -> Regs 0x20, 0x21
// Slot 4 (Right Bank Row 1): P1_7 (G), P1_6 (R) -> Regs 0x2F, 0x2E
// Slot 5 (Right Bank Row 2): P1_5 (G), P1_4 (R) -> Regs 0x2D, 0x2C
// Slot 6 (Right Bank Row 3): P0_7 (G), P0_6 (R) -> Regs 0x27, 0x26
const uint8_t AW_LED_REG[NUM_SLOTS][2] = {
  { 0x28, 0x29 }, // Slot 1: Pin 1 (G), Pin 2 (R)
  { 0x2A, 0x2B }, // Slot 2: Pin 3 (G), Pin 4 (R)
  { 0x20, 0x21 }, // Slot 3: Pin 5 (G), Pin 6 (R)
  { 0x2F, 0x2E }, // Slot 4: Pin 17 (G), Pin 16 (R)
  { 0x2D, 0x2C }, // Slot 5: Pin 15 (G), Pin 14 (R)
  { 0x27, 0x26 }, // Slot 6: Pin 13 (G), Pin 12 (R)
};

void writeAW9523(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(AW9523_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void setSlotLED(uint8_t slot, uint8_t red, uint8_t green) {
  if (slot >= NUM_SLOTS) return;
  writeAW9523(AW_LED_REG[slot][0], green);
  writeAW9523(AW_LED_REG[slot][1], red);
}

void initAW9523() {
  writeAW9523(AW_REG_SW_RESET, 0x00);
  delay(5);
  // Configure Port 0 and Port 1 into LED dimming mode
  writeAW9523(AW_REG_P0_MODE, 0x00);
  writeAW9523(AW_REG_P1_MODE, 0x00);
  // Set global current to 2/4 IMAX (~18.5mA max constant-current)
  writeAW9523(AW_REG_GCR, 0x02);

  for (uint8_t i = 0; i < NUM_SLOTS; i++) {
    setSlotLED(i, 0, 0);
  }
}

enum CellMode : uint8_t {
  MODE_LIPO_4V20    = 0,  // 4200 mV
  MODE_LIHV_4V35    = 1,  // 4350 mV
  MODE_STORAGE_3V80 = 2   // 3800 mV
};

enum SlotState : uint8_t {
  STATE_EMPTY    = 0,
  STATE_READY    = 1,
  STATE_CHARGING = 2,
  STATE_DONE     = 3,
  STATE_FAULT    = 4
};

struct SlotConfig {
  uint16_t chargeCurrent_mA; // 50..500 mA
  CellMode mode;             // 4.20V LiPo, 4.35V LiHV, or 3.80V Storage
  bool     autoStart;        // Start charging automatically on plug-in
};

struct SlotRuntime {
  uint16_t  voltage_mV;
  SlotState state;
  bool      batteryPresent;
  bool      chargingEnabled;
  bool      startRequested;
  bool      stopRequested;
  uint8_t   i2cFailures;
  uint32_t  plugInTimestamp;
};

SlotConfig  slotCfg[NUM_SLOTS];
SlotRuntime slotRt[NUM_SLOTS];
Preferences prefs;

Adafruit_SSD1306  display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

SemaphoreHandle_t stateMutex = nullptr;
bool oledPresent = false;
int8_t selectedSlotUI = 0; // -1 = Overview, 0..5 = Slot Detail

// --- BLE UUIDs ---
#define SERVICE_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHAR_TELEMETRY_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define CHAR_COMMAND_UUID   "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

BLECharacteristic *pTelemetryChar = nullptr;
bool bleClientConnected = false;

class LockGuard {
 public:
  LockGuard() { xSemaphoreTake(stateMutex, portMAX_DELAY); }
  ~LockGuard() { xSemaphoreGive(stateMutex); }
};

// --- Low-level I2C ---
bool selectI2CMuxChannel(uint8_t slot) {
  if (slot >= NUM_SLOTS) return false;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << I2C_MUX_CHANNEL[slot]);
  return Wire.endTransmission() == 0;
}

void deselectI2CMux() {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
}

bool bqWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BQ25188_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool bqReadReg(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(BQ25188_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)BQ25188_ADDR, (uint8_t)1) != 1) return false;
  val = Wire.read();
  return true;
}

bool bqWriteVerified(uint8_t reg, uint8_t val) {
  uint8_t readBack = 0;
  return bqWriteReg(reg, val) && bqReadReg(reg, readBack) && readBack == val;
}

uint8_t encodeChargeCurrent(uint16_t current_mA) {
  if (current_mA < 5) current_mA = 5;
  if (current_mA > 500) current_mA = 500;
  if (current_mA <= 35) {
    return (uint8_t)(current_mA - 5);
  }
  return (uint8_t)(31 + ((current_mA - 40) / 10));
}

uint8_t encodeTargetVoltage(CellMode mode) {
  uint16_t target_mV = 4200;
  if (mode == MODE_LIHV_4V35)    target_mV = 4350;
  if (mode == MODE_STORAGE_3V80) target_mV = STORAGE_MV;
  return (uint8_t)((target_mV - 3500) / 10);
}

bool configureSlotBQ25188(uint8_t slot, bool enableCharge) {
  uint8_t ichgCode = encodeChargeCurrent(slotCfg[slot].chargeCurrent_mA) & 0x7F;
  bool ok = selectI2CMuxChannel(slot)
         && bqWriteReg(BQ_REG_ICHG_CTRL, ichgCode | 0x80)
         && bqWriteVerified(BQ_REG_IC_CTRL, BQ_IC_CTRL_VALUE)
         && bqWriteVerified(BQ_REG_TMR_ILIM, BQ_TMR_ILIM_VALUE)
         && bqWriteVerified(BQ_REG_VBAT_CTRL, encodeTargetVoltage(slotCfg[slot].mode) & 0x7F);
  if (ok && enableCharge) {
    uint8_t stale = 0;
    ok = bqReadReg(BQ_REG_STAT1, stale) && bqReadReg(BQ_REG_FLAG0, stale)
      && bqWriteVerified(BQ_REG_ICHG_CTRL, ichgCode);
  }
  deselectI2CMux();
  slotRt[slot].chargingEnabled = ok && enableCharge;
  return ok;
}

bool holdSlotDisabled(uint8_t slot) {
  uint8_t ichgCode = encodeChargeCurrent(slotCfg[slot].chargeCurrent_mA) & 0x7F;
  bool ok = selectI2CMuxChannel(slot) && bqWriteReg(BQ_REG_ICHG_CTRL, ichgCode | 0x80);
  deselectI2CMux();
  return ok;
}

uint32_t committedCurrent_mA(uint8_t slot) {
  uint32_t total = slotCfg[slot].chargeCurrent_mA;
  for (uint8_t i = 0; i < NUM_SLOTS; i++) {
    if (i != slot && slotRt[i].chargingEnabled) total += slotCfg[i].chargeCurrent_mA;
  }
  return total;
}

void setFault(uint8_t slot) {
  slotRt[slot].state = STATE_FAULT;
  slotRt[slot].chargingEnabled = false;
  slotRt[slot].startRequested = false;
}

bool tryStartCharging(uint8_t slot) {
  uint16_t v = slotRt[slot].voltage_mV;
  if (v < BATTERY_PRESENT_MV || v > BATTERY_MAX_MV) return false;
  if (slotCfg[slot].mode == MODE_STORAGE_3V80 && v >= STORAGE_MV) {
    slotRt[slot].state = STATE_DONE;
    return true;
  }
  if (committedCurrent_mA(slot) > TOTAL_CHARGE_LIMIT_MA) return false;
  if (!configureSlotBQ25188(slot, true)) {
    setFault(slot);
    return false;
  }
  slotRt[slot].state = STATE_CHARGING;
  return true;
}

// 16x oversampling and median filter on high-impedance 50k divider
uint16_t readSlotVoltage_mV(uint8_t slot) {
  if (slot >= NUM_SLOTS) return 0;
  uint8_t channel = ADC_MUX_CHANNEL[slot];
  digitalWrite(PIN_MUX_A, (channel & 0x01) ? HIGH : LOW);
  digitalWrite(PIN_MUX_B, (channel & 0x02) ? HIGH : LOW);
  digitalWrite(PIN_MUX_C, (channel & 0x04) ? HIGH : LOW);
  delayMicroseconds(200);

  uint32_t samples[16];
  for (int i = 0; i < 16; i++) {
    samples[i] = analogReadMilliVolts(PIN_ADC_MUX_OUT);
    delayMicroseconds(20);
  }
  // Simple insertion sort to pick median
  for (int i = 0; i < 15; i++) {
    for (int j = i + 1; j < 16; j++) {
      if (samples[j] < samples[i]) {
        uint32_t t = samples[i];
        samples[i] = samples[j];
        samples[j] = t;
      }
    }
  }
  // Average middle 8 samples
  uint32_t avg = 0;
  for (int i = 4; i < 12; i++) avg += samples[i];
  avg /= 8;
  return (uint16_t)(avg * 2); // 2:1 divider
}

void updateLEDs() {
  bool flashOn = (millis() / 500) % 2;
  for (uint8_t i = 0; i < NUM_SLOTS; i++) {
    uint8_t r = 0, g = 0;
    switch (slotRt[i].state) {
      case STATE_EMPTY:
        r = 0; g = 0;                         // Off = Slot Empty
        break;
      case STATE_READY:
        r = 15; g = 15;                       // Dim Yellow = Battery Present, Ready
        break;
      case STATE_CHARGING:
        if (slotCfg[i].mode == MODE_STORAGE_3V80) {
          r = 75; g = 35;                     // Solid Amber/Yellow = Storage 3.80V Mode
        } else if (slotCfg[i].mode == MODE_LIHV_4V35) {
          r = 180; g = 0;                     // Bright Solid Red = LiHV 4.35V Charging
        } else {
          r = 85; g = 0;                      // Solid Red = LiPo 4.20V Charging
        }
        break;
      case STATE_DONE:
        r = 0; g = 85;                        // Solid Green = Charge Complete
        break;
      case STATE_FAULT:
        r = flashOn ? 120 : 0; g = 0;         // Flashing Red = Fault Condition
        break;
    }
    setSlotLED(i, r, g);
  }
}

void scanSlot(uint8_t i) {
  SlotRuntime &rt = slotRt[i];
  rt.voltage_mV = readSlotVoltage_mV(i);

  if (!rt.chargingEnabled) {
    if (holdSlotDisabled(i)) {
      rt.i2cFailures = 0;
    } else if (++rt.i2cFailures >= I2C_FAIL_LIMIT) {
      setFault(i);
    }
  }

  if (rt.state == STATE_FAULT) {
    if (rt.batteryPresent && rt.voltage_mV < BATTERY_GONE_MV) {
      rt.batteryPresent = false;
      rt.state = STATE_EMPTY;
    }
    return;
  }

  if (!rt.batteryPresent && rt.voltage_mV > BATTERY_PRESENT_MV && rt.voltage_mV < BATTERY_MAX_MV) {
    rt.batteryPresent = true;
    rt.plugInTimestamp = millis();
    rt.state = STATE_READY;
    rt.startRequested = slotCfg[i].autoStart;
  } else if (rt.batteryPresent && rt.voltage_mV < BATTERY_GONE_MV) {
    rt.batteryPresent = false;
    rt.chargingEnabled = false;
    rt.startRequested = false;
    rt.state = STATE_EMPTY;
    return;
  }
  if (!rt.batteryPresent) {
    rt.startRequested = false;
    return;
  }

  if (rt.stopRequested) {
    rt.stopRequested = false;
    rt.startRequested = false;
    rt.chargingEnabled = false;
    rt.state = STATE_READY;
    return;
  }
  if (rt.startRequested && !rt.chargingEnabled) {
    if (tryStartCharging(i)) rt.startRequested = false;
    if (rt.state == STATE_FAULT) return;
  }

  if (rt.chargingEnabled) {
    uint8_t stat0 = 0, stat1 = 0, flag0 = 0;
    if (!selectI2CMuxChannel(i) || !bqReadReg(BQ_REG_STAT0, stat0)
        || !bqReadReg(BQ_REG_STAT1, stat1) || !bqReadReg(BQ_REG_FLAG0, flag0)) {
      deselectI2CMux();
      if (++rt.i2cFailures >= I2C_FAIL_LIMIT) {
        setFault(i);
        holdSlotDisabled(i);
      }
      return;
    }
    deselectI2CMux();
    rt.i2cFailures = 0;

    if ((stat1 & BQ_STAT1_SAFETY_TMR_FAULT) || (flag0 & BQ_FLAG0_FAULT_MASK)) {
      Serial.printf("Slot %u fault: STAT1 0x%02X FLAG0 0x%02X\n", i + 1, stat1, flag0);
      setFault(i);
      holdSlotDisabled(i);
      return;
    }

    uint8_t chg = BQ_CHG_STAT(stat0);
    if (chg == BQ_CHG_CC || chg == BQ_CHG_CV) {
      rt.state = STATE_CHARGING;
    } else if (chg == BQ_CHG_DONE_OR_DIS) {
      rt.state = STATE_DONE;
      rt.chargingEnabled = false;
      holdSlotDisabled(i);
    }
  }
}

void saveSlotConfig(uint8_t s) {
  char key[8];
  snprintf(key, sizeof(key), "s%u", s);
  prefs.putBytes(key, &slotCfg[s], sizeof(SlotConfig));
}

void loadSlotConfig(uint8_t s) {
  char key[8];
  snprintf(key, sizeof(key), "s%u", s);
  SlotConfig stored;
  if (prefs.getBytes(key, &stored, sizeof(stored)) == sizeof(stored)
      && stored.chargeCurrent_mA >= 50 && stored.chargeCurrent_mA <= 500
      && stored.mode <= MODE_STORAGE_3V80) {
    slotCfg[s] = stored;
  }
}

// Stream JSON telemetry to BLE app: {"s":1,"v":4180,"st":2,"ma":150,"m":0}
void sendBLETelemetry() {
  if (!bleClientConnected || pTelemetryChar == nullptr) return;

  static uint8_t curSlot = 0;
  char buf[80];
  snprintf(buf, sizeof(buf), "{\"s\":%u,\"v\":%u,\"st\":%u,\"ma\":%u,\"m\":%u}",
           curSlot + 1,
           slotRt[curSlot].voltage_mV,
           (uint8_t)slotRt[curSlot].state,
           slotCfg[curSlot].chargeCurrent_mA,
           (uint8_t)slotCfg[curSlot].mode);
  pTelemetryChar->setValue((uint8_t*)buf, strlen(buf));
  pTelemetryChar->notify();

  curSlot = (curSlot + 1) % NUM_SLOTS;
}

// Render GeeekPi 128x64 OLED dashboard
void updateOLED() {
  if (!oledPresent) return;

  deselectI2CMux();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Header line
  display.setCursor(0, 0);
  display.print("6-SLOT PD CHARGER");
  if (bleClientConnected) {
    display.setCursor(110, 0);
    display.print("BT");
  }
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  // 6 slots in a 2-column x 3-row layout
  // Column 0: Slots 1-3 (X=0..60), Column 1: Slots 4-6 (X=66..127)
  for (uint8_t i = 0; i < NUM_SLOTS; i++) {
    uint8_t col = (i < 3) ? 0 : 1;
    uint8_t row = i % 3;
    uint8_t x = col * 66;
    uint8_t y = 12 + row * 17;

    display.setCursor(x, y);
    display.printf("P%u:", i + 1);

    if (slotRt[i].state == STATE_EMPTY) {
      display.setCursor(x + 22, y);
      display.print("---");
    } else {
      display.setCursor(x + 22, y);
      display.printf("%.2fV", slotRt[i].voltage_mV / 1000.0f);
    }

    display.setCursor(x + 10, y + 8);
    switch (slotRt[i].state) {
      case STATE_EMPTY:    display.print("IDLE "); break;
      case STATE_READY:    display.print("RDY  "); break;
      case STATE_CHARGING:
        display.printf("%dmA", slotCfg[i].chargeCurrent_mA);
        break;
      case STATE_DONE:     display.print("DONE "); break;
      case STATE_FAULT:    display.print("FLT! "); break;
    }

    display.setCursor(x + 44, y + 8);
    if (slotCfg[i].mode == MODE_LIHV_4V35)      display.print("HV");
    else if (slotCfg[i].mode == MODE_STORAGE_3V80) display.print("ST");
    else                                       display.print("LP");
  }

  display.display();
}

// Handle tactile buttons: SW1 (UP), SW2 (SELECT), SW3 (DOWN)
void handleButtons() {
  static uint32_t lastBtnCheck = 0;
  if (millis() - lastBtnCheck < 150) return;
  lastBtnCheck = millis();

  bool btnUp = (digitalRead(PIN_BTN_UP) == LOW);
  bool btnSel = (digitalRead(PIN_BTN_SELECT) == LOW);
  bool btnDown = (digitalRead(PIN_BTN_DOWN) == LOW);

  if (btnUp) {
    LockGuard lock;
    // Cycle charge current: 50mA -> 150mA -> 300mA -> 500mA -> 50mA
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
      if (slotRt[i].state == STATE_READY || slotRt[i].state == STATE_EMPTY) {
        if (slotCfg[i].chargeCurrent_mA >= 500) slotCfg[i].chargeCurrent_mA = 50;
        else if (slotCfg[i].chargeCurrent_mA == 50) slotCfg[i].chargeCurrent_mA = 150;
        else if (slotCfg[i].chargeCurrent_mA == 150) slotCfg[i].chargeCurrent_mA = 300;
        else slotCfg[i].chargeCurrent_mA = 500;
        saveSlotConfig(i);
      }
    }
  } else if (btnDown) {
    LockGuard lock;
    // Cycle charge mode: LiPo (4.20V) -> LiHV (4.35V) -> Storage (3.80V)
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
      if (slotRt[i].state == STATE_READY || slotRt[i].state == STATE_EMPTY) {
        slotCfg[i].mode = (CellMode)((slotCfg[i].mode + 1) % 3);
        saveSlotConfig(i);
      }
    }
  } else if (btnSel) {
    LockGuard lock;
    // Toggle manual start on connected cells waiting in READY state
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
      if (slotRt[i].state == STATE_READY) {
        configureAndStartSlot(i);
      }
    }
  }
}

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    String cmd = pChar->getValue().c_str();
    int s, ma, mode, autoSt, runNow;
    if (!cmd.startsWith("SET,")) return;
    if (sscanf(cmd.c_str(), "SET,%d,%d,%d,%d,%d", &s, &ma, &mode, &autoSt, &runNow) != 5) return;
    if (s < 0 || s >= NUM_SLOTS || mode < 0 || mode > MODE_STORAGE_3V80) return;

    LockGuard lock;
    uint16_t newCurrent = constrain(ma, 50, 500);
    bool changed = (slotCfg[s].chargeCurrent_mA != newCurrent) ||
                   (slotCfg[s].mode != (CellMode)mode) ||
                   (slotCfg[s].autoStart != (autoSt != 0));
    slotCfg[s].chargeCurrent_mA = newCurrent;
    slotCfg[s].mode = (CellMode)mode;
    slotCfg[s].autoStart = (autoSt != 0);

    // Save only on change to protect flash memory endurance
    if (changed) saveSlotConfig(s);

    if (changed && slotRt[s].chargingEnabled) slotRt[s].stopRequested = true;
    if (runNow != 0) {
      slotRt[s].startRequested = true;
    } else {
      slotRt[s].startRequested = false;
      slotRt[s].stopRequested = true;
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { bleClientConnected = true; }
  void onDisconnect(BLEServer *) override {
    bleClientConnected = false;
    BLEDevice::startAdvertising();
  }
};

void setup() {
  Serial.begin(115200);
  stateMutex = xSemaphoreCreateMutex();

  pinMode(PIN_MUX_A, OUTPUT);
  pinMode(PIN_MUX_B, OUTPUT);
  pinMode(PIN_MUX_C, OUTPUT);
  pinMode(PIN_BTN_SELECT, INPUT_PULLUP);
  pinMode(PIN_BTN_UP, INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN, INPUT_PULLUP);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_ADC_MUX_OUT, ADC_11db);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  deselectI2CMux();

  // Initialize GeeekPi 0.96" SSD1306 OLED
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    oledPresent = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(15, 24);
    display.print("6-SLOT CHARGER");
    display.setCursor(25, 36);
    display.print("INITIALIZING...");
    display.display();
  } else {
    Serial.println("SSD1306 OLED not found at 0x3C");
  }

  initAW9523();

  prefs.begin("charger", false);

  for (uint8_t i = 0; i < NUM_SLOTS; i++) {
    slotCfg[i].chargeCurrent_mA = 150;
    slotCfg[i].mode = MODE_LIPO_4V20;
    slotCfg[i].autoStart = true;
    loadSlotConfig(i);
    slotRt[i] = SlotRuntime{};
    slotRt[i].state = STATE_EMPTY;
    {
      LockGuard lock;
      if (!configureSlotBQ25188(i, false)) {
        Serial.printf("Slot %u: charger not responding\n", i + 1);
        setFault(i);
      }
    }
  }

  BLEDevice::init("LiPo-6Slot-Charger");
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  BLESecurity *pSecurity = new BLESecurity();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
  pSecurity->setCapability(ESP_IO_CAP_NONE);
  pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTelemetryChar = pService->createCharacteristic(
    CHAR_TELEMETRY_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pTelemetryChar->addDescriptor(new BLE2902());

  BLECharacteristic *pCmdChar = pService->createCharacteristic(
    CHAR_COMMAND_UUID, BLECharacteristic::PROPERTY_WRITE
  );
  pCmdChar->setAccessPermissions(ESP_GATT_PERM_WRITE_ENCRYPTED);
  pCmdChar->setCallbacks(new CommandCallbacks());
  pService->start();
  BLEDevice::getAdvertising()->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();

  updateLEDs();
  updateOLED();
}

void loop() {
  static uint32_t lastScan = 0;
  static uint32_t lastUI = 0;
  static uint32_t lastTelemetry = 0;

  handleButtons();

  if (millis() - lastScan >= 250) {
    lastScan = millis();
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
      LockGuard lock;
      scanSlot(i);
    }
    updateLEDs();
  }

  if (millis() - lastUI >= 100) {
    lastUI = millis();
    updateOLED();
  }

  if (millis() - lastTelemetry >= 200) {
    lastTelemetry = millis();
    sendBLETelemetry();
  }
}
