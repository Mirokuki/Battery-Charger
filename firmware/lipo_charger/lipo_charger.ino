/*
 * 6-slot 1S / 2S LiPo charger - ESP32-C3-WROOM-02 + 6 x BQ25798
 *
 * Board: "ESP32C3 Dev Module", USB CDC On Boot: Enabled
 * Libraries: Adafruit GFX, Adafruit SSD1306
 *
 * Each slot has a PicoBlade (1S) and an XT30 (2S) connector.  Each connector
 * reaches the charger through its own back-to-back FET switch, driven by the
 * charger's D+ (PicoBlade) and D- (XT30) output drivers.  The firmware never
 * closes both switches of a slot at once.
 *
 * Detection: with charging off, the firmware bleeds the charger's BAT node to
 * 0 V, closes one switch, and reads the pack voltage with the charger's ADC.
 *   PicoBlade 2.5-4.45 V  -> 1S
 *   XT30      5.0-8.9 V   -> 2S
 *   XT30      2.5-4.45 V  -> ambiguous (1S on XT30, or a flat 2S): asks first
 *   both connectors live   -> fault, nothing is charged
 *
 * Balancing: a 2S pack's balance lead goes into the slot's JST-XH port.  A
 * BQ29209 there bleeds the higher cell (about 10 mA) until both match, with no
 * firmware involved.  If either cell passes 4.30 V it pulls the charger's TS pin
 * low, which stops charging in hardware and shows up here as "cell>4.3".  That
 * cut-off is ignored in LiHV mode, where 4.35 V per cell is the target.
 *
 * Cell voltages: three ADS1115 ADCs (0x48-0x4A on the main bus, two slots each)
 * read each balance port through 1 M dividers: cell junction / 2 and pack / 3.
 * They are shown per cell and checked against the target while charging.
 *
 * Safety: CE of every charger is pulled high by hardware, so a reset or
 * crashed MCU stops all charging.  The charger's own safety timer, input
 * limits and over-voltage protection stay enabled.
 */

#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_task_wdt.h>

// ---------------------------------------------------------------- pins
const int PIN_CE_N = 1, PIN_BUZZ = 3, PIN_SDA = 4, PIN_SCL = 5, PIN_MUX_RST = 6;
const int PIN_UP = 8, PIN_OK = 9, PIN_DN = 10;

const uint8_t MUX_ADDR = 0x70, BQ_ADDR = 0x6B, OLED_ADDR = 0x3C;
const uint8_t ADS_ADDR[3] = {0x48, 0x49, 0x4A};
const int SLOTS = 6;

// ---------------------------------------------------------------- BQ25798 registers
enum : uint8_t {
  R_VREG = 0x01, R_ICHG = 0x03, R_VINDPM = 0x05, R_IINDPM = 0x06, R_PRECHG = 0x08,
  R_TERM = 0x09, R_RECHG = 0x0A, R_TIMER = 0x0E, R_CTRL0 = 0x0F, R_CTRL1 = 0x10,
  R_CTRL2 = 0x11, R_CTRL5 = 0x14, R_NTC1 = 0x18, R_STAT0 = 0x1B, R_STAT1 = 0x1C, R_STAT4 = 0x1F,
  R_FAULT0 = 0x20, R_FAULT1 = 0x21, R_ADC_CTRL = 0x2E, R_ADC_DIS0 = 0x2F, R_ADC_DIS1 = 0x30,
  R_IBAT = 0x33, R_VBUS = 0x35, R_VBAT = 0x3B, R_DPDM = 0x47, R_PART = 0x48
};
const uint8_t CTRL0_IBATDIS = 0x40, CTRL0_EN_CHG = 0x20, CTRL0_EN_TERM = 0x02;
const uint8_t DPDM_PB_ON = 6 << 5;   // D+ = 3.3 V closes the PicoBlade switch
const uint8_t DPDM_XT_ON = 6 << 2;   // D- = 3.3 V closes the XT30 switch

// ---------------------------------------------------------------- limits
const uint16_t CELL_MIN_MV = 2500;      // below this per cell: refuse
const uint16_t CELL_MAX_MV = 4450;      // above this per cell: refuse
const uint16_t RATE_MIN_MA = 50, RATE_MAX_MA = 1000, RATE_STEP_MA = 10;
const uint16_t MODE_MV[3] = {4200, 4350, 3850};
const char *MODE_NAME[3] = {"LiPo 4.20", "LiHV 4.35", "Store 3.85"};

enum Conn : uint8_t { CONN_NONE, CONN_PB, CONN_XT };
enum State : uint8_t { ST_ABSENT, ST_EMPTY, ST_ASK, ST_CHARGE, ST_DONE, ST_STOPPED, ST_FAULT };
enum Probe : uint8_t { PR_BLEED1, PR_PB, PR_BLEED2, PR_XT, PR_DECIDE };

struct Slot {
  State state = ST_ABSENT;
  Probe probe = PR_BLEED1;
  Conn conn = CONN_NONE;
  uint8_t cells = 0;
  uint16_t vbat = 0, vpb = 0, vxt = 0;
  uint16_t cell1 = 0, cell2 = 0;   // from the balance port, mV (valid while balLead)
  bool balLead = false;
  int16_t ibat = 0;
  uint16_t setMa = 0;          // current actually programmed
  float mAh = 0;
  uint32_t tStep = 0, tStart = 0, tLowI = 0, tCheck = 0;
  const char *fault = "";
  // user settings
  uint8_t mode1 = 0, mode2 = 0;
  uint16_t rate1 = 150, rate2 = 350;
};

Slot slot[SLOTS];
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
Preferences prefs;
uint16_t vbusMv = 0;
uint16_t budgetMw = 7000;

// ---------------------------------------------------------------- I2C helpers
bool muxSelect(int ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(ch < 0 ? 0 : (1 << ch));
  return Wire.endTransmission() == 0;
}
bool bqWrite8(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg); Wire.write(v);
  return Wire.endTransmission() == 0;
}
bool bqWrite16(uint8_t reg, uint16_t v) {
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg); Wire.write(v >> 8); Wire.write(v & 0xFF);
  return Wire.endTransmission() == 0;
}
int bqRead8(uint8_t reg) {
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((int)BQ_ADDR, 1) != 1) return -1;
  return Wire.read();
}
long bqRead16(uint8_t reg) {
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((int)BQ_ADDR, 2) != 2) return -1;
  uint16_t hi = Wire.read();
  return (hi << 8) | Wire.read();
}
bool bqUpdate(uint8_t reg, uint8_t mask, uint8_t val) {
  int cur = bqRead8(reg);
  if (cur < 0) return false;
  return bqWrite8(reg, (cur & ~mask) | (val & mask));
}

void beep(int ms, int hz = 2700) { tone(PIN_BUZZ, hz, ms); }

// ---------------------------------------------------------------- cell-voltage ADCs
// One single-shot conversion at a time, round-robin over the 12 inputs (about 8 per second
// in total).  Sampling this rarely keeps the ADC from loading the 1 M dividers.
bool adsStart(uint8_t addr, uint8_t ch) {
  uint16_t cfg = 0x8000 | ((4 + ch) << 12) | 0x0200 | 0x0100 | 0x0080 | 0x0003;   // AINch-GND, +-4.096 V, 128 SPS
  Wire.beginTransmission(addr);
  Wire.write(0x01); Wire.write(cfg >> 8); Wire.write(cfg & 0xFF);
  return Wire.endTransmission() == 0;
}
long adsReadMv(uint8_t addr) {                // input voltage in mV, or -1
  Wire.beginTransmission(addr);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((int)addr, 2) != 2) return -1;
  int16_t raw = (Wire.read() << 8) | Wire.read();
  if (raw < 0) raw = 0;
  return (long)raw / 8;                       // 125 uV per count
}
void adcStep() {
  static uint8_t idx = 0;
  static bool pending = false;
  static uint32_t t0 = 0, tNext = 0;
  static long midMv[SLOTS], packMv[SLOTS];
  uint32_t now = millis();
  uint8_t chip = idx / 4, ch = idx % 4, sl = chip * 2 + ch / 2;
  if (!pending) {
    if (now < tNext) return;
    pending = adsStart(ADS_ADDR[chip], ch);
    t0 = now;
    if (!pending) { slot[sl].balLead = false; idx = (idx + 1) % 12; tNext = now + 100; }
    return;
  }
  if (now - t0 < 12) return;
  long mv = adsReadMv(ADS_ADDR[chip]);
  pending = false;
  tNext = now + 110;
  if (ch % 2 == 0) midMv[sl] = mv < 0 ? -1 : mv * 2;
  else {
    packMv[sl] = mv < 0 ? -1 : mv * 3;
    Slot &s = slot[sl];
    if (midMv[sl] > 2000 && packMv[sl] > midMv[sl] + 2000) {
      s.cell1 = midMv[sl];
      s.cell2 = packMv[sl] - midMv[sl];
      s.balLead = true;
    } else s.balLead = false;
  }
  idx = (idx + 1) % 12;
}

// ---------------------------------------------------------------- charger control
// All of these expect muxSelect(slot) to have been called.
bool chargerOff() {             // stop charging, open both switches
  bool ok = bqUpdate(R_CTRL0, CTRL0_EN_CHG | CTRL0_IBATDIS, 0);
  return bqWrite8(R_DPDM, 0) && ok;
}

bool chargerInit() {
  int part = bqRead8(R_PART);
  if (part < 0 || ((part >> 3) & 7) != 3) return false;       // not a BQ25798
  bool ok = bqWrite8(R_DPDM, 0);
  ok &= bqUpdate(R_CTRL0, CTRL0_EN_CHG | CTRL0_IBATDIS | 0x10, 0);   // no charge, no ICO
  ok &= bqUpdate(R_CTRL1, 0x07, 0x00);        // I2C watchdog off: settings must never fall back to the 1 A default
  ok &= bqUpdate(R_CTRL2, 0x78, 0x00);        // no D+/D- adapter detection, no HVDCP
  ok &= bqUpdate(R_CTRL5, 0x02, 0x00);        // ILIM pin unused, register limit only
  ok &= bqUpdate(R_NTC1, 0x01, 0x01);         // TS ignored until a 2S LiPo charge needs the cell cut-off
  ok &= bqWrite8(R_TIMER, 0x39);              // top-off 0, all timers on, 5 h fast-charge limit, 2x slowdown
  ok &= bqWrite8(R_ADC_DIS0, 0x04);           // TS channel off
  ok &= bqWrite8(R_ADC_DIS1, 0xF0);           // D+/D-/VAC channels off
  ok &= bqWrite8(R_ADC_CTRL, 0x80);           // ADC on, continuous, 15 bit
  return ok;
}

bool chargerProgram(uint8_t cells, uint16_t cellMv, uint16_t ma) {
  bool ok = bqUpdate(R_CTRL0, CTRL0_EN_CHG, 0);
  int rechg = bqRead8(R_RECHG);
  if (rechg < 0) return false;
  uint8_t want = (cells - 1) << 6;
  if ((rechg & 0xC0) != want)                 // changing CELL resets VREG/ICHG to defaults,
    ok &= bqWrite8(R_RECHG, (rechg & 0x3F) | want);   // so it is always written first
  uint16_t vreg = (uint32_t)cellMv * cells / 10;
  ok &= bqWrite16(R_VREG, vreg);
  ok &= bqWrite16(R_ICHG, ma / 10);
  ok &= bqWrite8(R_PRECHG, 0xC0 | 1);         // precharge below 71 % of VREG at 40 mA
  ok &= bqWrite8(R_TERM, 1);                  // terminate at 40 mA (chip minimum)
  uint8_t topoff = ma < 400 ? 0x40 : 0x00;    // small packs: 15 min top-off after the 40 mA point
  ok &= bqWrite8(R_TIMER, 0x39 | topoff);
  ok &= bqWrite16(R_IINDPM, 100);             // 1 A input cap per slot
  ok &= bqWrite8(R_VINDPM, vbusMv > 7500 ? 78 : 44);   // back off if the adapter sags
  // TS carries the balancer's "a cell is above 4.30 V" flag.  Honour it for 2S up to 4.20 V/cell.
  bool cellCutoff = cells == 2 && cellMv <= 4250;
  ok &= bqUpdate(R_NTC1, 0x01, cellCutoff ? 0x00 : 0x01);
  // read back the two values that matter before anything is enabled
  if (bqRead16(R_VREG) != vreg || bqRead16(R_ICHG) != ma / 10) return false;
  return ok;
}

// ---------------------------------------------------------------- settings
void loadSettings() {
  prefs.begin("chg", false);
  for (int i = 0; i < SLOTS; i++) {
    char k[8];
    snprintf(k, sizeof k, "m1_%d", i); slot[i].mode1 = prefs.getUChar(k, 0) % 3;
    snprintf(k, sizeof k, "m2_%d", i); slot[i].mode2 = prefs.getUChar(k, 0) % 3;
    snprintf(k, sizeof k, "r1_%d", i); slot[i].rate1 = constrain(prefs.getUShort(k, 150), RATE_MIN_MA, RATE_MAX_MA);
    snprintf(k, sizeof k, "r2_%d", i); slot[i].rate2 = constrain(prefs.getUShort(k, 350), RATE_MIN_MA, RATE_MAX_MA);
  }
}
void saveSettings(int i) {
  char k[8];
  snprintf(k, sizeof k, "m1_%d", i); prefs.putUChar(k, slot[i].mode1);
  snprintf(k, sizeof k, "m2_%d", i); prefs.putUChar(k, slot[i].mode2);
  snprintf(k, sizeof k, "r1_%d", i); prefs.putUShort(k, slot[i].rate1);
  snprintf(k, sizeof k, "r2_%d", i); prefs.putUShort(k, slot[i].rate2);
}
uint16_t cellTarget(const Slot &s) { return MODE_MV[s.cells == 2 ? s.mode2 : s.mode1]; }
uint16_t userRate(const Slot &s) { return s.cells == 2 ? s.rate2 : s.rate1; }

// ---------------------------------------------------------------- power budget
// Scale every charging slot down by the same factor if the adapter cannot feed them all.
uint16_t budgetedRate(int i) {
  uint32_t want = 0;
  for (int k = 0; k < SLOTS; k++)
    if (slot[k].state == ST_CHARGE || k == i)
      want += (uint32_t)userRate(slot[k]) * cellTarget(slot[k]) * slot[k].cells / 1000;   // mW at the pack
  want = want * 100 / 85;                                                              // converter loss
  uint32_t ma = userRate(slot[i]);
  if (want > budgetMw) ma = ma * budgetMw / want;
  ma = (ma / 10) * 10;
  return max<uint32_t>(ma, RATE_MIN_MA);
}

// ---------------------------------------------------------------- slot state machine
void setFault(int i, const char *why) {
  muxSelect(i);
  chargerOff();
  slot[i].state = ST_FAULT;
  slot[i].fault = why;
  slot[i].tCheck = millis();
  beep(600, 1500);
  Serial.printf("slot %d fault: %s\n", i + 1, why);
}

void startCharge(int i) {
  Slot &s = slot[i];
  muxSelect(i);
  uint16_t target = cellTarget(s);
  if (s.vbat >= (uint32_t)target * s.cells - 30 * s.cells) {       // already at or above the target
    chargerOff();
    bqWrite8(R_DPDM, s.conn == CONN_PB ? DPDM_PB_ON : DPDM_XT_ON);
    s.state = ST_DONE; s.mAh = 0;
    return;
  }
  s.setMa = budgetedRate(i);
  if (!chargerProgram(s.cells, target, s.setMa)) { setFault(i, "I2C/program"); return; }
  if (!bqWrite8(R_DPDM, s.conn == CONN_PB ? DPDM_PB_ON : DPDM_XT_ON)) { setFault(i, "I2C"); return; }
  delay(30);
  if (!bqUpdate(R_CTRL0, CTRL0_EN_CHG | CTRL0_EN_TERM, CTRL0_EN_CHG | CTRL0_EN_TERM)) { setFault(i, "I2C"); return; }
  s.state = ST_CHARGE; s.mAh = 0; s.tStart = s.tStep = millis(); s.tLowI = 0;
  Serial.printf("slot %d: %dS to %u mV/cell at %u mA\n", i + 1, s.cells, target, s.setMa);
}

// One detection step for an empty slot; the steps are 300 ms apart so the ADC has fresh data.
void probeStep(int i) {
  Slot &s = slot[i];
  muxSelect(i);
  switch (s.probe) {
    case PR_BLEED1:
    case PR_BLEED2:
      bqWrite8(R_DPDM, 0);
      bqUpdate(R_CTRL0, CTRL0_EN_CHG | CTRL0_IBATDIS, CTRL0_IBATDIS);   // 30 mA load empties the BAT caps
      delay(40);
      bqUpdate(R_CTRL0, CTRL0_IBATDIS, 0);
      bqWrite8(R_DPDM, s.probe == PR_BLEED1 ? DPDM_PB_ON : DPDM_XT_ON);
      s.probe = s.probe == PR_BLEED1 ? PR_PB : PR_XT;
      break;
    case PR_PB:
      s.vpb = max(0L, bqRead16(R_VBAT));
      bqWrite8(R_DPDM, 0);
      s.probe = PR_BLEED2;
      break;
    case PR_XT:
      s.vxt = max(0L, bqRead16(R_VBAT));
      bqWrite8(R_DPDM, 0);
      s.probe = PR_DECIDE;
      break;
    case PR_DECIDE: {
      s.probe = PR_BLEED1;
      bool pb = s.vpb > 1500, xt = s.vxt > 1500;
      s.conn = CONN_NONE; s.cells = 0; s.vbat = 0;
      if (pb && xt) { setFault(i, "2 packs!"); return; }
      if (!pb && !xt) { s.state = ST_EMPTY; return; }
      s.conn = pb ? CONN_PB : CONN_XT;
      s.vbat = pb ? s.vpb : s.vxt;
      if (pb) {
        if (s.vbat > CELL_MAX_MV) { setFault(i, "not 1S!"); return; }
        if (s.vbat < CELL_MIN_MV) { setFault(i, "too low"); return; }
        s.cells = 1;
        startCharge(i);
      } else if (s.vbat >= 2 * CELL_MIN_MV && s.vbat <= 2 * CELL_MAX_MV) {
        s.cells = 2;
        startCharge(i);
      } else if (s.vbat >= CELL_MIN_MV && s.vbat <= CELL_MAX_MV) {
        s.cells = 1;                      // 1S on the XT30, or a flat 2S: let the user decide
        if (s.state != ST_ASK) beep(150);
        s.state = ST_ASK;
      } else {
        setFault(i, s.vbat > 2 * CELL_MAX_MV ? "over 2S!" : "too low");
      }
      break;
    }
  }
}

// Is a pack still on the closed switch?  Bleed the BAT node briefly: a pack holds its voltage.
bool packPresent(int i) {
  Slot &s = slot[i];
  bqUpdate(R_CTRL0, CTRL0_EN_CHG | CTRL0_IBATDIS, CTRL0_IBATDIS);
  delay(60);
  bqUpdate(R_CTRL0, CTRL0_IBATDIS, 0);
  delay(350);                                           // one full ADC round
  long v = bqRead16(R_VBAT);
  return v > (long)CELL_MIN_MV * s.cells - 300;
}

void slotStep(int i) {
  Slot &s = slot[i];
  uint32_t now = millis();
  if (now - s.tStep < 300) return;
  uint32_t dt = now - s.tStep;
  s.tStep = now;
  if (!muxSelect(i)) return;

  if (s.state == ST_ABSENT) {
    if (chargerInit()) s.state = ST_EMPTY;
    return;
  }
  if (bqRead8(R_PART) < 0) { s.state = ST_ABSENT; return; }       // charger stopped answering
  long vb = bqRead16(R_VBUS);
  if (vb > 0) vbusMv = vb;

  switch (s.state) {
    case ST_EMPTY:
    case ST_ASK:
      probeStep(i);
      break;

    case ST_CHARGE: {
      long v = bqRead16(R_VBAT), c = bqRead16(R_IBAT);
      int st = bqRead8(R_STAT1), f0 = bqRead8(R_FAULT0), f1 = bqRead8(R_FAULT1);
      if (v < 0 || c < 0 || st < 0) { setFault(i, "I2C"); return; }
      s.vbat = v; s.ibat = (int16_t)c;
      if (s.ibat > 0) s.mAh += s.ibat * (dt / 3600000.0f);
      uint32_t limit = (uint32_t)cellTarget(s) * s.cells;
      if (s.vbat > limit + 120) { setFault(i, "overvolt"); return; }
      if (s.ibat > (int)s.setMa + 150) { setFault(i, "overcurr"); return; }
      // BAT OVP/OCP, converter OCP, SYS short/OVP, thermal (ignored while the charger settles after a cell-count change)
      if (now - s.tStart > 3000 && ((f0 & 0x2C) || (f1 & 0xC4))) { setFault(i, "charger"); return; }
      if (s.mAh > 1500) { setFault(i, "capacity"); return; }
      if (s.cells == 2 && s.balLead) {                    // measured cell voltages
        uint16_t hi = max(s.cell1, s.cell2), lo = min(s.cell1, s.cell2);
        if (hi > cellTarget(s) + 80) { setFault(i, "cell high"); return; }
        if (lo < CELL_MIN_MV - 300) { setFault(i, "cell low"); return; }
      }
      if (s.cells == 2 && cellTarget(s) <= 4250) {       // balancer flagged a cell above 4.30 V
        int st4 = bqRead8(R_STAT4);
        if (st4 >= 0 && (st4 & 0x01)) { setFault(i, "cell>4.3"); return; }
      }
      int chg = (st >> 5) & 7;
      if (chg == 7) {                                   // charger says finished
        bqUpdate(R_CTRL0, CTRL0_EN_CHG, 0);
        if (packPresent(i)) { s.state = ST_DONE; beep(120); delay(160); beep(120); }
        else { chargerOff(); s.state = ST_EMPTY; s.probe = PR_BLEED1; }
        return;
      }
      // no current flowing: either pulled out or the charger has stopped (safety timer)
      if (s.ibat < 15 && now - s.tStart > 5000) {
        if (!s.tLowI) s.tLowI = now;
        if (now - s.tLowI > 4000 && chg != 6) {
          bqUpdate(R_CTRL0, CTRL0_EN_CHG, 0);
          if (packPresent(i)) setFault(i, chg == 0 ? "timeout" : "no curr");
          else { chargerOff(); s.state = ST_EMPTY; s.probe = PR_BLEED1; }
          return;
        }
      } else s.tLowI = 0;
      // follow the shared power budget
      uint16_t want = budgetedRate(i);
      if (want != s.setMa && bqWrite16(R_ICHG, want / 10)) s.setMa = want;
      break;
    }

    case ST_DONE:
    case ST_STOPPED: {
      long v = bqRead16(R_VBAT);
      if (v >= 0) s.vbat = v;
      s.ibat = 0;
      if (now - s.tCheck > 3000) {
        s.tCheck = now;
        if (!packPresent(i)) { chargerOff(); s.state = ST_EMPTY; s.probe = PR_BLEED1; }
      }
      break;
    }

    case ST_FAULT:                                        // clears once both connectors read empty
      if (s.probe == PR_DECIDE) {
        s.probe = PR_BLEED1;
        if (s.vpb < 1500 && s.vxt < 1500) { s.state = ST_EMPTY; s.conn = CONN_NONE; s.cells = 0; }
      } else {
        probeStep(i);                                     // closes one switch at a time, never charges
      }
      break;
    default: break;
  }
}

// ---------------------------------------------------------------- UI
enum Screen : uint8_t { SC_MAIN, SC_MENU };
Screen screen = SC_MAIN;
int cursor = 0, menuItem = 0;
bool editing = false;
const int MENU_ITEMS = 6;

struct Button { int pin; bool down; uint32_t t; uint32_t rep; };
Button btn[3] = {{PIN_UP, false, 0, 0}, {PIN_OK, false, 0, 0}, {PIN_DN, false, 0, 0}};

// returns 1 on press (with auto-repeat for UP / DOWN)
bool pressed(int b) {
  bool lvl = digitalRead(btn[b].pin) == LOW;
  uint32_t now = millis();
  if (lvl && !btn[b].down && now - btn[b].t > 30) { btn[b].down = true; btn[b].t = now; btn[b].rep = now + 450; return true; }
  if (!lvl && btn[b].down && now - btn[b].t > 30) { btn[b].down = false; btn[b].t = now; }
  if (lvl && btn[b].down && b != 1 && now > btn[b].rep) { btn[b].rep = now + 70; return true; }
  return false;
}

void adjust(Slot &s, int dir) {
  switch (menuItem) {
    case 0: s.mode1 = (s.mode1 + 3 + dir) % 3; break;
    case 1: s.rate1 = constrain((int)s.rate1 + dir * RATE_STEP_MA, RATE_MIN_MA, RATE_MAX_MA); break;
    case 2: s.mode2 = (s.mode2 + 3 + dir) % 3; break;
    case 3: s.rate2 = constrain((int)s.rate2 + dir * RATE_STEP_MA, RATE_MIN_MA, RATE_MAX_MA); break;
  }
}

void handleButtons() {
  bool up = pressed(0), ok = pressed(1), dn = pressed(2);
  Slot &s = slot[cursor];
  if (screen == SC_MAIN) {
    if (up) cursor = (cursor + SLOTS - 1) % SLOTS;
    if (dn) cursor = (cursor + 1) % SLOTS;
    if (ok) { screen = SC_MENU; menuItem = 0; editing = false; }
    return;
  }
  if (editing) {
    if (up) adjust(s, +1);
    if (dn) adjust(s, -1);
    if (ok) {
      editing = false;
      saveSettings(cursor);
      if (s.state == ST_CHARGE) { muxSelect(cursor); chargerOff(); s.state = ST_EMPTY; s.probe = PR_BLEED1; }   // restart with new values
    }
    return;
  }
  if (up) menuItem = (menuItem + MENU_ITEMS - 1) % MENU_ITEMS;
  if (dn) menuItem = (menuItem + 1) % MENU_ITEMS;
  if (ok) {
    if (menuItem <= 3) editing = true;
    else if (menuItem == 4) {                 // start / stop
      muxSelect(cursor);
      if (s.state == ST_CHARGE) { bqUpdate(R_CTRL0, CTRL0_EN_CHG, 0); s.state = ST_STOPPED; s.tCheck = millis(); }
      else if (s.state == ST_ASK || s.state == ST_STOPPED || s.state == ST_DONE) startCharge(cursor);
      screen = SC_MAIN;
    } else screen = SC_MAIN;
  }
}

const char *stateText(const Slot &s) {
  switch (s.state) {
    case ST_ABSENT: return "no chip";
    case ST_EMPTY: return "--";
    case ST_ASK: return "1S? OK";
    case ST_CHARGE: return "CHG";
    case ST_DONE: return "FULL";
    case ST_STOPPED: return "STOP";
    default: return s.fault;
  }
}

void draw() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  if (screen == SC_MAIN) {
    uint32_t mw = 0;
    for (auto &s : slot) if (s.state == ST_CHARGE) mw += (uint32_t)s.vbat * max<int>(s.ibat, 0) / 1000;
    oled.setCursor(0, 0);
    oled.printf("IN %u.%uV  OUT %u.%uW", vbusMv / 1000, (vbusMv % 1000) / 100, (unsigned)(mw / 1000), (unsigned)((mw % 1000) / 100));
    oled.drawFastHLine(0, 9, 128, SSD1306_WHITE);
    for (int i = 0; i < SLOTS; i++) {
      Slot &s = slot[i];
      int y = 11 + i * 9;
      if (i == cursor) { oled.fillRect(0, y - 1, 128, 9, SSD1306_WHITE); oled.setTextColor(SSD1306_BLACK); }
      else oled.setTextColor(SSD1306_WHITE);
      oled.setCursor(1, y);
      if (s.cells == 2 && s.balLead && s.state != ST_EMPTY && s.state != ST_FAULT) {
        oled.printf("%d 2S %u.%02u/%u.%02u ", i + 1, s.cell1 / 1000, (s.cell1 % 1000) / 10,
                    s.cell2 / 1000, (s.cell2 % 1000) / 10);
        if (s.state == ST_CHARGE) oled.printf("%dmA", s.ibat); else oled.print(stateText(s));
      } else if (s.cells && s.state != ST_EMPTY && s.state != ST_FAULT)
        oled.printf("%d %dS %u.%02uV %4dmA %s", i + 1, s.cells, s.vbat / 1000, (s.vbat % 1000) / 10,
                    s.state == ST_CHARGE ? s.ibat : 0, s.state == ST_CHARGE ? "" : stateText(s));
      else if (s.balLead && s.state == ST_EMPTY)          // balance lead only: still worth showing
        oled.printf("%d bal %u.%02u/%u.%02u", i + 1, s.cell1 / 1000, (s.cell1 % 1000) / 10,
                    s.cell2 / 1000, (s.cell2 % 1000) / 10);
      else
        oled.printf("%d %s", i + 1, stateText(s));
    }
  } else {
    Slot &s = slot[cursor];
    oled.setCursor(0, 0);
    if (s.balLead)
      oled.printf("S%d %u.%02u/%u.%02uV %umAh", cursor + 1, s.cell1 / 1000, (s.cell1 % 1000) / 10,
                  s.cell2 / 1000, (s.cell2 % 1000) / 10, (unsigned)s.mAh);
    else
      oled.printf("SLOT %d  %s %umAh", cursor + 1, stateText(s), (unsigned)s.mAh);
    oled.drawFastHLine(0, 9, 128, SSD1306_WHITE);
    for (int m = 0; m < MENU_ITEMS; m++) {
      int y = 11 + m * 9;
      bool sel = m == menuItem;
      if (sel && !editing) { oled.fillRect(0, y - 1, 128, 9, SSD1306_WHITE); oled.setTextColor(SSD1306_BLACK); }
      else oled.setTextColor(SSD1306_WHITE);
      oled.setCursor(1, y);
      switch (m) {
        case 0: oled.printf("1S volt %c%s", sel && editing ? '>' : ' ', MODE_NAME[s.mode1]); break;
        case 1: oled.printf("1S rate %c%u mA", sel && editing ? '>' : ' ', s.rate1); break;
        case 2: oled.printf("2S volt %c%s", sel && editing ? '>' : ' ', MODE_NAME[s.mode2]); break;
        case 3: oled.printf("2S rate %c%u mA", sel && editing ? '>' : ' ', s.rate2); break;
        case 4: oled.print(s.state == ST_CHARGE ? "Stop charging" : "Start charging"); break;
        case 5: oled.print("Back"); break;
      }
    }
  }
  oled.display();
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  pinMode(PIN_CE_N, OUTPUT);
  digitalWrite(PIN_CE_N, HIGH);               // chargers disabled until every one is configured
  pinMode(PIN_MUX_RST, OUTPUT);
  digitalWrite(PIN_MUX_RST, LOW);
  pinMode(PIN_BUZZ, OUTPUT);
  pinMode(PIN_UP, INPUT_PULLUP);
  pinMode(PIN_OK, INPUT_PULLUP);
  pinMode(PIN_DN, INPUT_PULLUP);
  Serial.begin(115200);
  delay(20);
  digitalWrite(PIN_MUX_RST, HIGH);
  Wire.begin(PIN_SDA, PIN_SCL, 100000);
  Wire.setTimeOut(20);

  muxSelect(-1);
  oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println("6x 1S/2S LiPo charger");
  oled.display();

  loadSettings();
  delay(300);                                 // chargers finish their power-up sequence
  for (int i = 0; i < SLOTS; i++) {
    muxSelect(i);
    if (chargerInit()) slot[i].state = ST_EMPTY;
    oled.printf("slot %d %s\n", i + 1, slot[i].state == ST_EMPTY ? "ok" : "NOT FOUND");
    oled.display();
    slot[i].tStep = millis() + i * 50;        // stagger the slots
  }
  delay(400);
  muxSelect(0);
  long vb = bqRead16(R_VBUS);
  if (vb > 0) vbusMv = vb;
  budgetMw = vbusMv > 7500 ? 18000 : 7000;    // 9 V PD (Samsung 25 W gives 20 W via PPS) or plain 5 V
  digitalWrite(PIN_CE_N, LOW);                // charging is now gated per slot by EN_CHG

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdt = {.timeout_ms = 8000, .idle_core_mask = 0, .trigger_panic = true};
  esp_task_wdt_reconfigure(&wdt);
#else
  esp_task_wdt_init(8, true);
#endif
  esp_task_wdt_add(NULL);
  beep(80);
}

void loop() {
  static uint32_t tDraw = 0;
  static int next = 0;
  esp_task_wdt_reset();
  handleButtons();
  slotStep(next);
  next = (next + 1) % SLOTS;
  muxSelect(-1);                              // OLED and the cell ADCs live on the upstream bus
  adcStep();
  if (millis() - tDraw > 150) { tDraw = millis(); draw(); }
}
