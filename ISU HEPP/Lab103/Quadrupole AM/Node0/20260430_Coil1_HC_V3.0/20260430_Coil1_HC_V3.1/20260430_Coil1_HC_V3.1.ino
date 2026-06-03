/*
  Elephont V3.1 - Arduino Mega 2560 Firmware
  
  General code for driving the single channel Quadrupole + Helmholtz coil controller.
  Quadrupole AM modulation--> 20260430_Coil1_HC_V3.1 --> V3.1

Afshin Mahmoudieh Champiry (ChatGPT)
HEPP ISU/ Lab 103
202604030

  Compatible with the Elephant V3.1 PyQt5 GUI.

  Protocol V2.0:
  PC -> Arduino: [0xAA][CMD][LEN][DATA...][CRC8]
  Arduino -> PC: [0x55][RSP][LEN][DATA...][CRC8]

  CRC8 is calculated over:
  [CMD/RSP][LEN][DATA...]


  SPI:
  D51 MOSI -> AD9833 SDATA + MCP41010 SI pins
  D52 SCK  -> AD9833 SCLK  + MCP41010 SCK pins
  D10      -> AD9833 FSYNC
  D9       -> MCP41010 #1 CS
  D8       -> MCP41010 #2 CS

  I2C OLED:
  D20 SDA
  D21 SCL

  Notes:
  - Clear Setpoints is GUI-only. Arduino receives new values only when Set Coil or Set HC is pressed.
  - Calibration constants below are placeholders. Adjust them to match your analog circuits.
  - Relay active level is configurable below.

  /*


Arduino MEGA2560

USART --> USB
Display OLED 0.96" (JMD0.96D-1) --> SPI
DDS signal generator (AD9833) --> SPI
Digital Potentiometer1 (MCP41010) ---> SPI
Digital Potentiometer2 (MCP41010) ---> SPI

  Main hardware map:

MEGA2560
|
|<INPUTs>
|
├── ADC0 ──────────> coil_dc_crt_msr
├── ADC1 ──────────> coil_ac_crt_msr
├── ADC2 ──────────> coil_temp_msr
├── ADC3 ──────────> hc_crt_msr
├── ADC4 ──────────> hc_temp_msr
|
├── D30  ──────────> coil_pw_msr
├── D31  ──────────> coil_pol_msr
├── D32  ──────────> hc_pw_msr
├── D33  ──────────> hc_pol_msr
├── D34  ──────────> emg_stop_msr
|
|<OUTPUTs>
├── D40 <────────── coil_pw_stat
├── D41 <────────── coil_pol_stat
├── D42 <────────── hc_pw_stat
├── D43 <────────── hc_pol_stat
├── D44 <────────── emg_stop_stat
|
├── DDS signal generator (AD9833) <- coil_mod_freq_stat and coil_wf_stat
├── Digital Potentiometer1 (MCP41010) <- coil_dc_crt_stat
├── Digital Potentiometer2 (MCP41010) <- coil_ac_crt_stat
├── Digital Potentiometer3 (MCP41010) <- hc_crt_stat
|
├── SPI BUS
│   ├── MOSI (51) ─────> AD9833 SDATA
│   │                 ├─> MCP41010 #1 SI
│   │                 ├─> MCP41010 #2 SI
│   │                 └─> MCP41010 #3 SI
│   │
│   ├── SCK (52) ─────> AD9833 SCLK
│   │                 ├─> MCP41010 #1 SCK
│   │                 ├─> MCP41010 #2 SCK
│   │                 └─> MCP41010 #3 SCK
│
├── CS Pins
│   ├── D10 ──────────> AD9833 FSYNC
│   ├── D9  ──────────> MCP41010 #1 CS
│   ├── D8  ──────────> MCP41010 #2 CS
│   └── D7  ──────────> MCP41010 #3 CS
│
├── I2C OLED
│   ├── SDA (20) ─────> OLED SDA
│   └── SCL (21) ─────> OLED SCL
│
└── Power
    ├── 5V  ──────────> All VCC/VDD
    └── GND ──────────> All Grounds

*/

/*
  Elephant V3.1 - Arduino Mega 2560 Firmware

  Compatible with Elephant V3.1 PyQt5 GUI.

  Data model on Arduino side:
  1) Controller Status (_stat): values stored in Arduino memory after GUI update.
  2) Measured Values (_msr): true hardware values continuously read from ADC/GPIO.

  Protocol V3.1:
  PC -> Arduino: [0xAA][CMD][LEN][DATA...][CRC8]
  Arduino -> PC: [0x55][RSP][LEN][DATA...][CRC8]

  CRC8 is calculated over:
  [CMD/RSP][LEN][DATA...]

  PC commands:
  0x10 CMD_SET_COIL
  0x20 CMD_SET_HC
  0x30 CMD_REQUEST_ALL
  0x31 CMD_REQUEST_MEASUREMENT
  0x40 CMD_TERMINAL_MESSAGE
  0x50 CMD_DISABLE_OUTPUT

  Arduino responses:
  0x90 RSP_ALL_DATA        controller status + measured values
  0x91 RSP_COIL_ACK        latest coil controller status
  0x92 RSP_HC_ACK          latest HC controller status
  0x93 RSP_MEASUREMENT     measured values only
  0x94 RSP_TERMINAL
  0x95 RSP_DISABLE_ACK
  0xE0 RSP_ERROR

  Scaling:
  Current in A: uint16 = A x 100
  AC current in mA: uint16 = mA
  Temperature in C: uint16 = C x 10
  Frequency in Hz: uint16 = Hz
  Waveform: 0=Sine, 1=Triangle, 2=Square
  Flags: bit0=power, bit1=polarity CCW
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// Set to 1 after installing Adafruit_GFX and Adafruit_SSD1306 libraries.
#define USE_OLED 1

#if USE_OLED
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
#endif

// -----------------------------------------------------------------------------
// Protocol constants
// -----------------------------------------------------------------------------
const uint8_t HEADER_PC      = 0xAA;
const uint8_t HEADER_ARDUINO = 0x55;

// PC -> Arduino commands
const uint8_t CMD_SET_COIL          = 0x10;
const uint8_t CMD_SET_HC            = 0x20;
const uint8_t CMD_REQUEST_ALL       = 0x30;
const uint8_t CMD_REQUEST_MEAS      = 0x31;
const uint8_t CMD_TERMINAL_MESSAGE  = 0x40;
const uint8_t CMD_DISABLE_OUTPUT    = 0x50;

// Arduino -> PC responses
const uint8_t RSP_ALL_DATA    = 0x90;
const uint8_t RSP_COIL_ACK    = 0x91;
const uint8_t RSP_HC_ACK      = 0x92;
const uint8_t RSP_MEASUREMENT = 0x93;
const uint8_t RSP_TERMINAL    = 0x94;
const uint8_t RSP_DISABLE_ACK = 0x95;
const uint8_t RSP_ERROR       = 0xE0;

// Error codes
const uint8_t ERR_BAD_CRC     = 0x01;
const uint8_t ERR_UNKNOWN_CMD = 0x02;
const uint8_t ERR_BAD_LENGTH  = 0x03;
const uint8_t ERR_BAD_VALUE   = 0x04;

// Waveform enum, must match GUI
const uint8_t WAVE_SINE     = 0;
const uint8_t WAVE_TRIANGLE = 1;
const uint8_t WAVE_SQUARE   = 2;

const uint8_t MAX_PAYLOAD_LEN = 64;

// -----------------------------------------------------------------------------
// Hardware map
// -----------------------------------------------------------------------------
// INPUTS: measured values
const uint8_t PIN_ADC_COIL_DC_CRT = A0;
const uint8_t PIN_ADC_COIL_AC_CRT = A1;
const uint8_t PIN_ADC_COIL_TEMP   = A2;
const uint8_t PIN_ADC_HC_CRT      = A3;
const uint8_t PIN_ADC_HC_TEMP     = A4;

const uint8_t PIN_IN_COIL_PW   = 30;
const uint8_t PIN_IN_COIL_POL  = 31;
const uint8_t PIN_IN_HC_PW     = 32;
const uint8_t PIN_IN_HC_POL    = 33;
const uint8_t PIN_IN_EMG_STOP  = 34;

// OUTPUTS: controller status applied to hardware
const uint8_t PIN_OUT_COIL_PW   = 40;
const uint8_t PIN_OUT_COIL_POL  = 41;
const uint8_t PIN_OUT_HC_PW     = 42;
const uint8_t PIN_OUT_HC_POL    = 43;
const uint8_t PIN_OUT_EMG_STOP  = 44;

// SPI chip-select pins
const uint8_t PIN_AD9833_CS = 10;
const uint8_t PIN_POT1_CS   = 9;   // coil_dc_crt_stat
const uint8_t PIN_POT2_CS   = 8;   // coil_ac_crt_stat
const uint8_t PIN_POT3_CS   = 7;   // hc_dc_crt_stat

// -----------------------------------------------------------------------------
// Hardware configuration
// -----------------------------------------------------------------------------
// Change if your relay/output/input modules are active LOW.
const bool OUTPUT_ACTIVE_HIGH = true;
const bool INPUT_ACTIVE_HIGH  = true;

const float ADC_REF_VOLTAGE = 5.0;
const float ADC_MAX_COUNT   = 1023.0;

// Calibration placeholders. Adjust to match real analog circuits.
const float COIL_DC_FULL_SCALE_A   = 10.0;
const float COIL_AC_FULL_SCALE_MA  = 100.0;
const float COIL_TEMP_FULL_SCALE_C = 255.0;
const float HC_CRT_FULL_SCALE_A    = 10.0;
const float HC_TEMP_FULL_SCALE_C   = 255.0;

// AD9833 reference clock, commonly 25 MHz modules.
const double AD9833_MCLK_HZ = 25000000.0;

#if USE_OLED
const int OLED_WIDTH = 128;
const int OLED_HEIGHT = 64;
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
#endif

// -----------------------------------------------------------------------------
// Controller Status variables (_stat): stored command/status memory
// -----------------------------------------------------------------------------
bool coil_pw_stat = false;
bool coil_pol_stat = false;       // false=CW, true=CCW
uint16_t coil_dc_crt_stat_x100 = 0;
uint16_t coil_ac_crt_stat_mA = 0;
uint16_t coil_mod_freq_stat_Hz = 0;
uint8_t coil_wf_stat = WAVE_SINE;

bool hc_pw_stat = false;
bool hc_pol_stat = false;         // false=CW, true=CCW
uint16_t hc_dc_crt_stat_x100 = 0;

bool emg_stop_stat = false;

// -----------------------------------------------------------------------------
// Measured Value variables (_msr): updated continuously from ADC/GPIO
// -----------------------------------------------------------------------------
bool coil_pw_msr = false;
bool coil_pol_msr = false;
uint16_t coil_dc_crt_msr_x100 = 0;
uint16_t coil_ac_crt_msr_mA = 0;
uint16_t coil_temp_msr_x10 = 0;

bool hc_pw_msr = false;
bool hc_pol_msr = false;
uint16_t hc_dc_crt_msr_x100 = 0;
uint16_t hc_temp_msr_x10 = 0;

bool emg_stop_msr = false;
uint8_t safety_flags = 0x00;

// Potentiometer raw values, for debug if needed.
uint8_t pot1Value = 0;
uint8_t pot2Value = 0;
uint8_t pot3Value = 0;

unsigned long lastMeasurementUpdateMs = 0;
unsigned long lastOledUpdateMs = 0;

// -----------------------------------------------------------------------------
// CRC8
// -----------------------------------------------------------------------------
uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x80) {
        crc = (uint8_t)((crc << 1) ^ 0x07);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// -----------------------------------------------------------------------------
// Utility helpers
// -----------------------------------------------------------------------------
uint16_t readU16BE(const uint8_t *p) {
  return ((uint16_t)p[0] << 8) | p[1];
}

void writeU16BE(uint8_t *p, uint16_t value) {
  p[0] = (value >> 8) & 0xFF;
  p[1] = value & 0xFF;
}

bool flagPower(uint8_t flags) {
  return flags & 0x01;
}

bool flagPolarityCCW(uint8_t flags) {
  return flags & 0x02;
}

uint8_t makeFlags(bool powerOn, bool polarityCCW) {
  uint8_t flags = 0;
  if (powerOn) flags |= 0x01;
  if (polarityCCW) flags |= 0x02;
  return flags;
}

void writeOutputPin(uint8_t pin, bool on) {
  if (OUTPUT_ACTIVE_HIGH) {
    digitalWrite(pin, on ? HIGH : LOW);
  } else {
    digitalWrite(pin, on ? LOW : HIGH);
  }
}

bool readInputPin(uint8_t pin) {
  bool raw = digitalRead(pin) == HIGH;
  return INPUT_ACTIVE_HIGH ? raw : !raw;
}

uint16_t adcScaled_x100(uint8_t pin, float fullScale) {
  int raw = analogRead(pin);
  float fraction = (float)raw / ADC_MAX_COUNT;
  float value = fraction * fullScale;
  return (uint16_t)round(value * 100.0);
}

uint16_t adcScaled_x10(uint8_t pin, float fullScale) {
  int raw = analogRead(pin);
  float fraction = (float)raw / ADC_MAX_COUNT;
  float value = fraction * fullScale;
  return (uint16_t)round(value * 10.0);
}

uint16_t adcScaled_integer(uint8_t pin, float fullScale) {
  int raw = analogRead(pin);
  float fraction = (float)raw / ADC_MAX_COUNT;
  float value = fraction * fullScale;
  return (uint16_t)round(value);
}

uint8_t mapU16ToByte(uint16_t value, uint16_t inMax) {
  if (value >= inMax) return 255;
  return (uint8_t)((uint32_t)value * 255UL / inMax);
}

// -----------------------------------------------------------------------------
// Packet senders
// -----------------------------------------------------------------------------
void sendPacket(uint8_t rsp, const uint8_t *payload, uint8_t len) {
  uint8_t crcData[MAX_PAYLOAD_LEN + 2];
  crcData[0] = rsp;
  crcData[1] = len;
  for (uint8_t i = 0; i < len; i++) {
    crcData[2 + i] = payload[i];
  }
  uint8_t crc = crc8(crcData, len + 2);

  Serial.write(HEADER_ARDUINO);
  Serial.write(rsp);
  Serial.write(len);
  if (len > 0 && payload != nullptr) {
    Serial.write(payload, len);
  }
  Serial.write(crc);
}

void sendError(uint8_t errCode) {
  uint8_t payload[1] = {errCode};
  sendPacket(RSP_ERROR, payload, 1);
}

void sendTextResponse(const char *msg) {
  uint8_t len = strlen(msg);
  if (len > MAX_PAYLOAD_LEN) len = MAX_PAYLOAD_LEN;
  sendPacket(RSP_TERMINAL, (const uint8_t *)msg, len);
}

// -----------------------------------------------------------------------------
// AD9833 low-level functions
// -----------------------------------------------------------------------------
void ad9833Write16(uint16_t data) {
  digitalWrite(PIN_AD9833_CS, LOW);
  SPI.transfer((data >> 8) & 0xFF);
  SPI.transfer(data & 0xFF);
  digitalWrite(PIN_AD9833_CS, HIGH);
}

void ad9833SetFrequency(uint16_t freqHz) {
  uint32_t freqWord = (uint32_t)((double)freqHz * 268435456.0 / AD9833_MCLK_HZ);
  uint16_t lsb = 0x4000 | (freqWord & 0x3FFF);
  uint16_t msb = 0x4000 | ((freqWord >> 14) & 0x3FFF);

  ad9833Write16(0x2100); // Reset, B28=1
  ad9833Write16(lsb);
  ad9833Write16(msb);
  ad9833Write16(0xC000); // Phase register 0
  ad9833Write16(0x2000); // Exit reset, sine by default
}

void ad9833SetWaveform(uint8_t waveform) {
  switch (waveform) {
    case WAVE_TRIANGLE:
      ad9833Write16(0x2002);
      break;
    case WAVE_SQUARE:
      ad9833Write16(0x2028);
      break;
    case WAVE_SINE:
    default:
      ad9833Write16(0x2000);
      break;
  }
}

void ad9833Apply(uint16_t freqHz, uint8_t waveform) {
  ad9833SetFrequency(freqHz);
  ad9833SetWaveform(waveform);
}

// -----------------------------------------------------------------------------
// MCP41010 digital potentiometer functions
// -----------------------------------------------------------------------------
void mcp41010Write(uint8_t csPin, uint8_t value) {
  digitalWrite(csPin, LOW);
  SPI.transfer(0x11);   // Write data to pot 0
  SPI.transfer(value);
  digitalWrite(csPin, HIGH);
}

void applyDigitalPotsFromStatus() {
  // Demo mapping only. Replace with calibrated mapping later.
  // Pot1 <- coil_dc_crt_stat: 0..10 A => 0..255
  // Pot2 <- coil_ac_crt_stat: 0..100 mA => 0..255
  // Pot3 <- hc_dc_crt_stat:   0..10 A => 0..255
  pot1Value = mapU16ToByte(coil_dc_crt_stat_x100, 1000);
  pot2Value = mapU16ToByte(coil_ac_crt_stat_mA, 100);
  pot3Value = mapU16ToByte(hc_dc_crt_stat_x100, 1000);

  mcp41010Write(PIN_POT1_CS, pot1Value);
  mcp41010Write(PIN_POT2_CS, pot2Value);
  mcp41010Write(PIN_POT3_CS, pot3Value);
}

// -----------------------------------------------------------------------------
// Measurement update and hardware apply
// -----------------------------------------------------------------------------
void updateMeasuredValues() {
  // Continuously read real hardware values.
  coil_dc_crt_msr_x100 = adcScaled_x100(PIN_ADC_COIL_DC_CRT, COIL_DC_FULL_SCALE_A);
  coil_ac_crt_msr_mA   = adcScaled_integer(PIN_ADC_COIL_AC_CRT, COIL_AC_FULL_SCALE_MA);
  coil_temp_msr_x10    = adcScaled_x10(PIN_ADC_COIL_TEMP, COIL_TEMP_FULL_SCALE_C);

  hc_dc_crt_msr_x100   = adcScaled_x100(PIN_ADC_HC_CRT, HC_CRT_FULL_SCALE_A);
  hc_temp_msr_x10      = adcScaled_x10(PIN_ADC_HC_TEMP, HC_TEMP_FULL_SCALE_C);

  coil_pw_msr = readInputPin(PIN_IN_COIL_PW);
  coil_pol_msr = readInputPin(PIN_IN_COIL_POL);
  hc_pw_msr = readInputPin(PIN_IN_HC_PW);
  hc_pol_msr = readInputPin(PIN_IN_HC_POL);
  emg_stop_msr = readInputPin(PIN_IN_EMG_STOP);

  safety_flags = 0x00;
  if (emg_stop_stat) safety_flags |= 0x01;  // requested/emitted by controller
  if (emg_stop_msr)  safety_flags |= 0x02;  // external measured emergency/input
}

void applyControllerStatusToHardware() {
  // Write controller status values to physical outputs.
  writeOutputPin(PIN_OUT_COIL_PW, coil_pw_stat);
  writeOutputPin(PIN_OUT_COIL_POL, coil_pol_stat);
  writeOutputPin(PIN_OUT_HC_PW, hc_pw_stat);
  writeOutputPin(PIN_OUT_HC_POL, hc_pol_stat);
  writeOutputPin(PIN_OUT_EMG_STOP, emg_stop_stat);

  ad9833Apply(coil_mod_freq_stat_Hz, coil_wf_stat);
  applyDigitalPotsFromStatus();
}

// -----------------------------------------------------------------------------
// Payload builders
// -----------------------------------------------------------------------------
uint8_t buildStatusPayload(uint8_t *payload) {
  // Status block length = 11 bytes:
  // [coil_flags_stat]
  // [coil_dc_stat_h][coil_dc_stat_l]
  // [coil_ac_stat_h][coil_ac_stat_l]
  // [coil_freq_stat_h][coil_freq_stat_l]
  // [coil_wf_stat]
  // [hc_flags_stat]
  // [hc_dc_stat_h][hc_dc_stat_l]
  uint8_t i = 0;
  payload[i++] = makeFlags(coil_pw_stat, coil_pol_stat);
  writeU16BE(&payload[i], coil_dc_crt_stat_x100); i += 2;
  writeU16BE(&payload[i], coil_ac_crt_stat_mA); i += 2;
  writeU16BE(&payload[i], coil_mod_freq_stat_Hz); i += 2;
  payload[i++] = coil_wf_stat;
  payload[i++] = makeFlags(hc_pw_stat, hc_pol_stat);
  writeU16BE(&payload[i], hc_dc_crt_stat_x100); i += 2;
  return i;
}

uint8_t buildMeasurementPayload(uint8_t *payload) {
  // Measurement block length = 13 bytes:
  // [coil_flags_msr]
  // [coil_dc_msr_h][coil_dc_msr_l]
  // [coil_ac_msr_h][coil_ac_msr_l]
  // [coil_temp_msr_h][coil_temp_msr_l]
  // [hc_flags_msr]
  // [hc_dc_msr_h][hc_dc_msr_l]
  // [hc_temp_msr_h][hc_temp_msr_l]
  // [safety_flags]
  uint8_t i = 0;
  payload[i++] = makeFlags(coil_pw_msr, coil_pol_msr);
  writeU16BE(&payload[i], coil_dc_crt_msr_x100); i += 2;
  writeU16BE(&payload[i], coil_ac_crt_msr_mA); i += 2;
  writeU16BE(&payload[i], coil_temp_msr_x10); i += 2;
  payload[i++] = makeFlags(hc_pw_msr, hc_pol_msr);
  writeU16BE(&payload[i], hc_dc_crt_msr_x100); i += 2;
  writeU16BE(&payload[i], hc_temp_msr_x10); i += 2;
  payload[i++] = safety_flags;
  return i;
}

void sendControllerStatusOnly(uint8_t responseType) {
  uint8_t payload[11];
  uint8_t len = buildStatusPayload(payload);
  sendPacket(responseType, payload, len);
}

void sendMeasurementOnly() {
  updateMeasuredValues();
  uint8_t payload[13];
  uint8_t len = buildMeasurementPayload(payload);
  sendPacket(RSP_MEASUREMENT, payload, len);
}

void sendAllData() {
  updateMeasuredValues();
  uint8_t payload[24];
  uint8_t i = 0;
  i += buildStatusPayload(&payload[i]);
  i += buildMeasurementPayload(&payload[i]);
  sendPacket(RSP_ALL_DATA, payload, i);
}

// -----------------------------------------------------------------------------
// Command handlers
// -----------------------------------------------------------------------------
void handleSetCoil(const uint8_t *payload, uint8_t len) {
  // Expected payload length = 8:
  // [flags]
  // [dc_h][dc_l]
  // [ac_h][ac_l]
  // [freq_h][freq_l]
  // [waveform]
  if (len != 8) {
    sendError(ERR_BAD_LENGTH);
    return;
  }

  uint8_t flags = payload[0];
  uint16_t dc_x100 = readU16BE(&payload[1]);
  uint16_t ac_mA = readU16BE(&payload[3]);
  uint16_t freq_Hz = readU16BE(&payload[5]);
  uint8_t waveform = payload[7];

  if (dc_x100 > 1000 || ac_mA > 100 || freq_Hz > 10000 || waveform > WAVE_SQUARE) {
    sendError(ERR_BAD_VALUE);
    return;
  }

  // 1) Store received update in Controller Status memory.
  coil_pw_stat = flagPower(flags);
  coil_pol_stat = flagPolarityCCW(flags);
  coil_dc_crt_stat_x100 = dc_x100;
  coil_ac_crt_stat_mA = ac_mA;
  coil_mod_freq_stat_Hz = freq_Hz;
  coil_wf_stat = waveform;

  // 2) Write new status values to related hardware.
  applyControllerStatusToHardware();

  // 3) Send back latest Controller Status to GUI.
  sendControllerStatusOnly(RSP_COIL_ACK);
}

void handleSetHc(const uint8_t *payload, uint8_t len) {
  // Expected payload length = 3:
  // [flags]
  // [current_h][current_l]
  if (len != 3) {
    sendError(ERR_BAD_LENGTH);
    return;
  }

  uint8_t flags = payload[0];
  uint16_t current_x100 = readU16BE(&payload[1]);

  if (current_x100 > 1000) {
    sendError(ERR_BAD_VALUE);
    return;
  }

  // 1) Store received update in Controller Status memory.
  hc_pw_stat = flagPower(flags);
  hc_pol_stat = flagPolarityCCW(flags);
  hc_dc_crt_stat_x100 = current_x100;

  // 2) Write new status values to related hardware.
  applyControllerStatusToHardware();

  // 3) Send back latest Controller Status to GUI.
  sendControllerStatusOnly(RSP_HC_ACK);
}

void handleTerminalMessage(const uint8_t *payload, uint8_t len) {
  char msg[MAX_PAYLOAD_LEN + 1];
  uint8_t n = len;
  if (n > MAX_PAYLOAD_LEN) n = MAX_PAYLOAD_LEN;

  for (uint8_t i = 0; i < n; i++) {
    msg[i] = (char)payload[i];
  }
  msg[n] = '\0';

  if (strcmp(msg, "status") == 0) {
    sendAllData();
  } else if (strcmp(msg, "meas") == 0 || strcmp(msg, "measurement") == 0) {
    sendMeasurementOnly();
  } else if (strcmp(msg, "id") == 0) {
    sendTextResponse("Elephant V3.1 Arduino Mega2560");
  } else if (strcmp(msg, "ping") == 0) {
    sendTextResponse("pong");
  } else {
    sendTextResponse("terminal message received");
  }
}

void handleDisableOutput(const uint8_t *payload, uint8_t len) {
  if (len != 1) {
    sendError(ERR_BAD_LENGTH);
    return;
  }

  emg_stop_stat = payload[0] != 0;
  applyControllerStatusToHardware();

  uint8_t reply[1] = { emg_stop_stat ? 1 : 0 };
  sendPacket(RSP_DISABLE_ACK, reply, 1);
}

void handleCommand(uint8_t cmd, const uint8_t *payload, uint8_t len) {
  switch (cmd) {
    case CMD_SET_COIL:
      handleSetCoil(payload, len);
      break;

    case CMD_SET_HC:
      handleSetHc(payload, len);
      break;

    case CMD_REQUEST_ALL:
      if (len != 0) sendError(ERR_BAD_LENGTH);
      else sendAllData();
      break;

    case CMD_REQUEST_MEAS:
      if (len != 0) sendError(ERR_BAD_LENGTH);
      else sendMeasurementOnly();
      break;

    case CMD_TERMINAL_MESSAGE:
      handleTerminalMessage(payload, len);
      break;

    case CMD_DISABLE_OUTPUT:
      handleDisableOutput(payload, len);
      break;

    default:
      sendError(ERR_UNKNOWN_CMD);
      break;
  }
}

// -----------------------------------------------------------------------------
// Serial parser state machine
// -----------------------------------------------------------------------------
void processSerial() {
  static enum {
    WAIT_HEADER,
    READ_CMD,
    READ_LEN,
    READ_PAYLOAD,
    READ_CRC
  } state = WAIT_HEADER;

  static uint8_t cmd = 0;
  static uint8_t len = 0;
  static uint8_t payload[MAX_PAYLOAD_LEN];
  static uint8_t index = 0;

  while (Serial.available() > 0) {
    uint8_t b = Serial.read();

    switch (state) {
      case WAIT_HEADER:
        if (b == HEADER_PC) state = READ_CMD;
        break;

      case READ_CMD:
        cmd = b;
        state = READ_LEN;
        break;

      case READ_LEN:
        len = b;
        index = 0;
        if (len > MAX_PAYLOAD_LEN) {
          sendError(ERR_BAD_LENGTH);
          state = WAIT_HEADER;
        } else if (len == 0) {
          state = READ_CRC;
        } else {
          state = READ_PAYLOAD;
        }
        break;

      case READ_PAYLOAD:
        payload[index++] = b;
        if (index >= len) state = READ_CRC;
        break;

      case READ_CRC: {
        uint8_t crcData[MAX_PAYLOAD_LEN + 2];
        crcData[0] = cmd;
        crcData[1] = len;
        for (uint8_t i = 0; i < len; i++) {
          crcData[2 + i] = payload[i];
        }

        uint8_t expected = crc8(crcData, len + 2);
        if (b != expected) {
          sendError(ERR_BAD_CRC);
        } else {
          handleCommand(cmd, payload, len);
        }

        state = WAIT_HEADER;
        break;
      }
    }
  }
}

// -----------------------------------------------------------------------------
// OLED debug display
// -----------------------------------------------------------------------------
void updateOled() {
#if USE_OLED
  if (millis() - lastOledUpdateMs < 500) return;
  lastOledUpdateMs = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Elephant V3.1");
  display.print("Cstat: "); display.print(coil_dc_crt_stat_x100 / 100.0, 2); display.println(" A");
  display.print("Cmsr:  "); display.print(coil_dc_crt_msr_x100 / 100.0, 2); display.println(" A");
  display.print("ACmsr: "); display.print(coil_ac_crt_msr_mA); display.println(" mA");
  display.print("Tcoil: "); display.print(coil_temp_msr_x10 / 10.0, 1); display.println(" C");
  display.print("HCmsr: "); display.print(hc_dc_crt_msr_x100 / 100.0, 2); display.println(" A");
  display.display();
#endif
}

// -----------------------------------------------------------------------------
// Setup and loop
// -----------------------------------------------------------------------------
void setup() {
  // Input pins
  pinMode(PIN_IN_COIL_PW, INPUT_PULLUP);
  pinMode(PIN_IN_COIL_POL, INPUT_PULLUP);
  pinMode(PIN_IN_HC_PW, INPUT_PULLUP);
  pinMode(PIN_IN_HC_POL, INPUT_PULLUP);
  pinMode(PIN_IN_EMG_STOP, INPUT_PULLUP);

  // Output pins
  pinMode(PIN_OUT_COIL_PW, OUTPUT);
  pinMode(PIN_OUT_COIL_POL, OUTPUT);
  pinMode(PIN_OUT_HC_PW, OUTPUT);
  pinMode(PIN_OUT_HC_POL, OUTPUT);
  pinMode(PIN_OUT_EMG_STOP, OUTPUT);

  writeOutputPin(PIN_OUT_COIL_PW, false);
  writeOutputPin(PIN_OUT_COIL_POL, false);
  writeOutputPin(PIN_OUT_HC_PW, false);
  writeOutputPin(PIN_OUT_HC_POL, false);
  writeOutputPin(PIN_OUT_EMG_STOP, false);

  pinMode(PIN_AD9833_CS, OUTPUT);
  pinMode(PIN_POT1_CS, OUTPUT);
  pinMode(PIN_POT2_CS, OUTPUT);
  pinMode(PIN_POT3_CS, OUTPUT);

  digitalWrite(PIN_AD9833_CS, HIGH);
  digitalWrite(PIN_POT1_CS, HIGH);
  digitalWrite(PIN_POT2_CS, HIGH);
  digitalWrite(PIN_POT3_CS, HIGH);

  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE2));

  Serial.begin(115200);

#if USE_OLED
  Wire.begin();
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Elephant V3.1");
    display.println("Starting...");
    display.display();
  }
#endif

  // Safe initial controller status and hardware state.
  applyControllerStatusToHardware();
  updateMeasuredValues();

  delay(200);
}

void loop() {
  // Continuously update measured values.
  updateMeasuredValues();

  // Process GUI requests.
  processSerial();

  updateOled();
}
