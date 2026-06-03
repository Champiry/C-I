/*
  Elephont V3.0 - Arduino Mega 2560 Firmware
  
  General code for driving the single channel Quadrupole + Helmholtz coil controller.
  Quadrupole AM modulation--> 20260430_Coil1_HC_V3.0 --> V3.0

Afshin Mahmoudieh Champiry (ChatGPT)
HEPP ISU/ Lab 103
202604030

  Compatible with the Elephant V3.0 PyQt5 GUI.

  Protocol V2.0:
  PC -> Arduino: [0xAA][CMD][LEN][DATA...][CRC8]
  Arduino -> PC: [0x55][RSP][LEN][DATA...][CRC8]

  CRC8 is calculated over:
  [CMD/RSP][LEN][DATA...]

  Main hardware map:
  ADC0 -> dc_current_stat
  ADC1 -> ac_current_stat
  ADC2 -> temp_stat
  ADC3 -> hc_current_stat

  D40 -> coil power relay
  D41 -> coil polarity relay
  D42 -> Helmholtz coil power relay
  D43 -> Helmholtz coil polarity relay
  D44 -> disable/safety output request

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

GPIO --> LED
GPIO --> Buzzer
ADC --> Status

Terminal command format:
p1=120 p2=80 w=sin f=1000
p1=255 p2=10 w=tri f=2500


MEGA2560
|
├── ADC0 ──────────> dc_current_stat
├── ADC1 ──────────> ac_current_stat
├── ADC2 ──────────> temp_stat
├── ADC3 ──────────> hc_current_stat
|
├── Digital (40) ──> coil_pw_stat
├── Digital (41) ──> coil_pol_stat 
├── Digital (42) ──> hc_pw_stat
├── Digital (43) ──> hc_pol_stat
├── Digital (44) ──> safety_disable_out
├── Digital (45) ──> external_interlock_in
|
├── SPI BUS
│   ├── MOSI (51) ─────> AD9833 SDATA
│   │                 ├─> MCP41010 #1 SI
│   │                 └─> MCP41010 #2 SI
│   │
│   ├── SCK (52) ─────> AD9833 SCLK
│   │                 ├─> MCP41010 #1 SCK
│   │                 └─> MCP41010 #2 SCK
│
├── CS Pins
│   ├── D10 ──────────> AD9833 FSYNC
│   ├── D9  ──────────> MCP41010 #1 CS
│   └── D8  ──────────> MCP41010 #2 CS
│
├── I2C OLED
│   ├── SDA (20) ─────> OLED SDA
│   └── SCL (21) ─────> OLED SCL
│
└── Power
    ├── 5V  ──────────> All VCC/VDD
    └── GND ──────────> All Grounds

*/


#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// Uncomment these if you have the Adafruit OLED libraries installed.
// If not installed yet, keep USE_OLED set to 0.
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
const uint8_t CMD_REQUEST_STATUS    = 0x30;
const uint8_t CMD_TERMINAL_MESSAGE  = 0x40;
const uint8_t CMD_DISABLE_OUTPUT    = 0x50;

// Arduino -> PC responses
const uint8_t RSP_FULL_STATUS = 0x90;
const uint8_t RSP_COIL_ACK    = 0x91;
const uint8_t RSP_HC_ACK      = 0x92;
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

// Maximum payload length accepted by Arduino
const uint8_t MAX_PAYLOAD_LEN = 64;

// -----------------------------------------------------------------------------
// Pin map
// -----------------------------------------------------------------------------
const uint8_t PIN_COIL_POWER_RELAY = 40;
const uint8_t PIN_COIL_POL_RELAY   = 41;
const uint8_t PIN_HC_POWER_RELAY   = 42;
const uint8_t PIN_HC_POL_RELAY     = 43;
const uint8_t PIN_DISABLE_OUTPUT   = 44;

const uint8_t PIN_AD9833_CS = 10;
const uint8_t PIN_POT1_CS   = 9;
const uint8_t PIN_POT2_CS   = 8;

const uint8_t PIN_ADC_DC_CURRENT = A0;
const uint8_t PIN_ADC_AC_CURRENT = A1;
const uint8_t PIN_ADC_TEMP       = A2;
const uint8_t PIN_ADC_HC_CURRENT = A3;

// -----------------------------------------------------------------------------
// Hardware configuration
// -----------------------------------------------------------------------------
// Change these if your relay module is active LOW.
const bool RELAY_ACTIVE_HIGH = true;
const bool DISABLE_ACTIVE_HIGH = true;

// ADC calibration placeholders.
// Adjust according to your sensor/output circuits.
// Assumption for demo:
// A0: 0..5 V -> 0..10 A
// A1: 0..5 V -> 0..100 mA
// A2: 0..5 V -> 0..128 deg C
// A3: 0..5 V -> 0..10 A
const float ADC_REF_VOLTAGE = 5.0;
const float ADC_MAX_COUNT   = 1023.0;

const float DC_CURRENT_FULL_SCALE_A = 10.0;
const float AC_CURRENT_FULL_SCALE_MA = 100.0;
const float TEMP_FULL_SCALE_C = 128.0;
const float HC_CURRENT_FULL_SCALE_A = 10.0;

// MCP41010 digital potentiometer defaults.
// Update mapping later based on your real analog amplitude/bias circuits.
uint8_t pot1Value = 0;
uint8_t pot2Value = 0;

// AD9833 reference clock, commonly 25 MHz modules.
const double AD9833_MCLK_HZ = 25000000.0;

// -----------------------------------------------------------------------------
// OLED configuration
// -----------------------------------------------------------------------------
#if USE_OLED
const int OLED_WIDTH = 128;
const int OLED_HEIGHT = 64;
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
#endif

// -----------------------------------------------------------------------------
// Controller variables
// -----------------------------------------------------------------------------
bool coilPowerSet = false;
bool coilPolarityCCWSet = false;
uint16_t dcCurrentSet_x100 = 0;       // A x 100
uint16_t acCurrentSet_mA = 0;         // mA
uint16_t modFreqSet_Hz = 0;           // Hz
uint8_t waveformSet = WAVE_SINE;

bool hcPowerSet = false;
bool hcPolarityCCWSet = false;
uint16_t hcCurrentSet_x100 = 0;       // A x 100

bool disableOutputRequested = false;
uint8_t safetyFlags = 0x00;

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

void digitalWriteRelay(uint8_t pin, bool on) {
  if (RELAY_ACTIVE_HIGH) {
    digitalWrite(pin, on ? HIGH : LOW);
  } else {
    digitalWrite(pin, on ? LOW : HIGH);
  }
}

void digitalWriteDisable(bool on) {
  if (DISABLE_ACTIVE_HIGH) {
    digitalWrite(PIN_DISABLE_OUTPUT, on ? HIGH : LOW);
  } else {
    digitalWrite(PIN_DISABLE_OUTPUT, on ? LOW : HIGH);
  }
}

float adcToVoltage(int adc) {
  return ((float)adc / ADC_MAX_COUNT) * ADC_REF_VOLTAGE;
}

uint16_t readDcCurrent_x100() {
  int raw = analogRead(PIN_ADC_DC_CURRENT);
  float fraction = (float)raw / ADC_MAX_COUNT;
  float currentA = fraction * DC_CURRENT_FULL_SCALE_A;
  return (uint16_t)round(currentA * 100.0);
}

uint16_t readAcCurrent_mA() {
  int raw = analogRead(PIN_ADC_AC_CURRENT);
  float fraction = (float)raw / ADC_MAX_COUNT;
  float currentmA = fraction * AC_CURRENT_FULL_SCALE_MA;
  return (uint16_t)round(currentmA);
}

uint16_t readTemp_x10() {
  int raw = analogRead(PIN_ADC_TEMP);
  float fraction = (float)raw / ADC_MAX_COUNT;
  float tempC = fraction * TEMP_FULL_SCALE_C;
  return (uint16_t)round(tempC * 10.0);
}

uint16_t readHcCurrent_x100() {
  int raw = analogRead(PIN_ADC_HC_CURRENT);
  float fraction = (float)raw / ADC_MAX_COUNT;
  float currentA = fraction * HC_CURRENT_FULL_SCALE_A;
  return (uint16_t)round(currentA * 100.0);
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
  // Frequency register value: freqWord = f_out * 2^28 / MCLK
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
  // Assumes frequency has already been written to FREQ0.
  // Control register variants:
  // sine:     0x2000
  // triangle: 0x2002
  // square:   0x2028  approx: OPBITEN=1, DIV2=1
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

void applyDigitalPotsFromSetpoints() {
  // Demo mapping only. Replace with your real calibration.
  // Pot1 follows AC current command: 0..100 mA -> 0..255.
  // Pot2 follows DC current command: 0..10 A -> 0..255.
  pot1Value = map(acCurrentSet_mA, 0, 100, 0, 255);
  pot2Value = map(dcCurrentSet_x100, 0, 1000, 0, 255);

  mcp41010Write(PIN_POT1_CS, pot1Value);
  mcp41010Write(PIN_POT2_CS, pot2Value);
}

// -----------------------------------------------------------------------------
// Hardware apply functions
// -----------------------------------------------------------------------------
void applyCoilSetpointsToHardware() {
  digitalWriteRelay(PIN_COIL_POWER_RELAY, coilPowerSet);
  digitalWriteRelay(PIN_COIL_POL_RELAY, coilPolarityCCWSet);

  applyDigitalPotsFromSetpoints();
  ad9833Apply(modFreqSet_Hz, waveformSet);
}

void applyHcSetpointsToHardware() {
  digitalWriteRelay(PIN_HC_POWER_RELAY, hcPowerSet);
  digitalWriteRelay(PIN_HC_POL_RELAY, hcPolarityCCWSet);

  // Placeholder:
  // In future, hcCurrentSet_x100 can be converted to a DAC/PWM/digital-pot value.
}

// -----------------------------------------------------------------------------
// Status packet
// -----------------------------------------------------------------------------
void sendFullStatus() {
  // Payload length must match GUI expectation: 14 bytes.
  uint8_t payload[14];
  uint8_t i = 0;

  uint8_t coilFlags = makeFlags(coilPowerSet, coilPolarityCCWSet);
  uint16_t dcCurrent_x100 = readDcCurrent_x100();
  uint16_t acCurrent_mA = readAcCurrent_mA();
  uint16_t freq_Hz = modFreqSet_Hz;
  uint8_t waveform = waveformSet;
  uint16_t temp_x10 = readTemp_x10();
  uint8_t hcFlags = makeFlags(hcPowerSet, hcPolarityCCWSet);
  uint16_t hcCurrent_x100 = readHcCurrent_x100();

  safetyFlags = 0x00;
  if (disableOutputRequested) safetyFlags |= 0x01;

  payload[i++] = coilFlags;
  writeU16BE(&payload[i], dcCurrent_x100); i += 2;
  writeU16BE(&payload[i], acCurrent_mA); i += 2;
  writeU16BE(&payload[i], freq_Hz); i += 2;
  payload[i++] = waveform;
  writeU16BE(&payload[i], temp_x10); i += 2;
  payload[i++] = hcFlags;
  writeU16BE(&payload[i], hcCurrent_x100); i += 2;
  payload[i++] = safetyFlags;

  sendPacket(RSP_FULL_STATUS, payload, 14);
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

  // Range checks, aligned with GUI.
  if (dc_x100 > 1000 || ac_mA > 100 || freq_Hz > 10000 || waveform > WAVE_SQUARE) {
    sendError(ERR_BAD_VALUE);
    return;
  }

  coilPowerSet = flagPower(flags);
  coilPolarityCCWSet = flagPolarityCCW(flags);
  dcCurrentSet_x100 = dc_x100;
  acCurrentSet_mA = ac_mA;
  modFreqSet_Hz = freq_Hz;
  waveformSet = waveform;

  applyCoilSetpointsToHardware();

  // Echo the accepted payload back to GUI.
  sendPacket(RSP_COIL_ACK, payload, len);
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

  hcPowerSet = flagPower(flags);
  hcPolarityCCWSet = flagPolarityCCW(flags);
  hcCurrentSet_x100 = current_x100;

  applyHcSetpointsToHardware();

  // Echo the accepted payload back to GUI.
  sendPacket(RSP_HC_ACK, payload, len);
}

void handleTerminalMessage(const uint8_t *payload, uint8_t len) {
  // Very simple terminal command layer for debug.
  // You can expand this later.
  char msg[MAX_PAYLOAD_LEN + 1];
  uint8_t n = len;
  if (n > MAX_PAYLOAD_LEN) n = MAX_PAYLOAD_LEN;

  for (uint8_t i = 0; i < n; i++) {
    msg[i] = (char)payload[i];
  }
  msg[n] = '\0';

  if (strcmp(msg, "status") == 0) {
    sendFullStatus();
  } else if (strcmp(msg, "id") == 0) {
    sendTextResponse("Elephont V3.0 Arduino Mega2560");
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

  disableOutputRequested = payload[0] != 0;
  digitalWriteDisable(disableOutputRequested);

  uint8_t reply[1] = { disableOutputRequested ? 1 : 0 };
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

    case CMD_REQUEST_STATUS:
      if (len != 0) {
        sendError(ERR_BAD_LENGTH);
      } else {
        sendFullStatus();
      }
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
        if (b == HEADER_PC) {
          state = READ_CMD;
        }
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
        if (index >= len) {
          state = READ_CRC;
        }
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

  uint16_t dc_x100 = readDcCurrent_x100();
  uint16_t ac_mA = readAcCurrent_mA();
  uint16_t temp_x10 = readTemp_x10();
  uint16_t hc_x100 = readHcCurrent_x100();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Elephont V3.0");
  display.print("DC: "); display.print(dc_x100 / 100.0, 2); display.println(" A");
  display.print("AC: "); display.print(ac_mA); display.println(" mA");
  display.print("Tmp:"); display.print(temp_x10 / 10.0, 1); display.println(" C");
  display.print("HC: "); display.print(hc_x100 / 100.0, 2); display.println(" A");
  display.print("F:  "); display.print(modFreqSet_Hz); display.println(" Hz");
  display.display();
#endif
}

// -----------------------------------------------------------------------------
// Setup and loop
// -----------------------------------------------------------------------------
void setup() {
  pinMode(PIN_COIL_POWER_RELAY, OUTPUT);
  pinMode(PIN_COIL_POL_RELAY, OUTPUT);
  pinMode(PIN_HC_POWER_RELAY, OUTPUT);
  pinMode(PIN_HC_POL_RELAY, OUTPUT);
  pinMode(PIN_DISABLE_OUTPUT, OUTPUT);

  digitalWriteRelay(PIN_COIL_POWER_RELAY, false);
  digitalWriteRelay(PIN_COIL_POL_RELAY, false);
  digitalWriteRelay(PIN_HC_POWER_RELAY, false);
  digitalWriteRelay(PIN_HC_POL_RELAY, false);
  digitalWriteDisable(false);

  pinMode(PIN_AD9833_CS, OUTPUT);
  pinMode(PIN_POT1_CS, OUTPUT);
  pinMode(PIN_POT2_CS, OUTPUT);

  digitalWrite(PIN_AD9833_CS, HIGH);
  digitalWrite(PIN_POT1_CS, HIGH);
  digitalWrite(PIN_POT2_CS, HIGH);

  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE2));

  Serial.begin(115200);

#if USE_OLED
  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    // OLED failed; continue without blocking.
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Elephont V3.0");
    display.println("Starting...");
    display.display();
  }
#endif

  // Safe initial hardware state.
  applyDigitalPotsFromSetpoints();
  ad9833Apply(1000, WAVE_SINE);

  delay(200);
}

void loop() {
  processSerial();
  updateOled();

  // Other future control-loop or monitoring tasks can be added here.
}
