/*
  Elephant V4.1 Node0 - Low-Noise TCP Firmware
  Main rule:
    MEAS?      -> read ADC/GPIO only, NO SPI writes
    SET_COIL   -> update stored setpoints + relays + AD9833/MCP41010 once
    SET_HC     -> update stored setpoints + relay + MCP41010 once
*/
/*
  Elephant V4.0 - Node0 TCP Firmware Foundation
  Author: Afshin + ChatGPT

  Purpose:
    Stable Ethernet firmware for Arduino Mega 2560 + W5100.
    Compatible with Elephant V4.0 GUI.

  Current implemented features:
    - W5100 Ethernet server on port 5000
    - OLED display at I2C address 0x3C
    - Persistent TCP client handling
    - Commands:
        PING
        STATUS
        GET_STATUS
        SET_COIL coil_pw_set=1 coil_pol_set=0 coil_dc_crt_set=2.500 coil_ac_crt_set=100 mod_freq_set=1000 wf_set=Sin
        SET_HC hc_pw_set=1 hc_pol_set=0 hc_dc_crt_set=1.500
        DISPLAY

  Notes:
    - Hardware driving is still placeholder/safe.
    - Later we will connect variables to relays, AD9833, MCP41010, and ADC calibration.

  Hardware map reference:
    W5100 CS = D10
    SD CS    = D4, disabled HIGH
    Mega SS  = D53, must be OUTPUT
    SPI      = D50/D51/D52
    OLED SDA = D20
    OLED SCL = D21

    Coil #1 input measurements:
      ADC2 -> coil_1_dc_crt_msr
      ADC3 -> coil_1_ac_crt_msr
      ADC4 -> coil_1_temp_msr
      D22  -> coil_1_pw_msr
      D23  -> coil_1_pol_msr

    HC input measurements:
      ADC14 -> hc_crt_msr
      ADC15 -> hc_temp_msr
      D30   -> hc_pw_msr
      D31   -> hc_pol_msr

    Coil #1 control outputs:
      D36 -> coil_1_pw_stat
      D37 -> coil_1_pol_stat

    HC control outputs:
      D44 -> hc_pw_stat
      D45 -> hc_pol_stat
*/

/*
  Elephant V4.1 - Node0 Low-Noise TCP Firmware
  Arduino Mega 2560 + W5100 + OLED + AD9833 + MCP41010

  Main V4.1 rule:
    MEAS?      -> read ADC/GPIO only, NO SPI hardware update
    SET_COIL   -> update relays + MCP41010 + AD9833 once
    SET_HC     -> update HC relay + MCP41010 once

  TCP commands:
    PING
    MEAS?
    SET_COIL coil_pw_set=1 coil_pol_set=0 coil_dc_crt_set=2.500 coil_ac_crt_set=100 mod_freq_set=1000 wf_set=Sin
    SET_HC hc_pw_set=1 hc_pol_set=0 hc_dc_crt_set=1.500
*/

#include <SPI.h>
#include <Ethernet.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================================
// Network
// ============================================================

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x40, 0x01 };

IPAddress ip(192, 168, 1, 50);
IPAddress dnsServer(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

EthernetServer server(5000);
EthernetClient client;

// ============================================================
// OLED
// ============================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool display_ok = false;
bool oled_dirty = true;

// ============================================================
// Pins
// ============================================================

const int PIN_SD_CS    = 4;
const int PIN_W5100_CS = 10;
const int PIN_MEGA_SS  = 53;
const int PIN_LED      = 13;

// SPI chip selects
const int CS_AD9833 = 2;
const int CS_MCP_COIL_DC = 3;
const int CS_MCP_COIL_AC = 5;
const int CS_MCP_HC_DC   = 14;

// Digital outputs
const int COIL_PW_OUT  = 36;
const int COIL_POL_OUT = 37;
const int HC_PW_OUT    = 44;
const int HC_POL_OUT   = 45;

// Digital measured inputs
const int COIL_PW_IN   = 22;
const int COIL_POL_IN  = 23;
const int HC_PW_IN     = 30;
const int HC_POL_IN    = 31;

// ADC inputs
const int ADC_COIL_DC   = A2;
const int ADC_COIL_AC   = A3;
const int ADC_COIL_TEMP = A4;
const int ADC_HC_DC     = A14;
const int ADC_HC_TEMP   = A15;

// ============================================================
// AD9833 settings
// ============================================================

const float AD9833_MCLK = 25000000.0;   // most AD9833 modules use 25 MHz crystal

#define AD9833_B28      0x2000
#define AD9833_RESET    0x0100
#define AD9833_FREQ0    0x4000
#define AD9833_PHASE0   0xC000

#define AD9833_MODE_TRI     0x0002
#define AD9833_OPBITEN      0x0020
#define AD9833_DIV2         0x0008

SPISettings SPI_AD9833_SETTINGS(2000000, MSBFIRST, SPI_MODE2);
SPISettings SPI_MCP_SETTINGS(1000000, MSBFIRST, SPI_MODE0);

// ============================================================
// State
// ============================================================

String rx = "";

int coil_pw_set = 0;
int coil_pol_set = 0;
float coil_dc_crt_set = 0.0;
int coil_ac_crt_set = 0;       // mA
int mod_freq_set = 1000;
String wf_set = "Sin";

int hc_pw_set = 0;
int hc_pol_set = 0;
float hc_dc_crt_set = 0.0;

// ============================================================
// Utility
// ============================================================

String getValue(String line, String key, String fallback) {
  String pattern = key + "=";
  int start = line.indexOf(pattern);
  if (start < 0) return fallback;

  start += pattern.length();
  int end = line.indexOf(' ', start);
  if (end < 0) end = line.length();

  String value = line.substring(start, end);
  value.trim();
  return value;
}

float adcToVolt(int raw) {
  return raw * 5.0 / 1023.0;
}

int currentToPot10A(float current_A) {
  int value = (int)(current_A / 10.0 * 255.0);
  return constrain(value, 0, 255);
}

int acCurrentToPot1000mA(int current_mA) {
  int value = (int)(current_mA / 1000.0 * 255.0);
  return constrain(value, 0, 255);
}

void deselectAllSPI() {
  digitalWrite(PIN_SD_CS, HIGH);
  digitalWrite(PIN_W5100_CS, HIGH);
  digitalWrite(CS_AD9833, HIGH);
  digitalWrite(CS_MCP_COIL_DC, HIGH);
  digitalWrite(CS_MCP_COIL_AC, HIGH);
  digitalWrite(CS_MCP_HC_DC, HIGH);
}

// ============================================================
// MCP41010
// ============================================================

void writeMCP41010(int csPin, int value) {
  value = constrain(value, 0, 255);

  deselectAllSPI();

  SPI.beginTransaction(SPI_MCP_SETTINGS);
  digitalWrite(csPin, LOW);
  SPI.transfer(0x11);      // write data to pot0
  SPI.transfer(value);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
}

// ============================================================
// AD9833
// ============================================================

void ad9833Write16(uint16_t data) {
  SPI.transfer16(data);
}

void writeAD9833(float frequency_hz, String waveform) {
  frequency_hz = constrain(frequency_hz, 0.0, 10000.0);

  uint32_t freqWord = (uint32_t)((frequency_hz * 268435456.0) / AD9833_MCLK);

  uint16_t lsb = AD9833_FREQ0 | (freqWord & 0x3FFF);
  uint16_t msb = AD9833_FREQ0 | ((freqWord >> 14) & 0x3FFF);

  waveform.trim();
  waveform.toUpperCase();

  uint16_t control = AD9833_B28;

  if (waveform == "TRI") {
    control |= AD9833_MODE_TRI;
  } else if (waveform == "SQR") {
    control |= AD9833_OPBITEN | AD9833_DIV2;
  } else {
    // sine: B28 only
  }

  deselectAllSPI();

  SPI.beginTransaction(SPI_AD9833_SETTINGS);
  digitalWrite(CS_AD9833, LOW);

  ad9833Write16(AD9833_B28 | AD9833_RESET);
  ad9833Write16(lsb);
  ad9833Write16(msb);
  ad9833Write16(AD9833_PHASE0);
  ad9833Write16(control);

  digitalWrite(CS_AD9833, HIGH);
  SPI.endTransaction();
}

// ============================================================
// Hardware update functions
// ============================================================

void applyCoilHardwareOnce() {
  digitalWrite(COIL_PW_OUT,  coil_pw_set  ? HIGH : LOW);
  digitalWrite(COIL_POL_OUT, coil_pol_set ? HIGH : LOW);

  int dcPot = currentToPot10A(coil_dc_crt_set);
  int acPot = acCurrentToPot1000mA(coil_ac_crt_set);

  writeMCP41010(CS_MCP_COIL_DC, dcPot);
  writeMCP41010(CS_MCP_COIL_AC, acPot);
  writeAD9833((float)mod_freq_set, wf_set);

  oled_dirty = true;
}

void applyHCHardwareOnce() {
  digitalWrite(HC_PW_OUT,  hc_pw_set  ? HIGH : LOW);
  digitalWrite(HC_POL_OUT, hc_pol_set ? HIGH : LOW);

  int hcPot = currentToPot10A(hc_dc_crt_set);
  writeMCP41010(CS_MCP_HC_DC, hcPot);

  oled_dirty = true;
}

// ============================================================
// Measurements
// ============================================================

void sendMeasurements(EthernetClient &c) {
  int coil_pw_msr  = digitalRead(COIL_PW_IN);
  int coil_pol_msr = digitalRead(COIL_POL_IN);
  int hc_pw_msr    = digitalRead(HC_PW_IN);
  int hc_pol_msr   = digitalRead(HC_POL_IN);

  float coil_dc = adcToVolt(analogRead(ADC_COIL_DC));              // placeholder calibration
  int coil_ac   = (int)(adcToVolt(analogRead(ADC_COIL_AC)) * 200); // 0-5V -> 0-1000 mA placeholder
  float coil_t  = adcToVolt(analogRead(ADC_COIL_TEMP)) * 20.0;     // 0-5V -> 0-100 C placeholder

  float hc_dc   = adcToVolt(analogRead(ADC_HC_DC));                // placeholder calibration
  float hc_t    = adcToVolt(analogRead(ADC_HC_TEMP)) * 20.0;

  c.print("MEAS ");
  c.print("coil_pw_msr="); c.print(coil_pw_msr);
  c.print(" coil_pol_msr="); c.print(coil_pol_msr);
  c.print(" coil_dc_crt_msr="); c.print(coil_dc, 3);
  c.print(" coil_ac_crt_msr="); c.print(coil_ac);
  c.print(" coil_temp_msr="); c.print(coil_t, 1);

  c.print(" hc_pw_msr="); c.print(hc_pw_msr);
  c.print(" hc_pol_msr="); c.print(hc_pol_msr);
  c.print(" hc_dc_crt_msr="); c.print(hc_dc, 3);
  c.print(" hc_temp_msr="); c.println(hc_t, 1);
}

// ============================================================
// Command handling
// ============================================================

void handleCommand(String line, EthernetClient &c) {
  line.trim();
  if (line.length() == 0) return;

  Serial.print("RX: ");
  Serial.println(line);

  String upper = line;
  upper.toUpperCase();

  if (upper == "PING") {
    c.println("PONG Node0 Elephant V4.1");
  }

  else if (upper == "MEAS?" || upper == "STATUS" || upper == "GET_STATUS") {
    // Very important: no SPI writes here.
    sendMeasurements(c);
  }

  else if (upper.startsWith("SET_COIL")) {
    coil_pw_set     = getValue(line, "coil_pw_set", String(coil_pw_set)).toInt();
    coil_pol_set    = getValue(line, "coil_pol_set", String(coil_pol_set)).toInt();
    coil_dc_crt_set = getValue(line, "coil_dc_crt_set", String(coil_dc_crt_set, 3)).toFloat();
    coil_ac_crt_set = getValue(line, "coil_ac_crt_set", String(coil_ac_crt_set)).toInt();
    mod_freq_set    = getValue(line, "mod_freq_set", String(mod_freq_set)).toInt();
    wf_set          = getValue(line, "wf_set", wf_set);

    coil_pw_set = constrain(coil_pw_set, 0, 1);
    coil_pol_set = constrain(coil_pol_set, 0, 1);
    coil_dc_crt_set = constrain(coil_dc_crt_set, 0.0, 10.0);
    coil_ac_crt_set = constrain(coil_ac_crt_set, 0, 1000);
    mod_freq_set = constrain(mod_freq_set, 0, 10000);

    applyCoilHardwareOnce();

    c.println("OK SET_COIL");
    Serial.println("OK SET_COIL");
  }

  else if (upper.startsWith("SET_HC")) {
    hc_pw_set     = getValue(line, "hc_pw_set", String(hc_pw_set)).toInt();
    hc_pol_set    = getValue(line, "hc_pol_set", String(hc_pol_set)).toInt();
    hc_dc_crt_set = getValue(line, "hc_dc_crt_set", String(hc_dc_crt_set, 3)).toFloat();

    hc_pw_set = constrain(hc_pw_set, 0, 1);
    hc_pol_set = constrain(hc_pol_set, 0, 1);
    hc_dc_crt_set = constrain(hc_dc_crt_set, 0.0, 10.0);

    applyHCHardwareOnce();

    c.println("OK SET_HC");
    Serial.println("OK SET_HC");
  }

  else {
    c.println("ERR UNKNOWN_COMMAND");
    Serial.println("ERR UNKNOWN_COMMAND");
  }
}

// ============================================================
// OLED
// ============================================================

void updateOLED() {
  if (!display_ok) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("Elephant V4.1 Node0");

  display.setCursor(0, 12);
  display.print("IP:");
  display.println(Ethernet.localIP());

  display.setCursor(0, 24);
  display.print("PWR:");
  display.print(coil_pw_set);
  display.print(" POL:");
  display.println(coil_pol_set);

  display.setCursor(0, 36);
  display.print("DC:");
  display.print(coil_dc_crt_set, 2);
  display.print(" AC:");
  display.println(coil_ac_crt_set);

  display.setCursor(0, 48);
  display.print("F:");
  display.print(mod_freq_set);
  display.print(" ");
  display.println(wf_set);

  display.display();
}

// ============================================================
// Setup
// ============================================================

void setupPins() {
  pinMode(PIN_LED, OUTPUT);

  pinMode(PIN_MEGA_SS, OUTPUT);
  digitalWrite(PIN_MEGA_SS, HIGH);

  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);

  pinMode(PIN_W5100_CS, OUTPUT);
  digitalWrite(PIN_W5100_CS, HIGH);

  pinMode(CS_AD9833, OUTPUT);
  pinMode(CS_MCP_COIL_DC, OUTPUT);
  pinMode(CS_MCP_COIL_AC, OUTPUT);
  pinMode(CS_MCP_HC_DC, OUTPUT);

  digitalWrite(CS_AD9833, HIGH);
  digitalWrite(CS_MCP_COIL_DC, HIGH);
  digitalWrite(CS_MCP_COIL_AC, HIGH);
  digitalWrite(CS_MCP_HC_DC, HIGH);

  pinMode(COIL_PW_OUT, OUTPUT);
  pinMode(COIL_POL_OUT, OUTPUT);
  pinMode(HC_PW_OUT, OUTPUT);
  pinMode(HC_POL_OUT, OUTPUT);

  digitalWrite(COIL_PW_OUT, LOW);
  digitalWrite(COIL_POL_OUT, LOW);
  digitalWrite(HC_PW_OUT, LOW);
  digitalWrite(HC_POL_OUT, LOW);

  pinMode(COIL_PW_IN, INPUT_PULLUP);
  pinMode(COIL_POL_IN, INPUT_PULLUP);
  pinMode(HC_PW_IN, INPUT_PULLUP);
  pinMode(HC_POL_IN, INPUT_PULLUP);
}

void setupOLED() {
  Wire.begin();
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    display_ok = true;
    updateOLED();
    Serial.println("OLED OK");
  } else {
    display_ok = false;
    Serial.println("OLED failed");
  }
}

void setupEthernet() {
  SPI.begin();

  delay(2500);

  Ethernet.init(PIN_W5100_CS);
  Ethernet.begin(mac, ip, dnsServer, gateway, subnet);
  delay(1000);

  server.begin();

  Serial.print("Node0 IP: ");
  Serial.println(Ethernet.localIP());
  Serial.println("TCP port: 5000");
}

// ============================================================
// Main
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=======================================");
  Serial.println("Elephant V4.1 Node0 Low-Noise Firmware");
  Serial.println("=======================================");

  setupPins();
  setupOLED();
  setupEthernet();

  Serial.println("Ready: PING, MEAS?, SET_COIL, SET_HC");
}

void loop() {
  static unsigned long lastBlink = 0;
  static unsigned long lastOLED = 0;
  static bool led = false;

  unsigned long now = millis();

  if (now - lastBlink >= 500) {
    lastBlink = now;
    led = !led;
    digitalWrite(PIN_LED, led ? HIGH : LOW);
  }

  if (!client || !client.connected()) {
    client = server.available();
    rx = "";
  }

  if (client && client.connected()) {
    while (client.available()) {
      char ch = client.read();

      if (ch == '\n' || ch == '\r') {
        rx.trim();
        if (rx.length() > 0) {
          handleCommand(rx, client);
          rx = "";
        }
      } else {
        if (rx.length() < 180) {
          rx += ch;
        } else {
          rx = "";
          client.println("ERR LINE_TOO_LONG");
        }
      }
    }
  }

  // OLED updates only after SET_COIL / SET_HC and not faster than 1 Hz.
  if (oled_dirty && now - lastOLED >= 1000) {
    lastOLED = now;
    oled_dirty = false;
    updateOLED();
  }
}