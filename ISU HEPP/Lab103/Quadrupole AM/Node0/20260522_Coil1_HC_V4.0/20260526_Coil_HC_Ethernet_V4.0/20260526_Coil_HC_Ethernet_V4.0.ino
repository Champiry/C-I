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

#include <SPI.h>
#include <Ethernet.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================================
// Network Settings
// ============================================================

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x40, 0x00 };
IPAddress ip(192, 168, 1, 50);
IPAddress dnsServer(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

const uint16_t TCP_PORT = 5000;
EthernetServer server(TCP_PORT);
EthernetClient activeClient;

// ============================================================
// OLED Display Settings
// ============================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool display_ok = false;

// ============================================================
// Pins
// ============================================================

const int PIN_SD_CS    = 4;
const int PIN_W5100_CS = 10;
const int PIN_MEGA_SS  = 53;
const int PIN_LED      = 13;

// Digital inputs, measurements
const int PIN_COIL_1_PW_MSR  = 22;
const int PIN_COIL_1_POL_MSR = 23;
const int PIN_HC_PW_MSR      = 30;
const int PIN_HC_POL_MSR     = 31;
const int PIN_EMG_STOP_MSR   = 32;

// Digital outputs, control status
const int PIN_COIL_1_PW_STAT  = 36;
const int PIN_COIL_1_POL_STAT = 37;
const int PIN_HC_PW_STAT      = 44;
const int PIN_HC_POL_STAT     = 45;
const int PIN_EMG_STOP_STAT   = 46;

// Analog inputs
const int ADC_COIL_1_DC_CRT_MSR = A2;
const int ADC_COIL_1_AC_CRT_MSR = A3;
const int ADC_COIL_1_TEMP_MSR   = A4;
const int ADC_HC_CRT_MSR        = A14;
const int ADC_HC_TEMP_MSR       = A15;

// ============================================================
// State Variables
// ============================================================

bool tcp_client_seen = false;
String rxLine = "";
unsigned long lastClientActivityMs = 0;
const unsigned long CLIENT_IDLE_TIMEOUT_MS = 30000UL;

// GUI setpoints / controller status
int coil_pw_set = 0;
int coil_pol_set = 0;
float coil_dc_crt_set = 0.0;
int coil_ac_crt_set = 0;      // mA
int mod_freq_set = 1000;
String wf_set = "Sin";

int hc_pw_set = 0;
int hc_pol_set = 0;
float hc_dc_crt_set = 0.0;

// Measurements
int coil_pw_msr = 0;
int coil_pol_msr = 0;
float coil_dc_crt_msr = 0.0;
int coil_ac_crt_msr = 0;
float coil_temp_msr = 0.0;

int hc_pw_msr = 0;
int hc_pol_msr = 0;
float hc_dc_crt_msr = 0.0;
float hc_temp_msr = 0.0;

int emg_stop_msr = 0;

// ============================================================
// Helper Functions
// ============================================================

void displayBootMessage(const char *msg) {
  if (!display_ok) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Elephant V4.0");
  display.println("Node0 TCP FW");
  display.println();
  display.println(msg);
  display.display();
}

void updateOLED() {
  if (!display_ok) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("Elephant V4.0 Node0");

  display.setCursor(0, 12);
  display.print("IP:");
  display.println(Ethernet.localIP());

  display.setCursor(0, 24);
  display.print("Link:");
  EthernetLinkStatus link = Ethernet.linkStatus();
  if (link == LinkON) display.print("ON");
  else if (link == LinkOFF) display.print("OFF");
  else display.print("UNK");

  display.print(" TCP:");
  display.println(tcp_client_seen ? "OK" : "NO");

  display.setCursor(0, 36);
  display.print("PWR:");
  display.print(coil_pw_set);
  display.print(" POL:");
  display.println(coil_pol_set);

  display.setCursor(0, 48);
  display.print("DC:");
  display.print(coil_dc_crt_set, 2);
  display.print(" AC:");
  display.println(coil_ac_crt_set);

  display.display();
}

void initOLED() {
  Wire.begin();

  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    display_ok = true;
    displayBootMessage("OLED OK");
    Serial.println("OLED detected at 0x3C");
  } else {
    display_ok = false;
    Serial.println("OLED init failed at 0x3C");
  }
}

void printEthernetStatus() {
  Serial.println("---------------- Ethernet Status ----------------");

  EthernetHardwareStatus hw = Ethernet.hardwareStatus();
  if (hw == EthernetNoHardware) {
    Serial.println("Hardware: Ethernet shield NOT found");
  } else if (hw == EthernetW5100) {
    Serial.println("Hardware: W5100 detected");
  } else if (hw == EthernetW5200) {
    Serial.println("Hardware: W5200 detected");
  } else if (hw == EthernetW5500) {
    Serial.println("Hardware: W5500 detected");
  } else {
    Serial.println("Hardware: Unknown Ethernet chip");
  }

  EthernetLinkStatus link = Ethernet.linkStatus();
  if (link == LinkON) {
    Serial.println("Link: ON");
  } else if (link == LinkOFF) {
    Serial.println("Link: OFF");
  } else {
    Serial.println("Link: Unknown");
  }

  Serial.print("Local IP: ");
  Serial.println(Ethernet.localIP());
  Serial.print("TCP server port: ");
  Serial.println(TCP_PORT);
  Serial.println("-------------------------------------------------");
}

void initEthernet() {
  Serial.println("Preparing SPI chip-select pins...");

  pinMode(PIN_MEGA_SS, OUTPUT);
  digitalWrite(PIN_MEGA_SS, HIGH);

  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);      // Disable SD card on shield

  pinMode(PIN_W5100_CS, OUTPUT);
  digitalWrite(PIN_W5100_CS, HIGH);   // Deselect W5100 before Ethernet.begin

  SPI.begin();

  Serial.println("Waiting for W5100 power-up stabilization...");
  delay(2500);

  Serial.println("Starting Ethernet with static IP...");
  Ethernet.init(PIN_W5100_CS);
  Ethernet.begin(mac, ip, dnsServer, gateway, subnet);

  delay(1000);
  server.begin();

  printEthernetStatus();
}

// ============================================================
// Measurement and Hardware Placeholder Layer
// ============================================================

float adcToVoltage(int raw) {
  return raw * (5.0 / 1023.0);
}

void readMeasurements() {
  // Digital measured statuses
  coil_pw_msr  = digitalRead(PIN_COIL_1_PW_MSR);
  coil_pol_msr = digitalRead(PIN_COIL_1_POL_MSR);
  hc_pw_msr    = digitalRead(PIN_HC_PW_MSR);
  hc_pol_msr   = digitalRead(PIN_HC_POL_MSR);
  emg_stop_msr = digitalRead(PIN_EMG_STOP_MSR);

  // Analog values: placeholder conversion for now.
  // Later replace with calibration equations.
  int raw_coil_dc   = analogRead(ADC_COIL_1_DC_CRT_MSR);
  int raw_coil_ac   = analogRead(ADC_COIL_1_AC_CRT_MSR);
  int raw_coil_temp = analogRead(ADC_COIL_1_TEMP_MSR);
  int raw_hc_dc     = analogRead(ADC_HC_CRT_MSR);
  int raw_hc_temp   = analogRead(ADC_HC_TEMP_MSR);

  coil_dc_crt_msr = adcToVoltage(raw_coil_dc);                 // placeholder, V shown as A
  coil_ac_crt_msr = (int)(adcToVoltage(raw_coil_ac) * 200.0);   // placeholder, 0-5V -> 0-1000mA
  coil_temp_msr   = adcToVoltage(raw_coil_temp) * 20.0;         // placeholder, 0-5V -> 0-100C

  hc_dc_crt_msr   = adcToVoltage(raw_hc_dc);                    // placeholder, V shown as A
  hc_temp_msr     = adcToVoltage(raw_hc_temp) * 20.0;           // placeholder
}

void applyDigitalOutputsOnly() {
  // Safe first step: only relay/status outputs.
  // Later: add MCP41010 and AD9833 update here.
  digitalWrite(PIN_COIL_1_PW_STAT,  coil_pw_set  ? HIGH : LOW);
  digitalWrite(PIN_COIL_1_POL_STAT, coil_pol_set ? HIGH : LOW);
  digitalWrite(PIN_HC_PW_STAT,      hc_pw_set    ? HIGH : LOW);
  digitalWrite(PIN_HC_POL_STAT,     hc_pol_set   ? HIGH : LOW);
}

// ============================================================
// Protocol Helpers
// ============================================================

String getValueByKey(const String &line, const String &key, const String &defaultValue) {
  String pattern = key + "=";
  int start = line.indexOf(pattern);
  if (start < 0) return defaultValue;

  start += pattern.length();
  int end = line.indexOf(' ', start);
  if (end < 0) end = line.length();

  String value = line.substring(start, end);
  value.trim();
  return value;
}

void sendStatus(EthernetClient &client) {
  readMeasurements();

  client.print("STATUS ");

  // GUI design names
  client.print("coil_pw_msr=");
  client.print(coil_pw_msr);
  client.print(" coil_pol_msr=");
  client.print(coil_pol_msr);
  client.print(" coil_dc_crt_msr=");
  client.print(coil_dc_crt_msr, 3);
  client.print(" coil_ac_crt_msr=");
  client.print(coil_ac_crt_msr);
  client.print(" coil_temp_msr=");
  client.print(coil_temp_msr, 1);

  client.print(" hc_pw_msr=");
  client.print(hc_pw_msr);
  client.print(" hc_pol_msr=");
  client.print(hc_pol_msr);
  client.print(" hc_dc_crt_msr=");
  client.print(hc_dc_crt_msr, 3);
  client.print(" hc_temp_msr=");
  client.print(hc_temp_msr, 1);

  client.print(" emg_stop_msr=");
  client.println(emg_stop_msr);
}

void handleSetCoil(const String &line, EthernetClient &client) {
  coil_pw_set      = getValueByKey(line, "coil_pw_set", String(coil_pw_set)).toInt();
  coil_pol_set     = getValueByKey(line, "coil_pol_set", String(coil_pol_set)).toInt();
  coil_dc_crt_set  = getValueByKey(line, "coil_dc_crt_set", String(coil_dc_crt_set, 3)).toFloat();
  coil_ac_crt_set  = getValueByKey(line, "coil_ac_crt_set", String(coil_ac_crt_set)).toInt();
  mod_freq_set     = getValueByKey(line, "mod_freq_set", String(mod_freq_set)).toInt();
  wf_set           = getValueByKey(line, "wf_set", wf_set);

  coil_pw_set = constrain(coil_pw_set, 0, 1);
  coil_pol_set = constrain(coil_pol_set, 0, 1);
  coil_dc_crt_set = constrain(coil_dc_crt_set, 0.0, 10.0);
  coil_ac_crt_set = constrain(coil_ac_crt_set, 0, 1000);
  mod_freq_set = constrain(mod_freq_set, 0, 10000);

  applyDigitalOutputsOnly();
  updateOLED();

  client.println("OK SET_COIL");
}

void handleSetHC(const String &line, EthernetClient &client) {
  hc_pw_set      = getValueByKey(line, "hc_pw_set", String(hc_pw_set)).toInt();
  hc_pol_set     = getValueByKey(line, "hc_pol_set", String(hc_pol_set)).toInt();
  hc_dc_crt_set  = getValueByKey(line, "hc_dc_crt_set", String(hc_dc_crt_set, 3)).toFloat();

  hc_pw_set = constrain(hc_pw_set, 0, 1);
  hc_pol_set = constrain(hc_pol_set, 0, 1);
  hc_dc_crt_set = constrain(hc_dc_crt_set, 0.0, 10.0);

  applyDigitalOutputsOnly();
  updateOLED();

  client.println("OK SET_HC");
}

void handleCommand(String line, EthernetClient &client) {
  line.trim();
  if (line.length() == 0) return;

  Serial.print("RX: ");
  Serial.println(line);

  String upper = line;
  upper.toUpperCase();

  if (upper == "PING") {
    client.println("PONG Node0 Elephant V4.0");
  }
  else if (upper == "STATUS" || upper == "GET_STATUS") {
    sendStatus(client);
  }
  else if (upper.startsWith("SET_COIL")) {
    handleSetCoil(line, client);
  }
  else if (upper.startsWith("SET_HC")) {
    handleSetHC(line, client);
  }
  else if (upper == "DISPLAY") {
    updateOLED();
    client.println("OK DISPLAY");
  }
  else {
    client.println("ERR UNKNOWN_COMMAND");
  }
}

// ============================================================
// TCP Server
// ============================================================

void serviceTcpServer() {
  // Accept new client if no active client exists.
  if (!activeClient || !activeClient.connected()) {
    EthernetClient newClient = server.available();
    if (newClient) {
      activeClient = newClient;
      tcp_client_seen = true;
      rxLine = "";
      lastClientActivityMs = millis();
      Serial.println("Client connected.");
      updateOLED();
    }
  }

  if (activeClient && activeClient.connected()) {
    while (activeClient.available()) {
      char c = activeClient.read();
      lastClientActivityMs = millis();

      if (c == '\n' || c == '\r') {
        rxLine.trim();
        if (rxLine.length() > 0) {
          handleCommand(rxLine, activeClient);
          rxLine = "";
        }
      } else {
        // Avoid unlimited buffer growth.
        if (rxLine.length() < 180) {
          rxLine += c;
        } else {
          rxLine = "";
          activeClient.println("ERR LINE_TOO_LONG");
        }
      }
    }

    // Close only after long idle time, not after 3 seconds.
    if (millis() - lastClientActivityMs > CLIENT_IDLE_TIMEOUT_MS) {
      Serial.println("Client idle timeout. Closing TCP client.");
      activeClient.stop();
      rxLine = "";
    }
  }
}

// ============================================================
// Setup / Loop
// ============================================================

void setupPins() {
  pinMode(PIN_LED, OUTPUT);

  pinMode(PIN_COIL_1_PW_MSR, INPUT_PULLUP);
  pinMode(PIN_COIL_1_POL_MSR, INPUT_PULLUP);
  pinMode(PIN_HC_PW_MSR, INPUT_PULLUP);
  pinMode(PIN_HC_POL_MSR, INPUT_PULLUP);
  pinMode(PIN_EMG_STOP_MSR, INPUT_PULLUP);

  pinMode(PIN_COIL_1_PW_STAT, OUTPUT);
  pinMode(PIN_COIL_1_POL_STAT, OUTPUT);
  pinMode(PIN_HC_PW_STAT, OUTPUT);
  pinMode(PIN_HC_POL_STAT, OUTPUT);
  pinMode(PIN_EMG_STOP_STAT, OUTPUT);

  digitalWrite(PIN_COIL_1_PW_STAT, LOW);
  digitalWrite(PIN_COIL_1_POL_STAT, LOW);
  digitalWrite(PIN_HC_PW_STAT, LOW);
  digitalWrite(PIN_HC_POL_STAT, LOW);
  digitalWrite(PIN_EMG_STOP_STAT, LOW);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=================================================");
  Serial.println("Elephant V4.0 - Node0 TCP Firmware Foundation");
  Serial.println("=================================================");

  setupPins();
  initOLED();
  displayBootMessage("Starting Ethernet...");
  initEthernet();
  updateOLED();

  Serial.println("Ready.");
  Serial.println("Ping: 192.168.1.50");
  Serial.println("TCP: 192.168.1.50:5000");
  Serial.println("Commands: PING, STATUS, GET_STATUS, SET_COIL, SET_HC");
}

void loop() {
  static unsigned long lastBlink = 0;
  static unsigned long lastStatus = 0;
  static bool ledState = false;

  unsigned long now = millis();

  if (now - lastBlink >= 500) {
    lastBlink = now;
    ledState = !ledState;
    digitalWrite(PIN_LED, ledState ? HIGH : LOW);
  }

  if (now - lastStatus >= 5000) {
    lastStatus = now;
    printEthernetStatus();
    readMeasurements();
    updateOLED();
  }

  serviceTcpServer();
}
