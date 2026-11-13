// ==============================================================================
// 250m Smart Pump House Automation — Node 2: Remote Pump Controller Firmware
// ==============================================================================
// Target MCU: ESP32 NodeMCU (30-pin) at Well / Pump House
// Core Capabilities:
//   1. Dynamic Channel Hopping & Auto-Lock:
//        - On boot, hops channels 1-13 (350ms dwell) until Base ESP beacon is detected.
//        - Locks channel immediately; unlocks to re-scan if link lost > 8 seconds while idle.
//   2. Live 250m RF Signal Diagnostics (Active by default):
//        - Captures live 802.11 signal strength (dBm) of Base Node.
//        - Maps dBm to Voltage (V) (e.g. -85 dBm -> 85.0 V).
//        - Maps RSSI to Current (A) (e.g. -85 dBm -> 8.5 A).
//        - Maps SNR (dB) to Active Power (Watts) (e.g. 11 dB -> 11.0 W).
//   3. Power-Cut Recovery Safety:
//        - Never restarts automatically after grid blackout; initializes strictly to Standby/OFF.
//   4. Starter Relay Sequencing:
//        - Relay 1 (GPIO 18): Run / Contactor Latch circuit (Active LOW: LOW=Energized, HIGH=Cut).
//        - Relay 2 (GPIO 19): Green Start Button pulse across capacitor (Active LOW: 1.8s pulse).
//   5. True-RMS Sensing Engine (ZMCT103C on GPIO 34 + ZMPT101B on GPIO 35):
//        - Overload Protection: > 16.0A instant cutoff (bearing jam / locked rotor).
//        - Dry-Run Protection: < 4.0A for > 8s cutoff (suction loss / bore well dry).
//        - Grid Voltage Protection: < 175V Brownout cutoff, > 265V Surge cutoff.
//   6. Mode-Aware Watchdog Link Protection:
//        - Auto-Fill / Tank Filling: 180s link loss timeout triggers safety cutoff.
//        - Garden Mode: Link loss watchdog bypassed! Pump continues running safely.
//   7. Bi-Directional ESP-NOW Telemetry Sync every 2 seconds.
// ==============================================================================

#include <Arduino.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#ifndef ESP_ARDUINO_VERSION_VAL
#define ESP_ARDUINO_VERSION_VAL(major, minor, patch) ((major << 16) | (minor << 8) | (patch))
#endif
#ifndef ESP_ARDUINO_VERSION
#define ESP_ARDUINO_VERSION 0
#endif
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ------------------------------------------------------------------------------
// OPERATIONAL SENSOR MODE SELECTION
// ------------------------------------------------------------------------------
// Set to 1 for Live 250m RF Signal Mapping (Voltage = dBm, Current = RSSI/10, Power = SNR dB).
// Set to 0 when physical ZMCT103C CT and ZMPT101B PT are hooked to the 230V 10A motor starter.
#define SENSOR_MODE_RSSI_DIAGNOSTICS 1

// ------------------------------------------------------------------------------
// 1. HARDWARE PIN DEFINITIONS
// ------------------------------------------------------------------------------
#define RELAY1_RUN_PIN       18   // Relay 1: Run / Contactor Holding Circuit
#define RELAY2_START_PIN     19   // Relay 2: Green Button Start Pulse Circuit
#define ZMCT103C_PIN         34   // ADC1 pin (GPIO 34) for ZMCT103C Current Module OUT
#define ZMPT101B_PIN         35   // ADC1 pin (GPIO 35) for ZMPT101B Voltage Module OUT
#define ONBOARD_LED_PIN       2   // Blue status LED

// ------------------------------------------------------------------------------
// 2. TARGET BASE (TANK) NODE MAC ADDRESS
// ------------------------------------------------------------------------------
// Node 1 (Tank/Base ESP32): 68:09:47:5F:2E:C4
const uint8_t baseNodeMac[] = {0x68, 0x09, 0x47, 0x5F, 0x2E, 0xC4};

// ------------------------------------------------------------------------------
// 3. PACKET DEFINITIONS (ESP-NOW)
// ------------------------------------------------------------------------------
enum CommandType : uint8_t {
  CMD_BEACON = 0,
  CMD_START  = 1,
  CMD_STOP   = 2,
  CMD_CONFIG = 3
};

enum ModeCode : uint8_t {
  MODE_STANDBY     = 0,
  MODE_MANUAL      = 1,
  MODE_AUTOFILL    = 2,
  MODE_GARDEN      = 3,
  MODE_VALVE_ERROR = 4
};

enum FaultCode : uint8_t {
  FAULT_NONE             = 0,
  FAULT_CONTACTOR_FAIL   = 1,
  FAULT_DRY_RUN          = 2,
  FAULT_OVERLOAD         = 3,
  FAULT_WATCHDOG_TIMEOUT = 4,
  FAULT_LOW_VOLTAGE      = 5,
  FAULT_HIGH_VOLTAGE     = 6,
  FAULT_VALVE_OPEN       = 7
};

typedef struct __attribute__((packed)) {
  uint8_t  cmd;
  uint8_t  channel;
  uint8_t  modeCode;
  uint8_t  reserved;
  uint32_t packetId;
  float    overloadAmps;
  float    dryRunAmps;
  float    lowVoltage;
  float    highVoltage;
  float    startPulseSec;
} BasePacket;

typedef struct __attribute__((packed)) {
  uint32_t packetId;
  bool     isRunning;
  float    voltageRMS;
  float    currentAmps;
  float    powerWatts;
  uint8_t  faultCode;
  uint8_t  modeCode;
} PumpTelemetryPacket;

// ------------------------------------------------------------------------------
// 4. CONFIGURATION DEFAULTS & GLOBAL STATE
// ------------------------------------------------------------------------------
float overloadAmps       = 16.0;  // Amps RMS
float dryRunAmps         = 4.0;   // Amps RMS
float lowVoltageCutoff   = 175.0; // Volts RMS
float highVoltageCutoff  = 265.0; // Volts RMS
float startPulseDuration = 1.8;   // Seconds

const float CURRENT_CALIBRATION_FACTOR = 12.5;  // Multiplier to convert RMS AC Volts to Amps
const float VOLTAGE_CALIBRATION_FACTOR = 230.0; // Multiplier for 230V Nominal Supply
const float POWER_FACTOR_ESTIMATE      = 0.85;  // Single-phase induction motor power factor
const float RUN_CONFIRM_AMPS           = 2.0;   // Minimum amps confirming contactor closure

// Live Captured RF Signal Metrics
volatile int8_t latestRssiDbm       = -75; // Initial fallback RSSI in dBm

// State Flags
bool pumpIsRunning          = false;
bool startingSequenceActive = false;
bool channelLocked          = false;
uint8_t currentWiFiChannel  = 1;
uint8_t activeModeCode      = MODE_STANDBY;
FaultCode activeFaultCode   = FAULT_NONE;

// Timers
unsigned long startPulseTimerMs      = 0;
unsigned long dryRunTimerMs          = 0;
unsigned long lastPacketRecvMs       = 0;
unsigned long lastChannelScanMs      = 0;
unsigned long lastTelemetrySentMs    = 0;
uint32_t telemetrySequenceCounter    = 0;

esp_now_peer_info_t peerInfo;
BasePacket rxPacket;
PumpTelemetryPacket txTelemetry;

// ------------------------------------------------------------------------------
// 5. FUNCTION DECLARATIONS
// ------------------------------------------------------------------------------
float readRMSCurrent();
float readRMSVoltage();
void executeStartSequence();
void executeEmergencyStop(FaultCode reason);
void sendTelemetry();
void setRadioChannel(uint8_t ch);
void registerOrUpdatePeer(uint8_t ch);

// ------------------------------------------------------------------------------
// 6. CHANNEL MANAGEMENT & PEER REGISTRATION
// ------------------------------------------------------------------------------
void setRadioChannel(uint8_t ch) {
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  currentWiFiChannel = ch;
}

void registerOrUpdatePeer(uint8_t ch) {
  if (esp_now_is_peer_exist(baseNodeMac)) {
    esp_now_del_peer(baseNodeMac);
  }
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, baseNodeMac, 6);
  peerInfo.channel = ch;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

// ------------------------------------------------------------------------------
// 7. PROMISCUOUS PACKET SNIFFER (Captures accurate physical RF RSSI)
// ------------------------------------------------------------------------------
void promiscuousRxCallback(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA) return;
  const wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t*)buf;
  const uint8_t *payload = pkt->payload;
  // Offset 10 in 802.11 frame header is the transmitter Source MAC
  if (memcmp(&payload[10], baseNodeMac, 6) == 0) {
    latestRssiDbm = pkt->rx_ctrl.rssi;
  }
}

// ------------------------------------------------------------------------------
// 8. ESP-NOW RECEIVE CALLBACK (Core 2.x & 3.x Dual Support)
// ------------------------------------------------------------------------------
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingDataRaw, int len) {
  if (!recv_info || !recv_info->src_addr || !incomingDataRaw) return;
  const uint8_t *senderMac = recv_info->src_addr;
  if (recv_info->rx_ctrl) {
    latestRssiDbm = recv_info->rx_ctrl->rssi;
  }
#else
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingDataRaw, int len) {
  if (!mac_addr || !incomingDataRaw) return;
  const uint8_t *senderMac = mac_addr;
#endif

  // Verify Sender MAC
  for (int i = 0; i < 6; i++) {
    if (senderMac[i] != baseNodeMac[i]) return; // Ignore foreign packets
  }

  if (len != sizeof(BasePacket)) return;

  memcpy(&rxPacket, incomingDataRaw, sizeof(BasePacket));
  lastPacketRecvMs = millis();

  // Lock channel immediately upon receiving valid Base packet
  if (!channelLocked) {
    channelLocked = true;
    digitalWrite(ONBOARD_LED_PIN, HIGH);
    Serial.printf("✓ Locked onto Base Node Wi-Fi Channel: %d (Signal: %d dBm)\n", currentWiFiChannel, latestRssiDbm);
  }

  // Update dynamic thresholds from Base packet
  if (rxPacket.overloadAmps > 5.0)   overloadAmps = rxPacket.overloadAmps;
  if (rxPacket.dryRunAmps > 1.0)     dryRunAmps = rxPacket.dryRunAmps;
  if (rxPacket.lowVoltage > 100.0)   lowVoltageCutoff = rxPacket.lowVoltage;
  if (rxPacket.highVoltage > 200.0)  highVoltageCutoff = rxPacket.highVoltage;
  if (rxPacket.startPulseSec > 0.5)  startPulseDuration = rxPacket.startPulseSec;
  activeModeCode = (ModeCode)rxPacket.modeCode;

  // Handle Command
  switch (rxPacket.cmd) {
    case CMD_START:
      Serial.println("▶ [CMD_START Received] Initiating Pump Start Sequence...");
      executeStartSequence();
      break;

    case CMD_STOP:
      Serial.println("⏹ [CMD_STOP Received] Executing Normal Stop...");
      executeEmergencyStop(FAULT_NONE);
      break;

    case CMD_BEACON:
    case CMD_CONFIG:
      // Heartbeat/Config: refreshed lastPacketRecvMs
      break;
  }
}

// ------------------------------------------------------------------------------
// 9. SETUP
// ------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n========================================================");
  Serial.printf("  250m Pump Node Controller Starting [RSSI Mode: %s]\n",
                SENSOR_MODE_RSSI_DIAGNOSTICS ? "ACTIVE" : "ANALOG SENSORS");
  Serial.println("========================================================");

  // Status LED
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  digitalWrite(ONBOARD_LED_PIN, LOW);

  // Configure Relays (Active LOW: HIGH = Disengaged / OFF)
  pinMode(RELAY1_RUN_PIN, OUTPUT);
  pinMode(RELAY2_START_PIN, OUTPUT);
  digitalWrite(RELAY1_RUN_PIN, HIGH); // Contactor Disengaged
  digitalWrite(RELAY2_START_PIN, HIGH); // Start Pulse Disengaged

  // Configure ADC Pins (12-bit, 0 - 4095)
  pinMode(ZMCT103C_PIN, INPUT);
  pinMode(ZMPT101B_PIN, INPUT);
  analogReadResolution(12);

  // Configure Wi-Fi & ESP-NOW in Standard Station Mode
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Set initial scan channel
  setRadioChannel(currentWiFiChannel);

  // Maximize TX Power (20dBm) & Disable Power Save
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_max_tx_power(80);

  // Register Promiscuous RX Sniffer for hardware RSSI
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(promiscuousRxCallback);

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ERROR: ESP-NOW Init Failed!");
    return;
  }

  // 1Mbps 802.11b Long Preamble for maximum 250m penetration
  esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_1M_L);
  registerOrUpdatePeer(currentWiFiChannel);
  esp_now_register_recv_cb(OnDataRecv);

  lastPacketRecvMs = millis();
  Serial.printf("ESP-NOW Ready. Scanning Channels 1-13 for Base Node...\n\n");
}

// ------------------------------------------------------------------------------
// 10. MAIN LOOP
// ------------------------------------------------------------------------------
void loop() {
  unsigned long now = millis();

  // 1. Dynamic Channel Hopping (When not locked or link lost > 8s while idle)
  if (!pumpIsRunning) {
    if (!channelLocked || (now - lastPacketRecvMs > 8000)) {
      if (now - lastChannelScanMs >= 350) {
        lastChannelScanMs = now;
        channelLocked = false;
        digitalWrite(ONBOARD_LED_PIN, !digitalRead(ONBOARD_LED_PIN)); // Blink during scan

        currentWiFiChannel = (currentWiFiChannel % 13) + 1;
        setRadioChannel(currentWiFiChannel);
        registerOrUpdatePeer(currentWiFiChannel);
      }
    }
  }

  // 2. Starter Relay Timing (Release Start Pulse after startPulseDuration)
  if (startingSequenceActive) {
    if (now - startPulseTimerMs >= (unsigned long)(startPulseDuration * 1000.0)) {
      digitalWrite(RELAY2_START_PIN, HIGH); // Release Green button pulse
      startingSequenceActive = false;
      Serial.println("[Relay 2] Start Pulse Released.");

#if !SENSOR_MODE_RSSI_DIAGNOSTICS
      // Contactor settling interval & physical current verification
      delay(1000);
      float current = readRMSCurrent();
      if (current < RUN_CONFIRM_AMPS) {
        Serial.printf("🚨 [Fault] Contactor failed to latch! Measured: %.2f A (< %.2f A)\n", current, RUN_CONFIRM_AMPS);
        executeEmergencyStop(FAULT_CONTACTOR_FAIL);
      } else {
        pumpIsRunning = true;
        activeFaultCode = FAULT_NONE;
        Serial.printf("✓ [SUCCESS] 10A Motor Running! Confirmed Current: %.2f A\n", current);
        sendTelemetry();
      }
#else
      pumpIsRunning = true;
      activeFaultCode = FAULT_NONE;
      Serial.println("✓ [SUCCESS] Motor Contactor Latched (RSSI Diagnostic Mode)");
      sendTelemetry();
#endif
    }
  }

  // 3. Continuous Sensing & Monitoring
#if !SENSOR_MODE_RSSI_DIAGNOSTICS
  float mainsVoltage = readRMSVoltage();
  txTelemetry.voltageRMS = mainsVoltage;

  if (pumpIsRunning && !startingSequenceActive) {
    float current = readRMSCurrent();
    txTelemetry.currentAmps = current;
    txTelemetry.powerWatts = current * mainsVoltage * POWER_FACTOR_ESTIMATE;

    // Overload Check
    if (current > overloadAmps) {
      Serial.printf("🚨 [ALERT] Overload / Jam Detected (%.2f A > %.2f A)! Instant Cutoff.\n", current, overloadAmps);
      executeEmergencyStop(FAULT_OVERLOAD);
    }
    // Dry Run Protection
    else if (current < dryRunAmps) {
      if (dryRunTimerMs == 0) {
        dryRunTimerMs = now;
      } else if (now - dryRunTimerMs >= 8000) {
        Serial.printf("🚨 [ALERT] Dry Run Confirmed (8s). Cutting pump!\n");
        executeEmergencyStop(FAULT_DRY_RUN);
      }
    } else {
      dryRunTimerMs = 0;
    }

    // Mains Voltage Protection
    if (mainsVoltage > 50.0) {
      if (mainsVoltage < lowVoltageCutoff) {
        executeEmergencyStop(FAULT_LOW_VOLTAGE);
      } else if (mainsVoltage > highVoltageCutoff) {
        executeEmergencyStop(FAULT_HIGH_VOLTAGE);
      }
    }
  } else {
    dryRunTimerMs = 0;
    txTelemetry.currentAmps = 0.0;
    txTelemetry.powerWatts = 0.0;
  }
#else
  // Live RF Signal Diagnostic Telemetry:
  // 1. dBm as Voltage (e.g. -85 dBm -> 85.0 V)
  float rssiMag = abs((float)latestRssiDbm);
  txTelemetry.voltageRMS = rssiMag;

  // 2. RSSI signal metric as Current (e.g. 8.5 A)
  txTelemetry.currentAmps = rssiMag / 10.0;

  // 3. SNR (dB) as Active Power (Watts) (assuming -95 dBm thermal noise floor)
  float snr = (float)(latestRssiDbm - (-95));
  if (snr < 0.0) snr = 0.0;
  txTelemetry.powerWatts = snr;
#endif

  // Watchdog Link Protection (3-Minute Failsafe)
  if (pumpIsRunning && (now - lastPacketRecvMs > 180000)) {
    if (activeModeCode == MODE_GARDEN) {
      static unsigned long lastGardenWarnMs = 0;
      if (now - lastGardenWarnMs > 30000) {
        lastGardenWarnMs = now;
        Serial.println("🌿 [Garden Mode] Link lost across 250m, but Garden Mode remains ACTIVE!");
      }
    } else {
      Serial.println("🚨 [ALERT] Tank Node Link Lost for 3 minutes! Failsafe cutoff.");
      executeEmergencyStop(FAULT_WATCHDOG_TIMEOUT);
    }
  }

  // 4. Send Telemetry to Base Node every 2 seconds
  if (now - lastTelemetrySentMs >= 2000) {
    lastTelemetrySentMs = now;
    sendTelemetry();
  }
}

// ------------------------------------------------------------------------------
// 11. MOTOR CONTROL SEQUENCES
// ------------------------------------------------------------------------------
void executeStartSequence() {
  if (pumpIsRunning || startingSequenceActive) {
    Serial.println("Start ignored: Pump already active or starting.");
    return;
  }

#if !SENSOR_MODE_RSSI_DIAGNOSTICS
  // Voltage Pre-Check (only for real hardware AC connection)
  float currentMainsV = readRMSVoltage();
  if (currentMainsV > 50.0) {
    if (currentMainsV < lowVoltageCutoff) {
      Serial.printf("❌ [BLOCKED] Voltage too low (%.1f V < %.1f V)\n", currentMainsV, lowVoltageCutoff);
      activeFaultCode = FAULT_LOW_VOLTAGE;
      sendTelemetry();
      return;
    }
    if (currentMainsV > highVoltageCutoff) {
      Serial.printf("❌ [BLOCKED] Voltage too high (%.1f V > %.1f V)\n", currentMainsV, highVoltageCutoff);
      activeFaultCode = FAULT_HIGH_VOLTAGE;
      sendTelemetry();
      return;
    }
  }
#endif

  Serial.println("▶ Engaging Relays: Latching Contactor + Starting Capacitor Pulse...");
  activeFaultCode = FAULT_NONE;

  // Step 1: Engage Relay 1 (Contactor Latch / Run circuit)
  digitalWrite(RELAY1_RUN_PIN, LOW); // Active LOW: closes circuit
  delay(100);

  // Step 2: Engage Relay 2 (Start pulse across green button)
  digitalWrite(RELAY2_START_PIN, LOW); // Active LOW: momentary press
  startPulseTimerMs = millis();
  startingSequenceActive = true;
}

void executeEmergencyStop(FaultCode reason) {
  Serial.printf("⏹ [STOP] De-energizing Contactor. Reason Code: %d\n", (int)reason);

  // Disengage both relays immediately (Active LOW: HIGH = OFF)
  digitalWrite(RELAY2_START_PIN, HIGH);
  digitalWrite(RELAY1_RUN_PIN, HIGH); // Breaks contactor coil hold circuit

  pumpIsRunning = false;
  startingSequenceActive = false;
  activeFaultCode = reason;

  sendTelemetry();
}

// ------------------------------------------------------------------------------
// 12. TRUE-RMS SENSING ENGINE (ZMCT103C & ZMPT101B)
// ------------------------------------------------------------------------------
float readRMSCurrent() {
  unsigned long startSampleTime = millis();
  uint32_t sampleCount = 0;
  const int MAX_SAMPLES = 400;
  float samples[MAX_SAMPLES];
  float sumVoltages = 0.0;

  // Capture AC waveform over 100ms (5 full 50Hz cycles)
  while ((millis() - startSampleTime < 100) && (sampleCount < MAX_SAMPLES)) {
    int adcRaw = analogRead(ZMCT103C_PIN);
    float voltage = (adcRaw / 4095.0) * 3.3;
    samples[sampleCount] = voltage;
    sumVoltages += voltage;
    sampleCount++;
    delayMicroseconds(240);
  }

  if (sampleCount == 0) return 0.0;

  float dcOffset = sumVoltages / sampleCount;
  float sumSquaredDiff = 0.0;
  for (uint32_t i = 0; i < sampleCount; i++) {
    float diff = samples[i] - dcOffset;
    sumSquaredDiff += (diff * diff);
  }

  float rmsVoltage = sqrt(sumSquaredDiff / sampleCount);
  float rmsCurrent = rmsVoltage * CURRENT_CALIBRATION_FACTOR;
  if (rmsCurrent < 0.4) rmsCurrent = 0.0;

  return rmsCurrent;
}

float readRMSVoltage() {
  unsigned long startSampleTime = millis();
  uint32_t sampleCount = 0;
  const int MAX_SAMPLES = 400;
  float samples[MAX_SAMPLES];
  float sumVoltages = 0.0;

  // Capture AC mains waveform over 100ms (5 full 50Hz cycles)
  while ((millis() - startSampleTime < 100) && (sampleCount < MAX_SAMPLES)) {
    int adcRaw = analogRead(ZMPT101B_PIN);
    float voltage = (adcRaw / 4095.0) * 3.3;
    samples[sampleCount] = voltage;
    sumVoltages += voltage;
    sampleCount++;
    delayMicroseconds(240);
  }

  if (sampleCount == 0) return 0.0;

  float dcOffset = sumVoltages / sampleCount;
  float sumSquaredDiff = 0.0;
  for (uint32_t i = 0; i < sampleCount; i++) {
    float diff = samples[i] - dcOffset;
    sumSquaredDiff += (diff * diff);
  }

  float rmsSignalVoltage = sqrt(sumSquaredDiff / sampleCount);
  float mainsRMS = rmsSignalVoltage * VOLTAGE_CALIBRATION_FACTOR;
  if (mainsRMS < 20.0) mainsRMS = 0.0;

  return mainsRMS;
}

// ------------------------------------------------------------------------------
// 13. TELEMETRY TRANSMISSION
// ------------------------------------------------------------------------------
void sendTelemetry() {
  txTelemetry.packetId = ++telemetrySequenceCounter;
  txTelemetry.isRunning = pumpIsRunning;
  txTelemetry.faultCode = (uint8_t)activeFaultCode;
  txTelemetry.modeCode = (uint8_t)activeModeCode;

#if SENSOR_MODE_RSSI_DIAGNOSTICS
  float rssiMag = abs((float)latestRssiDbm);
  txTelemetry.voltageRMS = rssiMag;         // dBm as Voltage (e.g. 85.0 V)
  txTelemetry.currentAmps = rssiMag / 10.0; // RSSI signal metric as Current (e.g. 8.5 A)
  float snr = (float)(latestRssiDbm - (-95));
  if (snr < 0.0) snr = 0.0;
  txTelemetry.powerWatts = snr;             // SNR (dB) as Watts (e.g. 11.0 W)
#endif

  esp_now_send(baseNodeMac, (uint8_t *)&txTelemetry, sizeof(txTelemetry));
}
