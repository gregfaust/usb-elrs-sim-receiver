#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <string.h>
#include <stdlib.h>


#if defined(ARDUINO_ARCH_RP2040)
  #include <EEPROM.h>
#else
  #include <FlashStorage.h>
#endif

// ------------------------------------------------------------
// HID descriptor: 8 axes, 16-bit each + 16 buttons + keyboard
// ------------------------------------------------------------

#define TUD_HID_REPORT_DESC_GAMEPAD_16BTN(...) \
  HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP), \
  HID_USAGE(HID_USAGE_DESKTOP_GAMEPAD), \
  HID_COLLECTION(HID_COLLECTION_APPLICATION), \
  __VA_ARGS__ \
  HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP), \
  HID_USAGE(HID_USAGE_DESKTOP_X), \
  HID_USAGE(HID_USAGE_DESKTOP_Y), \
  HID_USAGE(HID_USAGE_DESKTOP_Z), \
  HID_USAGE(HID_USAGE_DESKTOP_RX), \
  HID_USAGE(HID_USAGE_DESKTOP_RY), \
  HID_USAGE(HID_USAGE_DESKTOP_RZ), \
  HID_USAGE(HID_USAGE_DESKTOP_SLIDER), \
  HID_USAGE(HID_USAGE_DESKTOP_DIAL), \
  HID_LOGICAL_MIN(0), \
  HID_LOGICAL_MAX_N(0xffff, 3), \
  HID_REPORT_COUNT(8), \
  HID_REPORT_SIZE(16), \
  HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE), \
  HID_USAGE_PAGE(HID_USAGE_PAGE_BUTTON), \
  HID_USAGE_MIN(1), \
  HID_USAGE_MAX(16), \
  HID_LOGICAL_MIN(0), \
  HID_LOGICAL_MAX(1), \
  HID_REPORT_COUNT(16), \
  HID_REPORT_SIZE(1), \
  HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE), \
  HID_COLLECTION_END

enum {
  RID_GAMEPAD  = 1,
  RID_KEYBOARD = 2
};

Adafruit_USBD_HID usb_hid;

uint8_t const desc_hid_report[] = {
  TUD_HID_REPORT_DESC_GAMEPAD_16BTN(HID_REPORT_ID(RID_GAMEPAD)),
  TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(RID_KEYBOARD))
};

// ------------------------------------------------------------
// CRSF definitions
// ------------------------------------------------------------

#define CRSF_BAUDRATE 420000
#define CRSF_MAX_PACKET_LEN 64
#define CRSF_NUM_CHANNELS 16

#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16

#define CRSF_CH_MIN_DEFAULT 172
#define CRSF_CH_MID_DEFAULT 992
#define CRSF_CH_MAX_DEFAULT 1811

// Button zones for CRSF normal range.
#define CRSF_LOW_ZONE_MAX 600
#define CRSF_HIGH_ZONE_MIN 1400

// CRSF UART selection.
#if defined(BOARD_PICO) || defined(ARDUINO_RASPBERRY_PI_PICO)
  // Raspberry Pi Pico: 
  #define CRSF_SERIAL Serial2
  #define CRSF_CUSTOM_PINS
  #define CRSF_TX_PIN 4
  #define CRSF_RX_PIN 5

#elif defined(BOARD_XIAO_RP2040)
  // Seeed XIAO RP2040: 
  #define CRSF_SERIAL Serial1

#elif defined(BOARD_XIAO_SAMD21)
  // Seeed XIAO SAMD21
  #define CRSF_SERIAL Serial1

#else

  #define CRSF_SERIAL Serial1

#endif

// HID axis source mapping.
// Axis 1..4 = CH1..CH4
// Axis 5..8 = CH6..CH9
// CH5 and CH10..CH16 are normally useful as AUX/button channels.
static const uint8_t AXIS_SOURCE[8] = {
  0, 1, 2, 3, 5, 6, 7, 8
};

// ------------------------------------------------------------
// Config structures
// ------------------------------------------------------------

#define CONFIG_MAGIC 0x45524C53UL  // "ELRS"
#define CONFIG_VERSION 2

#define BTN_OFF  0
#define BTN_LOW  1
#define BTN_MID  2
#define BTN_HIGH 3

#define EEPROM_CONFIG_SIZE 512

struct AxisConfig {
  uint16_t min;
  uint16_t mid;
  uint16_t max;
  uint16_t deadband;
  uint8_t invert;
};

struct ButtonConfig {
  uint8_t enabled;
  uint8_t channel;   // 0..15
  uint8_t position;  // BTN_LOW / BTN_MID / BTN_HIGH
};

struct DeviceConfig {
  uint32_t magic;
  uint16_t version;
  AxisConfig axis[8];
  ButtonConfig button[16];

  // Keyboard HID usage per logical button.
  // 0 = OFF. When key[i] != 0, logical button i sends keyboard only,
  // and is not sent as a joystick button to avoid duplicate input.
  uint8_t key[16];
};

#if !defined(ARDUINO_ARCH_RP2040)
  FlashStorage(configStore, DeviceConfig);
#endif

DeviceConfig config;

// ------------------------------------------------------------
// HID report structure
// ------------------------------------------------------------

typedef struct __attribute__((packed)) {
  uint16_t ch[8];
  uint16_t buttons;
} gp_t;

static gp_t gp;

// Logical button state before output filtering.
// This is what the CRSF/button remapper produces.
static uint16_t logicalButtons = 0;

// Last keyboard report sent.
static uint8_t lastKeyboardKeys[6] = {0};

// ------------------------------------------------------------
// Runtime state
// ------------------------------------------------------------

uint8_t rxbuf[CRSF_MAX_PACKET_LEN + 3];
uint8_t rxPos = 0;
uint8_t frameSize = 0;
uint32_t gaptime = 0;

uint16_t rawCh[CRSF_NUM_CHANNELS];
uint16_t hidAxis[8];

bool hasSignal = false;
bool datardyf = false;
bool calibrating = false;
bool streamEnabled = false;

uint32_t lastPacketMs = 0;
uint32_t lastStreamMs = 0;

// ------------------------------------------------------------
// Forward declarations
// ------------------------------------------------------------

void loadDefaults();
void loadConfig();
void saveConfig();
void crsf();
void crsfdecode();
void handleUsbSerial();
void processCfgCommand(char *line);
void printConfig();
void printData();
void updateCalibration();
void updateButtons();
uint16_t mapAxis(uint16_t raw, const AxisConfig &cfg);
bool channelMatchesPosition(uint16_t raw, uint8_t pos);
void sendKeyboardFromButtons(uint16_t buttons);
uint8_t parseKeyCode(const char *s);
void printKeyName(uint8_t key);

// ------------------------------------------------------------
// Config
// ------------------------------------------------------------

void setButtonDefault(uint8_t buttonIndex, uint8_t channel, uint8_t position)
{
  if (buttonIndex >= 16 || channel >= 16) return;

  config.button[buttonIndex].enabled = 1;
  config.button[buttonIndex].channel = channel;
  config.button[buttonIndex].position = position;
}

void loadDefaults()
{
  config.magic = CONFIG_MAGIC;
  config.version = CONFIG_VERSION;

  for (uint8_t i = 0; i < 8; i++) {
    config.axis[i].min = CRSF_CH_MIN_DEFAULT;
    config.axis[i].mid = CRSF_CH_MID_DEFAULT;
    config.axis[i].max = CRSF_CH_MAX_DEFAULT;
    config.axis[i].deadband = 3;
    config.axis[i].invert = 0;
  }

  for (uint8_t i = 0; i < 16; i++) {
    config.button[i].enabled = 0;
    config.button[i].channel = 0;
    config.button[i].position = BTN_OFF;
    config.key[i] = HID_KEY_NONE;
  }

  // Sensible defaults:
  // Button 1..3: CH5 LOW/MID/HIGH
  // Button 4..10: CH10..CH16 HIGH
  setButtonDefault(0, 4,  BTN_LOW);   // CH5 LOW
  setButtonDefault(1, 4,  BTN_MID);   // CH5 MID
  setButtonDefault(2, 4,  BTN_HIGH);  // CH5 HIGH
  setButtonDefault(3, 9,  BTN_HIGH);  // CH10 HIGH
  setButtonDefault(4, 10, BTN_HIGH);  // CH11 HIGH
  setButtonDefault(5, 11, BTN_HIGH);  // CH12 HIGH
  setButtonDefault(6, 12, BTN_HIGH);  // CH13 HIGH
  setButtonDefault(7, 13, BTN_HIGH);  // CH14 HIGH
  setButtonDefault(8, 14, BTN_HIGH);  // CH15 HIGH
  setButtonDefault(9, 15, BTN_HIGH);  // CH16 HIGH
}

void loadConfig()
{
#if defined(ARDUINO_ARCH_RP2040)
  EEPROM.begin(EEPROM_CONFIG_SIZE);

  uint8_t *ptr = (uint8_t *)&config;

  for (size_t i = 0; i < sizeof(DeviceConfig); i++) {
    ptr[i] = EEPROM.read(i);
  }
#else
  config = configStore.read();
#endif

  if (config.magic != CONFIG_MAGIC || config.version != CONFIG_VERSION) {
    loadDefaults();
    saveConfig();
  }
}

void saveConfig()
{
  config.magic = CONFIG_MAGIC;
  config.version = CONFIG_VERSION;

#if defined(ARDUINO_ARCH_RP2040)
  const uint8_t *ptr = (const uint8_t *)&config;

  for (size_t i = 0; i < sizeof(DeviceConfig); i++) {
    EEPROM.write(i, ptr[i]);
  }

  EEPROM.commit();
#else
  configStore.write(config);
#endif
}

// ------------------------------------------------------------
// Axis and button mapping
// ------------------------------------------------------------

uint16_t mapAxis(uint16_t raw, const AxisConfig &cfg)
{
  uint16_t minv = cfg.min;
  uint16_t midv = cfg.mid;
  uint16_t maxv = cfg.max;

  if (maxv <= minv + 10) {
    return 32767;
  }

  if (raw < minv) raw = minv;
  if (raw > maxv) raw = maxv;

  int32_t value = 32767;

  // If midpoint is invalid, fall back to simple linear min/max mapping.
  if (midv <= minv + 2 || midv >= maxv - 2) {
    value = (int32_t)(raw - minv) * 65535L / (maxv - minv);
  } else {
    uint16_t db = cfg.deadband;

    if (raw >= midv - db && raw <= midv + db) {
      value = 32767;
    } else if (raw < midv) {
      value = (int32_t)(raw - minv) * 32767L / (midv - minv);
    } else {
      value = 32767L + (int32_t)(raw - midv) * 32768L / (maxv - midv);
    }
  }

  if (value < 0) value = 0;
  if (value > 65535) value = 65535;

  if (cfg.invert) {
    value = 65535 - value;
  }

  return (uint16_t)value;
}

bool channelMatchesPosition(uint16_t raw, uint8_t pos)
{
  switch (pos) {
    case BTN_LOW:
      return raw < CRSF_LOW_ZONE_MAX;

    case BTN_MID:
      return raw >= CRSF_LOW_ZONE_MAX && raw <= CRSF_HIGH_ZONE_MIN;

    case BTN_HIGH:
      return raw > CRSF_HIGH_ZONE_MIN;

    default:
      return false;
  }
}

void updateCalibration()
{
  if (!calibrating || !hasSignal) return;

  for (uint8_t i = 0; i < 8; i++) {
    uint8_t src = AXIS_SOURCE[i];
    uint16_t raw = rawCh[src];

    if (raw < config.axis[i].min) config.axis[i].min = raw;
    if (raw > config.axis[i].max) config.axis[i].max = raw;
  }
}

void updateButtons()
{
  logicalButtons = 0;
  gp.buttons = 0;

  for (uint8_t i = 0; i < 16; i++) {
    ButtonConfig &b = config.button[i];

    if (!b.enabled) continue;
    if (b.channel >= CRSF_NUM_CHANNELS) continue;

    if (channelMatchesPosition(rawCh[b.channel], b.position)) {
      logicalButtons |= (1 << i);

      // If the logical button has a keyboard key assigned, do not also
      // send it as a joystick button. This avoids double input in games.
      if (config.key[i] == HID_KEY_NONE) {
        gp.buttons |= (1 << i);
      }
    }
  }
}

void sendKeyboardFromButtons(uint16_t buttons)
{
  uint8_t keys[6] = {0};
  uint8_t count = 0;

  for (uint8_t i = 0; i < 16 && count < 6; i++) {
    if ((buttons & (1 << i)) && config.key[i] != HID_KEY_NONE) {
      keys[count++] = config.key[i];
    }
  }

  if (memcmp(keys, lastKeyboardKeys, sizeof(keys)) != 0) {
    usb_hid.keyboardReport(RID_KEYBOARD, 0, keys);
    memcpy(lastKeyboardKeys, keys, sizeof(keys));
  }
}

// CFG KEY accepts numeric HID keyboard usage codes.
// Examples:
// CFG KEY 1 20   -> Button 1 sends Q
// CFG KEY 1 44   -> Button 1 sends Space
// CFG KEY 1 0    -> Button 1 sends no key and remains joystick output
uint8_t parseKeyCode(const char *s)
{
  if (!s) return HID_KEY_NONE;

  if (strcmp(s, "OFF") == 0) return HID_KEY_NONE;
  if (strcmp(s, "NONE") == 0) return HID_KEY_NONE;

  char *end = nullptr;
  long value = strtol(s, &end, 10);

  if (end != s && *end == '\0' && value >= 0 && value <= 255) {
    return (uint8_t)value;
  }

  // Optional text aliases for manual serial use.
  if (strlen(s) == 1) {
    char c = s[0];

    if (c >= 'a' && c <= 'z') return HID_KEY_A + (c - 'a');
    if (c >= 'A' && c <= 'Z') return HID_KEY_A + (c - 'A');
    if (c >= '1' && c <= '9') return HID_KEY_1 + (c - '1');
    if (c == '0') return HID_KEY_0;
  }

  if (strcmp(s, "SPACE") == 0) return HID_KEY_SPACE;
  if (strcmp(s, "ENTER") == 0) return HID_KEY_END;
  if (strcmp(s, "ESC") == 0) return HID_KEY_ESCAPE;
  if (strcmp(s, "ESCAPE") == 0) return HID_KEY_ESCAPE;
  if (strcmp(s, "TAB") == 0) return HID_KEY_TAB;
  if (strcmp(s, "BACKSPACE") == 0) return HID_KEY_BACKSPACE;
  if (strcmp(s, "UP") == 0) return HID_KEY_ARROW_UP;
  if (strcmp(s, "DOWN") == 0) return HID_KEY_ARROW_DOWN;
  if (strcmp(s, "LEFT") == 0) return HID_KEY_ARROW_LEFT;
  if (strcmp(s, "RIGHT") == 0) return HID_KEY_ARROW_RIGHT;

  return HID_KEY_NONE;
}

void printKeyName(uint8_t key)
{
  if (key == HID_KEY_NONE) {
    Serial.print("OFF");
  } else if (key >= HID_KEY_A && key <= HID_KEY_Z) {
    Serial.print((char)('A' + key - HID_KEY_A));
  } else if (key >= HID_KEY_1 && key <= HID_KEY_9) {
    Serial.print((char)('1' + key - HID_KEY_1));
  } else if (key == HID_KEY_0) {
    Serial.print("0");
  } else if (key == HID_KEY_SPACE) {
    Serial.print("SPACE");
  } else if (key == HID_KEY_ESCAPE) {
    Serial.print("ESC");
  } else if (key == HID_KEY_TAB) {
    Serial.print("TAB");
  } else if (key == HID_KEY_BACKSPACE) {
    Serial.print("BACKSPACE");
  } else if (key == HID_KEY_ARROW_UP) {
    Serial.print("UP");
  } else if (key == HID_KEY_ARROW_DOWN) {
    Serial.print("DOWN");
  } else if (key == HID_KEY_ARROW_LEFT) {
    Serial.print("LEFT");
  } else if (key == HID_KEY_ARROW_RIGHT) {
    Serial.print("RIGHT");
  } else {
    Serial.print(key);
  }
}

// ------------------------------------------------------------
// Setup / Loop
// ------------------------------------------------------------

void setup()
{
  memset(rawCh, 0, sizeof(rawCh));
  memset(hidAxis, 0, sizeof(hidAxis));
  memset(&gp, 0, sizeof(gp));
  memset(lastKeyboardKeys, 0, sizeof(lastKeyboardKeys));
  logicalButtons = 0;

  loadConfig();

  USBDevice.setProductDescriptor("USB EspressLRS");
  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.begin();

  while (!USBDevice.mounted()) {
    delay(1);
  }

  Serial.begin(CRSF_BAUDRATE);

#if defined(CRSF_CUSTOM_PINS)
  CRSF_SERIAL.setTX(CRSF_TX_PIN);
  CRSF_SERIAL.setRX(CRSF_RX_PIN);
#endif

  CRSF_SERIAL.begin(CRSF_BAUDRATE, SERIAL_8N1);

  rxPos = 0;
  frameSize = 0;
  datardyf = false;
  calibrating = false;
  streamEnabled = false;
}

void loop()
{
  if (USBDevice.suspended()) {
    USBDevice.remoteWakeup();
  }

  crsf();
  handleUsbSerial();

  // Detect signal lost
  if (hasSignal && millis() - lastPacketMs > 500) {
  hasSignal = false;
  } // testing

  if (datardyf) {
    if (usb_hid.ready()) {
      usb_hid.sendReport(RID_GAMEPAD, &gp, sizeof(gp));
    }

    delay(2);

    if (usb_hid.ready()) {
      sendKeyboardFromButtons(logicalButtons);
    }

    datardyf = false;
  }

  if (streamEnabled && millis() - lastStreamMs >= 50) {
    printData();
    lastStreamMs = millis();
  }
}

// ------------------------------------------------------------
// CRSF receive process
// ------------------------------------------------------------

void crsf()
{
  uint8_t data;

  while (CRSF_SERIAL.available()) {
    data = CRSF_SERIAL.read();
    gaptime = micros();

    if (rxPos >= sizeof(rxbuf)) {
      rxPos = 0;
    }

    if (rxPos == 1) {
      frameSize = data;

      if (frameSize > CRSF_MAX_PACKET_LEN) {
        rxPos = 0;
        frameSize = 0;
        return;
      }
    }

    rxbuf[rxPos++] = data;

    if (rxPos > 1 && rxPos >= frameSize + 2) {
      crsfdecode();
      rxPos = 0;
      frameSize = 0;
    }
  }

  if (rxPos > 0 && micros() - gaptime > 800) {
    rxPos = 0;
    frameSize = 0;
  }
}

void crsfdecode()
{
  if (rxbuf[0] != CRSF_ADDRESS_FLIGHT_CONTROLLER) return;
  if (rxbuf[2] != CRSF_FRAMETYPE_RC_CHANNELS_PACKED) return;

  rawCh[0]  = (rxbuf[3]  | rxbuf[4]  << 8) & 0x07ff;
  rawCh[1]  = (rxbuf[4]  >> 3 | rxbuf[5]  << 5) & 0x07ff;
  rawCh[2]  = (rxbuf[5]  >> 6 | rxbuf[6]  << 2 | rxbuf[7]  << 10) & 0x07ff;
  rawCh[3]  = (rxbuf[7]  >> 1 | rxbuf[8]  << 7) & 0x07ff;
  rawCh[4]  = (rxbuf[8]  >> 4 | rxbuf[9]  << 4) & 0x07ff;
  rawCh[5]  = (rxbuf[9]  >> 7 | rxbuf[10] << 1 | rxbuf[11] << 9) & 0x07ff;
  rawCh[6]  = (rxbuf[11] >> 2 | rxbuf[12] << 6) & 0x07ff;
  rawCh[7]  = (rxbuf[12] >> 5 | rxbuf[13] << 3) & 0x07ff;
  rawCh[8]  = (rxbuf[14] | rxbuf[15] << 8) & 0x07ff;
  rawCh[9]  = (rxbuf[15] >> 3 | rxbuf[16] << 5) & 0x07ff;
  rawCh[10] = (rxbuf[16] >> 6 | rxbuf[17] << 2 | rxbuf[18] << 10) & 0x07ff;
  rawCh[11] = (rxbuf[18] >> 1 | rxbuf[19] << 7) & 0x07ff;
  rawCh[12] = (rxbuf[19] >> 4 | rxbuf[20] << 4) & 0x07ff;
  rawCh[13] = (rxbuf[20] >> 7 | rxbuf[21] << 1 | rxbuf[22] << 9) & 0x07ff;
  rawCh[14] = (rxbuf[22] >> 2 | rxbuf[23] << 6) & 0x07ff;
  rawCh[15] = (rxbuf[23] >> 5 | rxbuf[24] << 3) & 0x07ff;

  hasSignal = true;
  lastPacketMs = millis();

  updateCalibration();

  for (uint8_t i = 0; i < 8; i++) {
    uint8_t src = AXIS_SOURCE[i];
    hidAxis[i] = mapAxis(rawCh[src], config.axis[i]);
    gp.ch[i] = hidAxis[i];
  }

  updateButtons();

  datardyf = true;
}

// ------------------------------------------------------------
// USB Serial CFG protocol
// ------------------------------------------------------------

void flushToCrsf(const char *buf, uint8_t len)
{
  for (uint8_t i = 0; i < len; i++) {
    CRSF_SERIAL.write((uint8_t)buf[i]);
  }
}

// Intercepts lines beginning with "CFG".
// Other serial data is forwarded to the CRSF UART as basic passthrough.
void handleUsbSerial()
{
  static char line[128];
  static uint8_t idx = 0;
  static bool collecting = false;
  static uint32_t startMs = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();

    if (!collecting) {
      if (c == 'C') {
        collecting = true;
        idx = 0;
        startMs = millis();
        line[idx++] = c;
      } else {
        CRSF_SERIAL.write((uint8_t)c);
      }
    } else {
      if (idx < sizeof(line) - 1) {
        line[idx++] = c;
      }

      if (idx == 3 && strncmp(line, "CFG", 3) != 0) {
        flushToCrsf(line, idx);
        idx = 0;
        collecting = false;
        continue;
      }

      if (c == '\n' || c == '\r') {
        line[idx] = '\0';

        if (strncmp(line, "CFG", 3) == 0) {
          processCfgCommand(line);
        } else {
          flushToCrsf(line, idx);
        }

        idx = 0;
        collecting = false;
      }
    }
  }

  // Avoid hanging forever on accidental passthrough bytes starting with 'C'.
  if (collecting && millis() - startMs > 250 && idx < 3) {
    flushToCrsf(line, idx);
    idx = 0;
    collecting = false;
  }
}

uint8_t parsePosition(const char *s)
{
  if (!s) return BTN_OFF;
  if (strcmp(s, "LOW") == 0) return BTN_LOW;
  if (strcmp(s, "MID") == 0) return BTN_MID;
  if (strcmp(s, "HIGH") == 0) return BTN_HIGH;
  if (strcmp(s, "OFF") == 0) return BTN_OFF;
  return BTN_OFF;
}

const char *positionName(uint8_t pos)
{
  switch (pos) {
    case BTN_LOW: return "LOW";
    case BTN_MID: return "MID";
    case BTN_HIGH: return "HIGH";
    default: return "OFF";
  }
}

void processCfgCommand(char *line)
{
  char *tok = strtok(line, " \t\r\n");
  if (!tok || strcmp(tok, "CFG") != 0) return;

  char *cmd = strtok(NULL, " \t\r\n");
  if (!cmd) {
    Serial.println("ERR missing_command");
    return;
  }

  if (strcmp(cmd, "GET") == 0) {
    printConfig();
    return;
  }

  if (strcmp(cmd, "STREAM") == 0) {
    char *v = strtok(NULL, " \t\r\n");
    streamEnabled = v && atoi(v) != 0;
    Serial.println(streamEnabled ? "OK STREAM 1" : "OK STREAM 0");
    return;
  }

  if (strcmp(cmd, "CENTER") == 0) {
    if (!hasSignal) {
      Serial.println("ERR no_crsf_signal");
      return;
    }

    for (uint8_t i = 0; i < 8; i++) {
      uint8_t src = AXIS_SOURCE[i];
      config.axis[i].mid = rawCh[src];
    }

    Serial.println("OK CENTER");
    printConfig();
    return;
  }

  if (strcmp(cmd, "CALSTART") == 0) {
    if (!hasSignal) {
      Serial.println("ERR no_crsf_signal");
      return;
    }

    for (uint8_t i = 0; i < 8; i++) {
      uint8_t src = AXIS_SOURCE[i];
      config.axis[i].min = rawCh[src];
      config.axis[i].max = rawCh[src];
    }

    calibrating = true;
    Serial.println("OK CALSTART");
    return;
  }

  if (strcmp(cmd, "CALSTOP") == 0) {
    calibrating = false;

    for (uint8_t i = 0; i < 8; i++) {
      if (config.axis[i].min > config.axis[i].mid) {
        config.axis[i].min = CRSF_CH_MIN_DEFAULT;
      }

      if (config.axis[i].max < config.axis[i].mid) {
        config.axis[i].max = CRSF_CH_MAX_DEFAULT;
      }

      if (config.axis[i].max <= config.axis[i].min + 10) {
        config.axis[i].min = CRSF_CH_MIN_DEFAULT;
        config.axis[i].mid = CRSF_CH_MID_DEFAULT;
        config.axis[i].max = CRSF_CH_MAX_DEFAULT;
      }
    }

    Serial.println("OK CALSTOP");
    printConfig();
    return;
  }

  if (strcmp(cmd, "SAVE") == 0) {
    saveConfig();
    Serial.println("OK SAVE");
    return;
  }

  if (strcmp(cmd, "DEFAULTS") == 0) {
    loadDefaults();
    saveConfig();
    Serial.println("OK DEFAULTS");
    printConfig();
    return;
  }

  if (strcmp(cmd, "AXIS") == 0) {
    char *axisStr = strtok(NULL, " \t\r\n");
    char *minStr = strtok(NULL, " \t\r\n");
    char *midStr = strtok(NULL, " \t\r\n");
    char *maxStr = strtok(NULL, " \t\r\n");
    char *invStr = strtok(NULL, " \t\r\n");
    char *dbStr = strtok(NULL, " \t\r\n");

    if (!axisStr || !minStr || !midStr || !maxStr || !invStr || !dbStr) {
      Serial.println("ERR axis_args");
      return;
    }

    int axis = atoi(axisStr) - 1;
    if (axis < 0 || axis >= 8) {
      Serial.println("ERR axis_range");
      return;
    }

    config.axis[axis].min = constrain(atoi(minStr), 0, 2047);
    config.axis[axis].mid = constrain(atoi(midStr), 0, 2047);
    config.axis[axis].max = constrain(atoi(maxStr), 0, 2047);
    config.axis[axis].invert = atoi(invStr) ? 1 : 0;
    config.axis[axis].deadband = constrain(atoi(dbStr), 0, 200);

    Serial.println("OK AXIS");
    return;
  }

  if (strcmp(cmd, "BTN") == 0) {
    char *btnStr = strtok(NULL, " \t\r\n");
    char *chStr = strtok(NULL, " \t\r\n");
    char *posStr = strtok(NULL, " \t\r\n");

    if (!btnStr || !chStr || !posStr) {
      Serial.println("ERR btn_args");
      return;
    }

    int btn = atoi(btnStr) - 1;
    int ch = atoi(chStr) - 1;
    uint8_t pos = parsePosition(posStr);

    if (btn < 0 || btn >= 16) {
      Serial.println("ERR btn_range");
      return;
    }

    if (pos == BTN_OFF) {
      config.button[btn].enabled = 0;
      config.button[btn].channel = 0;
      config.button[btn].position = BTN_OFF;
      Serial.println("OK BTN");
      return;
    }

    if (ch < 0 || ch >= 16) {
      Serial.println("ERR channel_range");
      return;
    }

    config.button[btn].enabled = 1;
    config.button[btn].channel = ch;
    config.button[btn].position = pos;

    Serial.println("OK BTN");
    return;
  }

  if (strcmp(cmd, "KEY") == 0) {
    char *btnStr = strtok(NULL, " \t\r\n");
    char *keyStr = strtok(NULL, " \t\r\n");

    if (!btnStr || !keyStr) {
      Serial.println("ERR key_args");
      return;
    }

    int btn = atoi(btnStr) - 1;

    if (btn < 0 || btn >= 16) {
      Serial.println("ERR btn_range");
      return;
    }

    config.key[btn] = parseKeyCode(keyStr);

    Serial.println("OK KEY");
    return;
  }

  if (strcmp(cmd, "BTNCLR") == 0) {
    for (uint8_t i = 0; i < 16; i++) {
      config.button[i].enabled = 0;
      config.button[i].channel = 0;
      config.button[i].position = BTN_OFF;
      config.key[i] = HID_KEY_NONE;
    }

    Serial.println("OK BTNCLR");
    printConfig();
    return;
  }

  Serial.println("ERR unknown_command");
}

// ------------------------------------------------------------
// JSON output
// ------------------------------------------------------------

void printConfig()
{
  Serial.print("CFG {\"version\":");
  Serial.print(CONFIG_VERSION);

  Serial.print(",\"axisSource\":[");
  for (uint8_t i = 0; i < 8; i++) {
    if (i) Serial.print(",");
    Serial.print(AXIS_SOURCE[i] + 1);
  }
  Serial.print("]");

  Serial.print(",\"axes\":[");
  for (uint8_t i = 0; i < 8; i++) {
    if (i) Serial.print(",");
    Serial.print("{\"min\":");
    Serial.print(config.axis[i].min);
    Serial.print(",\"mid\":");
    Serial.print(config.axis[i].mid);
    Serial.print(",\"max\":");
    Serial.print(config.axis[i].max);
    Serial.print(",\"invert\":");
    Serial.print(config.axis[i].invert);
    Serial.print(",\"deadband\":");
    Serial.print(config.axis[i].deadband);
    Serial.print("}");
  }
  Serial.print("]");

  Serial.print(",\"buttons\":[");
  for (uint8_t i = 0; i < 16; i++) {
    if (i) Serial.print(",");
    Serial.print("{\"enabled\":");
    Serial.print(config.button[i].enabled);
    Serial.print(",\"channel\":");
    Serial.print(config.button[i].channel + 1);
    Serial.print(",\"position\":\"");
    Serial.print(positionName(config.button[i].position));
    Serial.print("\"}");
  }
  Serial.print("]");

  Serial.print(",\"keys\":[");
  for (uint8_t i = 0; i < 16; i++) {
    if (i) Serial.print(",");
    Serial.print(config.key[i]);
  }
  Serial.print("]");

  Serial.print(",\"keyNames\":[");
  for (uint8_t i = 0; i < 16; i++) {
    if (i) Serial.print(",");
    Serial.print("\"");
    printKeyName(config.key[i]);
    Serial.print("\"");
  }
  Serial.print("]");

  Serial.print(",\"calibrating\":");
  Serial.print(calibrating ? 1 : 0);
  Serial.println("}");
}

void printData()
{
  Serial.print("DATA {\"signal\":");
  Serial.print(hasSignal ? 1 : 0);

  Serial.print(",\"raw\":[");
  for (uint8_t i = 0; i < 16; i++) {
    if (i) Serial.print(",");
    Serial.print(rawCh[i]);
  }
  Serial.print("]");

  Serial.print(",\"hid\":[");
  for (uint8_t i = 0; i < 8; i++) {
    if (i) Serial.print(",");
    Serial.print(hidAxis[i]);
  }
  Serial.print("]");

  // Logical buttons are used for the UI, so key-mapped buttons remain visible.
  Serial.print(",\"buttons\":");
  Serial.print(logicalButtons);

  // Actual joystick buttons sent in the gamepad report after keyboard filtering.
  Serial.print(",\"joystickButtons\":");
  Serial.print(gp.buttons);

  Serial.print(",\"calibrating\":");
  Serial.print(calibrating ? 1 : 0);

  Serial.println("}");
}
