struct AnalysisResult;  // Forward declaration for Arduino auto-generated prototypes.
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <mbedtls/base64.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_camera.h"
#include <Preferences.h>
#include <time.h>
#include <esp_wifi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ESP32_OV5640_AF.h"

OV5640 ov5640 = OV5640();
Preferences prefs;

String wifiSSID;
String wifiPASS;
String openAIKey;

// Use a low-cost vision-capable model.
// You can change this to another vision-capable model you have access to.
const char* OPENAI_MODEL = "gpt-5.5";

// Buttons
#define BTN_UP   1
#define BTN_DOWN 2
#define BTN_SNAP 3

// =========================
// OLED CONFIG
// =========================
#define OLED_SDA 5
#define OLED_SCL 6
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define OLED_ADDR 0x3C
uint8_t detectedOLEDAddr = OLED_ADDR;

// =========================
// XIAO ESP32S3 Sense + OV3660 camera pins
// =========================
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   10
#define SIOD_GPIO_NUM   40
#define SIOC_GPIO_NUM   39
#define Y9_GPIO_NUM     48
#define Y8_GPIO_NUM     11
#define Y7_GPIO_NUM     12
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     16
#define Y4_GPIO_NUM     18
#define Y3_GPIO_NUM     17
#define Y2_GPIO_NUM     15
#define VSYNC_GPIO_NUM  38
#define HREF_GPIO_NUM   47
#define PCLK_GPIO_NUM   13

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// 128x32 OLED at text size 1 ~= 21 chars x 4 rows
static const int OLED_COLS = 21;
static const int OLED_ROWS = 4;
static const int MAX_LINES = 100;

String oledLines[MAX_LINES];
int oledLineCount = 0;
int oledTopLine = 0;

String currentPrompt =
"Analyze the photographed multiple-choice question. Select the best visible answer. "
"Read the question number, selected answer choice, and the full text of that selected answer. "
"All returned string values must use only normal ASCII characters.";

String lastResponse = "";
// =========================
// BATTERY / POWER TUNING
// =========================
// Battery saver is the default. It keeps quality decent, but avoids the
// expensive parts of the old sketch: very large photos, always-awake WiFi,
// continuous autofocus, high CPU speed while idle, and an always-on OLED.
static bool batterySaver = true;
static bool oledAutoOff = true;
static bool oledIsOn = true;

static const uint32_t OLED_IDLE_OFF_MS = 45000;        // Turn OLED off after 45s of no UI update.
static const uint32_t WIFI_IDLE_OFF_MS = 120000;       // Turn WiFi radio off after 2 min idle.
static const uint32_t SETTINGS_WAIT_REFRESH_MS = 1000; // OLED refresh while waiting for Serial input.

unsigned long lastActivityMs = 0;
unsigned long lastNetworkUseMs = 0;
bool ov5640Detected = false;

volatile bool chatLoadingActive = false;
TaskHandle_t chatLoadingTaskHandle = nullptr;

void noteActivity() {
  lastActivityMs = millis();
}

// =========================
// CAMERA / API TUNING
// =========================
// Lower number = better JPEG quality. Bigger frames help OCR, but huge JPEGs
// can break ESP32 memory or make the HTTPS POST fail. The code starts high and
// automatically falls back to smaller/safer capture profiles when needed.
static const uint32_t CAMERA_XCLK_HZ = 10000000;   // Keep 10 MHz: reliable on marginal camera/cable setups.
static const size_t MAX_JPEG_BYTES_FOR_API_SAVER = 180000;
static const size_t MAX_JPEG_BYTES_FOR_API_NORMAL = 320000;
static bool rawApiDebug = false;

size_t maxJpegBytesForApi() {
  return batterySaver ? MAX_JPEG_BYTES_FOR_API_SAVER : MAX_JPEG_BYTES_FOR_API_NORMAL;
}

enum PhotoMode { PHOTO_AUTO, PHOTO_HIGH, PHOTO_MEDIUM, PHOTO_LOW };
PhotoMode photoMode = PHOTO_MEDIUM;  // Better battery default: XGA/SVGA instead of trying UXGA first.

struct CaptureProfile {
  framesize_t frameSize;
  int jpegQuality;
  const char* name;
};

CaptureProfile captureProfiles[] = {
  { FRAMESIZE_XGA,  8, "XGA 1024x768 q8"  },
  { FRAMESIZE_SVGA, 9, "SVGA 800x600 q9"  },
  { FRAMESIZE_VGA,  9, "VGA 640x480 q9"   }
};

static const int CAPTURE_PROFILE_COUNT = sizeof(captureProfiles) / sizeof(captureProfiles[0]);

const char* photoModeName();

void setCpuForIdle() {
  if (batterySaver) {
    setCpuFrequencyMhz(80);
  }
}

void setCpuForBurst() {
  // 160 MHz is a good compromise: faster JPEG/base64/HTTPS without maxing out at 240 MHz.
  setCpuFrequencyMhz(batterySaver ? 160 : 240);
}

void applyWiFiPowerPolicy() {
  if (batterySaver) {
    WiFi.setSleep(true);                 // modem sleep while connected
    WiFi.setTxPower(WIFI_POWER_11dBm);   // enough for nearby router, lower peak current
  } else {
    WiFi.setSleep(false);                // more stable / responsive, more current
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
  }
}

void oledWake() {
  if (!oledIsOn) {
    display.ssd1306_command(SSD1306_DISPLAYON);
    oledIsOn = true;
  }
  noteActivity();
}

void oledMaybeSleep() {
  if (!batterySaver || !oledAutoOff || !oledIsOn) return;
  if (millis() - lastActivityMs > OLED_IDLE_OFF_MS) {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    oledIsOn = false;
  }
}

void wifiMaybeSleep() {
  if (!batterySaver) return;
  if (WiFi.status() == WL_CONNECTED && millis() - lastNetworkUseMs > WIFI_IDLE_OFF_MS) {
    Serial.println("Battery saver: WiFi radio off until next request.");
    WiFi.disconnect(false);
    WiFi.mode(WIFI_OFF);
  }
}

void enterIdlePowerState() {
  if (batterySaver) {
    applyWiFiPowerPolicy();
    setCpuForIdle();
  }
}

void testHTTPS() {
  WiFiClientSecure client;
  client.setInsecure();
  client.setHandshakeTimeout(30);

  HTTPClient https;
  https.setTimeout(30000);

  Serial.println("Testing HTTPS...");

  if (!https.begin(client, "https://api.openai.com/v1/models")) {
    Serial.println("HTTPS begin failed");
    return;
  }

  https.addHeader("Authorization", String("Bearer ") + openAIKey);

  int code = https.GET();
  String body = https.getString();

  Serial.print("HTTPS test code: ");
  Serial.println(code);
  Serial.println(body);

  https.end();
}

// =========================
// OLED TEXT HANDLING
// =========================
void wrapTextForOLED(const String& text) {
  oledLineCount = 0;
  oledTopLine = 0;

  String s = text;
  s.replace("\r", "");

  int start = 0;
  while (start < s.length() && oledLineCount < MAX_LINES) {
    int nl = s.indexOf('\n', start);
    String part;

    if (nl == -1) {
      part = s.substring(start);
      start = s.length();
    } else {
      part = s.substring(start, nl);
      start = nl + 1;
    }

    if (part.length() == 0) {
      oledLines[oledLineCount++] = "";
      continue;
    }

    while (part.length() > OLED_COLS && oledLineCount < MAX_LINES) {
      int cut = OLED_COLS;
      int spacePos = part.lastIndexOf(' ', cut);
      if (spacePos <= 0) spacePos = cut;

      oledLines[oledLineCount++] = part.substring(0, spacePos);
      part = part.substring(spacePos);
      part.trim();
    }

    if (oledLineCount < MAX_LINES) {
      oledLines[oledLineCount++] = part;
    }
  }

  if (oledLineCount == 0) {
    oledLines[oledLineCount++] = "";
  }
}

void renderOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  for (int i = 0; i < OLED_ROWS; i++) {
    int idx = oledTopLine + i;
    if (idx >= oledLineCount) break;
    display.setCursor(0, i * 8);
    display.print(oledLines[idx]);
  }

  display.display();
}

void showOLEDText(const String& text) {
  oledWake();
  lastResponse = text;
  wrapTextForOLED(text);
  renderOLED();
}

void clearOLEDScreen() {
  oledWake();
  lastResponse = "";
  oledLineCount = 0;
  oledTopLine = 0;
  display.clearDisplay();
  display.display();
}

void drawChatLoadingFrame(int dotCount) {
  if (!oledIsOn) {
    display.ssd1306_command(SSD1306_DISPLAYON);
    oledIsOn = true;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 8);
  display.print("Processing");
  for (int i = 0; i < dotCount; i++) {
    display.print(".");
  }

  display.display();
  lastActivityMs = millis();
}

void chatLoadingTask(void* parameter) {
  int dots = 1;

  while (chatLoadingActive) {
    drawChatLoadingFrame(dots);
    dots++;
    if (dots > 3) dots = 1;
    vTaskDelay(pdMS_TO_TICKS(350));
  }

  chatLoadingTaskHandle = nullptr;
  vTaskDelete(nullptr);
}

void startChatLoadingAnimation() {
  if (chatLoadingActive) return;

  oledWake();
  chatLoadingActive = true;

  BaseType_t ok = xTaskCreatePinnedToCore(
    chatLoadingTask,
    "chatLoading",
    4096,
    nullptr,
    1,
    &chatLoadingTaskHandle,
    0
  );

  if (ok != pdPASS) {
    chatLoadingActive = false;
    chatLoadingTaskHandle = nullptr;
    showOLEDText("Processing...");
  }
}

void stopChatLoadingAnimation() {
  if (!chatLoadingActive) return;

  chatLoadingActive = false;

  for (int i = 0; i < 30 && chatLoadingTaskHandle != nullptr; i++) {
    delay(20);
  }
}

void scrollUp() {
  oledWake();
  if (oledTopLine > 0) {
    oledTopLine--;
    renderOLED();
  }
}

void scrollDown() {
  oledWake();
  if (oledTopLine + OLED_ROWS < oledLineCount) {
    oledTopLine++;
    renderOLED();
  }
}

// =========================
// OLED / WIFI / CAMERA INIT
// =========================
bool scanI2COnPins(int sda, int scl, uint8_t &foundAddr) {
  Serial.printf("Scanning SDA=%d SCL=%d\n", sda, scl);

  Wire.end();
  delay(50);
  Wire.begin(sda, scl);
  Wire.setClock(100000);
  delay(300);

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.printf("Found I2C device at 0x%02X on SDA=%d SCL=%d\n", address, sda, scl);
      foundAddr = address;
      return true;
    }
  }

  Serial.println("Nothing found");
  return false;
}

bool initOLED() {
  struct I2CPins {
    int sda;
    int scl;
  };

  I2CPins pinTests[] = {
    {OLED_SDA, OLED_SCL},
    {6, 7},
    {5, 6},
    {8, 9},
    {4, 5},
    {7, 6}
  };

  uint8_t addr = 0;

  Serial.println("OLED: scanning possible I2C pin pairs");

  for (int i = 0; i < sizeof(pinTests) / sizeof(pinTests[0]); i++) {
    if (scanI2COnPins(pinTests[i].sda, pinTests[i].scl, addr)) {
      detectedOLEDAddr = addr;

      Serial.printf("OLED: using SDA=%d SCL=%d ADDR=0x%02X\n",
                    pinTests[i].sda,
                    pinTests[i].scl,
                    detectedOLEDAddr);

      if (!display.begin(SSD1306_SWITCHCAPVCC, detectedOLEDAddr)) {
        Serial.println("OLED: display.begin failed");
        return false;
      }

      Serial.println("OLED: success");
      display.clearDisplay();
      display.display();
      showOLEDText("OLED OK");
      return true;
    }
  }

  Serial.println("OLED: no I2C devices found on tested pins");
  return false;
}

String readSerialLineBlocking(const String& question) {
  Serial.println(question);
  showOLEDText(question);

  String input = "";
  unsigned long lastRefresh = 0;

  while (true) {
    if (millis() - lastRefresh > SETTINGS_WAIT_REFRESH_MS) {
      showOLEDText(question);
      lastRefresh = millis();
    }

    while (Serial.available()) {
      char c = (char)Serial.read();

      if (c == '\n' || c == '\r') {
        input.trim();
        if (input.length() > 0) {
          Serial.println("Saved.");
          showOLEDText("Saved.");
          return input;
        }
      } else {
        input += c;
      }
    }

    delay(20);
  }
}


void loadRuntimePreferences() {
  batterySaver = prefs.getBool("battery_saver", true);
  oledAutoOff = prefs.getBool("oled_auto_off", batterySaver);

  uint8_t savedPhotoMode = prefs.getUChar("photo_mode", (uint8_t)PHOTO_MEDIUM);
  if (savedPhotoMode > (uint8_t)PHOTO_LOW) {
    savedPhotoMode = (uint8_t)PHOTO_MEDIUM;
  }

  photoMode = (PhotoMode)savedPhotoMode;

  Serial.print("Loaded power mode: ");
  Serial.println(batterySaver ? "saver" : "normal");

  Serial.print("Loaded photo mode: ");
  Serial.println(photoModeName());
}

void savePowerPreference() {
  prefs.putBool("battery_saver", batterySaver);
  prefs.putBool("oled_auto_off", oledAutoOff);
}

void savePhotoPreference() {
  prefs.putUChar("photo_mode", (uint8_t)photoMode);
}

void loadSettings() {
  prefs.begin("settings", false);

  wifiSSID = prefs.getString("ssid", "");
  wifiPASS = prefs.getString("pass", "");
  openAIKey = prefs.getString("api_key", "");

  if (wifiSSID.length() == 0) {
    wifiSSID = readSerialLineBlocking("Enter WiFi name:");
    prefs.putString("ssid", wifiSSID);
  }

  if (wifiPASS.length() == 0) {
    wifiPASS = readSerialLineBlocking("Enter WiFi password:");
    prefs.putString("pass", wifiPASS);
  }

  if (openAIKey.length() == 0) {
    openAIKey = readSerialLineBlocking("Enter OpenAI API key:");
    prefs.putString("api_key", openAIKey);
  }

  loadRuntimePreferences();
}

void printSettingsStatus() {
  Serial.println();
  Serial.println("Saved settings:");
  Serial.print("WiFi SSID: ");
  Serial.println(wifiSSID.length() ? wifiSSID : "(not set)");
  Serial.print("WiFi password: ");
  Serial.println(wifiPASS.length() ? "(saved)" : "(not set)");
  Serial.print("OpenAI API key: ");
  Serial.println(openAIKey.length() ? "(saved)" : "(not set)");
  Serial.print("Power mode: ");
  Serial.println(batterySaver ? "saver" : "normal");
  Serial.print("Photo mode: ");
  Serial.println(photoModeName());
  Serial.print("OLED auto-off: ");
  Serial.println(oledAutoOff ? "on" : "off");
  Serial.println();
}

bool connectWiFi() {
  setCpuForBurst();

  WiFi.mode(WIFI_STA);
  applyWiFiPowerPolicy();
  WiFi.begin(wifiSSID.c_str(), wifiPASS.c_str());

  Serial.print("Connecting WiFi");
  showOLEDText("Connecting WiFi...");

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 25000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi failed");
    Serial.print("WiFi status: ");
    Serial.println(WiFi.status());
    Serial.print("API key length: ");
    Serial.println(openAIKey.length());
    showOLEDText("WiFi failed");
    enterIdlePowerState();
    return false;
  }

  Serial.println("WiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("RSSI: ");
  Serial.println(WiFi.RSSI());

  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Syncing time");

  time_t now = time(nullptr);
  unsigned long timeStart = millis();
  while (now < 1700000000 && millis() - timeStart < 20000) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println();

  if (now >= 1700000000) {
    Serial.println("Time synced");
  } else {
    Serial.println("Time sync timed out; HTTPS may still work if TLS accepts it.");
  }

  lastNetworkUseMs = millis();
  showOLEDText("WiFi connected");
  delay(500);
  enterIdlePowerState();
  return true;
}

bool ensureWiFiConnected() {
  if (WiFi.status() == WL_CONNECTED) {
    lastNetworkUseMs = millis();
    return true;
  }

  Serial.println("WiFi is off/disconnected; reconnecting for API request.");
  return connectWiFi();
}

const char* photoModeName() {
  switch (photoMode) {
    case PHOTO_HIGH:   return "high";
    case PHOTO_MEDIUM: return "medium";
    case PHOTO_LOW:    return "low";
    default:           return "auto";
  }
}

int firstProfileIndexForMode() {
  switch (photoMode) {
    case PHOTO_HIGH:   return 0; // XGA
    case PHOTO_MEDIUM: return 0; // XGA
    case PHOTO_LOW:    return 1; // SVGA
    default:           return 0; // XGA
  }
}

void printCameraStatus() {
  sensor_t* s = esp_camera_sensor_get();
  Serial.println();
  Serial.println("Camera status:");
  Serial.printf("  PSRAM: %s\n", psramFound() ? "yes" : "no");
  Serial.printf("  Free heap: %u\n", ESP.getFreeHeap());
  Serial.printf("  Free PSRAM: %u\n", ESP.getFreePsram());
  Serial.printf("  Battery saver: %s\n", batterySaver ? "on" : "off");
  Serial.printf("  CPU MHz: %u\n", getCpuFrequencyMhz());
  Serial.printf("  Photo mode: %s\n", photoModeName());
  Serial.printf("  Max JPEG bytes for API: %u\n", (unsigned)maxJpegBytesForApi());
  if (s) {
    Serial.printf("  Sensor PID: 0x%04x\n", s->id.PID);
  }
  Serial.println();
}

void applyCameraSensorTuning(sensor_t* s) {
  if (!s) return;

  // Orientation for XIAO ESP32S3 Sense adapter. Change these if your image is flipped.
  s->set_vflip(s, 1);
  s->set_hmirror(s, 0);

  // OCR-friendly: keep colors neutral, preserve edges, let exposure/white balance adapt.
  s->set_brightness(s, 0);
  s->set_contrast(s, 1);
  s->set_saturation(s, 0);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 0);        // Auto white balance mode.
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 1);           // Better auto exposure behavior.
  s->set_ae_level(s, 0);
  s->set_gain_ctrl(s, 1);
  s->set_gainceiling(s, (gainceiling_t)3);
  s->set_bpc(s, 1);            // Bad pixel correction.
  s->set_wpc(s, 1);            // White pixel correction.
  s->set_raw_gma(s, 1);        // Gamma correction.
  s->set_lenc(s, 1);           // Lens correction.
  s->set_dcw(s, 1);            // Downsize/crop helper for non-native sizes.

  // These exist in current esp32-camera sensor_t. If your older camera library
  // does not compile, comment out the next two lines.
  s->set_sharpness(s, 1);
  s->set_denoise(s, 1);
}

bool initCamera() {
  camera_config_t config = {};

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = CAMERA_XCLK_HZ;
  config.pixel_format = PIXFORMAT_JPEG;

  // Initialize conservatively. One framebuffer uses less RAM and avoids continuous capture churn.
  config.frame_size = psramFound() ? FRAMESIZE_SVGA : FRAMESIZE_VGA;
  config.jpeg_quality = 14;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (!s) {
    Serial.println("Could not get camera sensor");
    return false;
  }

  Serial.printf("Camera PID: 0x%04x\n", s->id.PID);
  applyCameraSensorTuning(s);

  ov5640Detected = (s->id.PID == OV5640_PID);

  if (ov5640Detected) {
    Serial.println("OV5640 detected");

    ov5640.start(s);

    if (ov5640.focusInit() == 0) {
      Serial.println("Focus init OK");
    } else {
      Serial.println("Focus init FAILED");
    }

    if (batterySaver) {
      Serial.println("Battery saver: continuous autofocus skipped to reduce current.");
    } else {
      if (ov5640.autoFocusMode() == 0) {
        Serial.println("Continuous autofocus OK");
      } else {
        Serial.println("Continuous autofocus FAILED");
      }
    }

    delay(batterySaver ? 500 : 1200);  // Let focus/exposure settle before first shot.
  } else {
    Serial.println("Not OV5640, autofocus disabled");
  }

  printCameraStatus();
  Serial.println("Camera OK");
  return true;
}

camera_fb_t* captureBestFrame() {
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return nullptr;

  int startIndex = firstProfileIndexForMode();

  for (int i = startIndex; i < CAPTURE_PROFILE_COUNT; i++) {
    CaptureProfile p = captureProfiles[i];

    // Without PSRAM, skip the large modes that usually exhaust memory.
    if (!psramFound() && p.frameSize > FRAMESIZE_SVGA) {
      continue;
    }

    Serial.printf("Trying capture profile: %s\n", p.name);
    s->set_framesize(s, p.frameSize);
    s->set_quality(s, p.jpegQuality);
    delay(batterySaver ? 180 : 350);  // Let exposure/white balance adapt to the new frame size.

    // Throw away one stale frame. More flushes improve freshness but cost battery/time.
    int flushCount = batterySaver ? 1 : 2;
    for (int flush = 0; flush < flushCount; flush++) {
      camera_fb_t* old = esp_camera_fb_get();
      if (old) esp_camera_fb_return(old);
      delay(batterySaver ? 35 : 60);
    }

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Capture failed on this profile; trying smaller.");
      continue;
    }

    Serial.printf("Captured %s: %u bytes\n", p.name, (unsigned)fb->len);

    if (fb->format != PIXFORMAT_JPEG || fb->len == 0) {
      Serial.println("Bad frame; trying smaller.");
      esp_camera_fb_return(fb);
      continue;
    }

    if (fb->len > maxJpegBytesForApi()) {
      Serial.printf("Frame too large for stable API POST (%u > %u); trying smaller.\n",
                    (unsigned)fb->len,
                    (unsigned)maxJpegBytesForApi());
      esp_camera_fb_return(fb);
      continue;
    }

    return fb;
  }

  return nullptr;
}

// =========================
// OPENAI HELPERS
// =========================
String base64Encode(const uint8_t* data, size_t len) {
  size_t outLen = 4 * ((len + 2) / 3);
  unsigned char* out = (unsigned char*)ps_malloc(outLen + 1);
  if (!out) {
    Serial.println("Base64 allocation failed");
    return "";
  }

  size_t actualOutLen = 0;
  int ret = mbedtls_base64_encode(out, outLen + 1, &actualOutLen, data, len);
  if (ret != 0) {
    free(out);
    Serial.printf("Base64 encode failed: %d\n", ret);
    return "";
  }

  out[actualOutLen] = '\0';

  String result;
  result.reserve(actualOutLen + 1);
  result = (char*)out;
  free(out);
  return result;
}

String jsonEscape(const String& in) {
  String out;
  out.reserve(in.length() + 16);

  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if ((uint8_t)c < 0x20) {
          char buf[7];
          snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out += c;
        }
        break;
    }
  }

  return out;
}


String normalAsciiOnly(const String& in) {
  String out;
  out.reserve(in.length());

  for (size_t i = 0; i < in.length(); i++) {
    uint8_t c = (uint8_t)in[i];

    // Keep printable normal ASCII only.
    if (c >= 32 && c <= 126) {
      out += (char)c;
    } else if (c == '\n' || c == '\r' || c == '\t') {
      out += ' ';
    } else {
      // Replace UTF-8 / extended bytes with a plain space.
      // The prompt asks the model to convert symbols to ASCII words,
      // so this is only a safety fallback.
      out += ' ';
    }
  }

  // Collapse repeated spaces caused by stripped UTF-8 bytes.
  while (out.indexOf("  ") >= 0) {
    out.replace("  ", " ");
  }

  out.trim();
  return out;
}

struct AnalysisResult {
  bool ok = false;
  int questionNumber = 0;
  String answerChoice = "?";
  String answerText = "Unclear";
  String error = "";
};

String promptWithStructuredJsonInstructions(const String& prompt) {
  String p = prompt;
  p += "\n\nReturn only structured JSON. Do not use markdown. Do not include explanations.";
  p += "\nThe JSON fields must be: question_number, answer_choice, answer_text.";
  p += "\nquestion_number: integer question number visible in the image. Use 0 if no question number is visible.";
  p += "\nanswer_choice: selected multiple-choice letter only, such as A, B, C, D. Use ? if no choice is identifiable.";
  p += "\nanswer_text: only the full text of the selected answer choice. Do not include Complete or status text.";
  p += "\nUse only normal ASCII characters in every JSON string value. Allowed characters are ASCII 32 through 126 only.";
  p += "\nDo not use smart quotes, curly apostrophes, em dashes, en dashes, bullets, degree symbols, superscripts, subscripts, Unicode math symbols, or non-English punctuation.";
  p += "\nConvert visual symbols to plain ASCII text when needed, for example: pi, sqrt(), ^2, >=, <=, !=, degrees.";
  return p;
}

String trimToJsonObject(const String& text) {
  String t = text;
  t.trim();

  int start = t.indexOf('{');
  int end = t.lastIndexOf('}');
  if (start >= 0 && end > start) {
    return t.substring(start, end + 1);
  }

  return t;
}

String normalizeAnswerChoice(String choice) {
  choice.trim();
  choice.toUpperCase();

  if (choice.length() == 0) return "?";

  char c = choice.charAt(0);
  if (c >= 'A' && c <= 'Z') {
    String out = "";
    out += c;
    return out;
  }

  return "?";
}

bool parseAnalysisJson(const String& rawText, AnalysisResult& result) {
  String json = trimToJsonObject(rawText);

  DynamicJsonDocument doc(2048);
  DeserializationError err = deserializeJson(doc, json);
  if (err) {
    result.ok = false;
    result.error = String("Bad JSON from API: ") + err.c_str();
    return false;
  }

  if (!doc["question_number"].is<int>() ||
      !doc["answer_choice"].is<const char*>() ||
      !doc["answer_text"].is<const char*>()) {
    result.ok = false;
    result.error = "Bad JSON from API: missing field";
    return false;
  }

  result.questionNumber = doc["question_number"].as<int>();
  if (result.questionNumber < 0) result.questionNumber = 0;

  result.answerChoice = normalizeAnswerChoice(normalAsciiOnly(doc["answer_choice"].as<String>()));

  result.answerText = normalAsciiOnly(doc["answer_text"].as<String>());
  result.answerText.replace("\r", " ");
  result.answerText.replace("\n", " ");
  result.answerText.trim();
  if (result.answerText.length() == 0) result.answerText = "Unclear";

  result.ok = true;
  result.error = "";
  return true;
}

String answerDisplayText(const AnalysisResult& result) {
  String out = "Question ";
  if (result.questionNumber > 0) {
    out += String(result.questionNumber);
  } else {
    out += "?";
  }

  out += "\n";

  if (result.answerChoice.length() > 0 && result.answerChoice != "?") {
    out += result.answerChoice;
    out += ". ";
  }

  out += result.answerText;
  return out;
}

void printAnalysisResultToSerial(const AnalysisResult& result) {
  Serial.println();
  Serial.println("================ PARSED RESULT =================");
  Serial.print("question_number: ");
  Serial.println(result.questionNumber);
  Serial.print("answer_choice: ");
  Serial.println(result.answerChoice);
  Serial.print("answer_text: ");
  Serial.println(result.answerText);
  Serial.println("=================================================");
  Serial.println();
}

void displayAnalysisSequence(const AnalysisResult& result) {
  showOLEDText("Complete");
  delay(700);

  showOLEDText(answerDisplayText(result));
}

String buildOpenAIPayload(const String& prompt, const String& b64) {
  String payload;
  payload.reserve(b64.length() + prompt.length() + 1600);

  payload += "{\"model\":\"";
  payload += OPENAI_MODEL;
  payload += "\",\"input\":[{\"role\":\"user\",\"content\":[";
  payload += "{\"type\":\"input_text\",\"text\":\"";
  payload += jsonEscape(prompt);
  payload += "\"},";
  payload += "{\"type\":\"input_image\",\"image_url\":\"data:image/jpeg;base64,";
  payload += b64;
  payload += "\",\"detail\":\"original\"}";
  payload += "]}],";
  payload += "\"reasoning\":{\"effort\":\"medium\"},";
  payload += "\"max_output_tokens\":2000,";

  // Structured Outputs: force the assistant's visible text to be exactly one JSON object.
  payload += "\"text\":{\"format\":{\"type\":\"json_schema\",\"name\":\"mcq_image_analysis\",\"strict\":true,\"schema\":{";
  payload += "\"type\":\"object\",";
  payload += "\"properties\":{";
  payload += "\"question_number\":{\"type\":\"integer\"},";
  payload += "\"answer_choice\":{\"type\":\"string\"},";
  payload += "\"answer_text\":{\"type\":\"string\"}";
  payload += "},";
  payload += "\"required\":[\"question_number\",\"answer_choice\",\"answer_text\"],";
  payload += "\"additionalProperties\":false";
  payload += "}}}";

  payload += "}";

  return payload;
}
void printChatGPTReplyToSerial(const String& text) {
  Serial.println();
  Serial.println("================ CHATGPT RESPONSE ================");
  if (text.length() == 0) {
    Serial.println("(empty reply)");
  } else {
    Serial.println(text);
  }
  Serial.println("==================================================");
  Serial.println();
}

String extractOutputText(const String& response) {
  DynamicJsonDocument doc(65536);
  DeserializationError err = deserializeJson(doc, response);

  if (err) {
    Serial.print("JSON parse failed: ");
    Serial.println(err.c_str());
    if (rawApiDebug) {
      Serial.println("Full response that failed JSON parse:");
      Serial.println(response);
    }
    return "JSON parse failed";
  }

  if (doc["output_text"].is<const char*>()) {
    String txt = doc["output_text"].as<const char*>();
    if (txt.length() > 0) return txt;
  }

  if (doc["output"].is<JsonArray>()) {
    String combined = "";
    JsonArray output = doc["output"].as<JsonArray>();

    for (JsonObject item : output) {
      if (!item["content"].is<JsonArray>()) continue;

      JsonArray content = item["content"].as<JsonArray>();
      for (JsonObject part : content) {
        const char* type = part["type"] | "";

        if (strcmp(type, "output_text") == 0 || strcmp(type, "text") == 0) {
          if (part["text"].is<const char*>()) {
            String t = part["text"].as<const char*>();
            if (t.length() > 0) {
              if (combined.length() > 0) combined += "\n";
              combined += t;
            }
          } else if (part["text"].is<JsonObject>() && part["text"]["value"].is<const char*>()) {
            String t = part["text"]["value"].as<const char*>();
            if (t.length() > 0) {
              if (combined.length() > 0) combined += "\n";
              combined += t;
            }
          }
        }
      }
    }

    if (combined.length() > 0) return combined;
  }

  if (doc["error"].is<JsonObject>() && doc["error"]["message"].is<const char*>()) {
    return String("API error: ") + doc["error"]["message"].as<const char*>();
  }

  if (rawApiDebug) {
    Serial.println("No usable text found. Full JSON:");
    serializeJsonPretty(doc, Serial);
    Serial.println();
  }

  return "No text found in response";
}

String askOpenAIWithImage(camera_fb_t* fb, const String& prompt) {
  if (!fb || !fb->buf || fb->len == 0) {
    return "No camera frame";
  }

  if (!ensureWiFiConnected()) {
    return "WiFi not connected";
  }

  setCpuForBurst();
  applyWiFiPowerPolicy();

  WiFiClientSecure client;
  client.setInsecure();
  client.setHandshakeTimeout(30);

  HTTPClient https;
  https.setTimeout(batterySaver ? 90000 : 120000);
  https.setReuse(false);

  if (!https.begin(client, "https://api.openai.com/v1/responses")) {
    return "HTTPS begin failed";
  }

  https.addHeader("Content-Type", "application/json");
  https.addHeader("Authorization", String("Bearer ") + openAIKey);

  Serial.print("OpenAI model: ");
  Serial.println(OPENAI_MODEL);
  Serial.printf("Encoding JPEG: %u bytes\n", (unsigned)fb->len);

  int httpCode = -1;

  // Keep b64 and payload scoped so the big buffers are destroyed before
  // we allocate the response String.
  {
    String b64 = base64Encode(fb->buf, fb->len);
    if (b64.length() == 0) {
      https.end();
      return "Base64 encode failed";
    }

    String apiPrompt = promptWithStructuredJsonInstructions(prompt);
    String payload = buildOpenAIPayload(apiPrompt, b64);
    if (payload.length() < 1000) {
      https.end();
      return "Payload build failed / too small";
    }

    Serial.print("Payload bytes: ");
    Serial.println(payload.length());
    Serial.print("Free heap before POST: ");
    Serial.println(ESP.getFreeHeap());
    Serial.print("Free PSRAM before POST: ");
    Serial.println(ESP.getFreePsram());

    startChatLoadingAnimation();
    httpCode = https.POST(payload);
  }

  String response = https.getString();
  stopChatLoadingAnimation();
  https.end();

  lastNetworkUseMs = millis();
  enterIdlePowerState();

  Serial.printf("OpenAI HTTP code: %d\n", httpCode);

  if (rawApiDebug) {
    Serial.println("RAW API RESPONSE:");
    Serial.println(response);
  }

  if (httpCode <= 0) {
    return String("HTTP POST failed: ") + httpCode;
  }

  if (httpCode < 200 || httpCode >= 300) {
    String err = String("HTTP error ") + httpCode;
    if (response.length() > 0) {
      err += "\n";
      err += extractOutputText(response);
    }
    return err;
  }

  String parsed = normalAsciiOnly(extractOutputText(response));
  printChatGPTReplyToSerial(parsed);
  return parsed;
}

// =========================
// CAMERA + DISPLAY FLOW
// =========================
void captureAnalyzeAndDisplay(const String& prompt) {
  noteActivity();
  setCpuForBurst();

  // Keep the OLED simple during the whole user-facing wait:
  // from button press / snap command until the ChatGPT response is ready.
  startChatLoadingAnimation();

  Serial.println();
  Serial.println("Capturing image...");

  camera_fb_t* fb = captureBestFrame();
  if (!fb) {
    Serial.println("Camera capture failed on all profiles");
    stopChatLoadingAnimation();
    showOLEDText("Camera capture failed");
    enterIdlePowerState();
    return;
  }

  Serial.printf("Final JPEG bytes: %u\n", (unsigned)fb->len);

  String reply = askOpenAIWithImage(fb, prompt);
  esp_camera_fb_return(fb);
  stopChatLoadingAnimation();

  if (reply.length() == 0) {
    reply = "Empty reply";
  }

  // This guarantees API failures appear in Serial Monitor even if there was no normal response box.
  if (reply.startsWith("HTTP") || reply.startsWith("API error") || reply == "Empty reply") {
    printChatGPTReplyToSerial(reply);
    showOLEDText(reply);
    enterIdlePowerState();
    return;
  }

  AnalysisResult result;
  if (!parseAnalysisJson(reply, result)) {
    Serial.println(result.error);
    Serial.println("Raw assistant text was:");
    Serial.println(reply);
    showOLEDText(result.error);
    enterIdlePowerState();
    return;
  }

  printAnalysisResultToSerial(result);
  displayAnalysisSequence(result);
  enterIdlePowerState();
}

void printPowerStatus() {
  Serial.println();
  Serial.println("Power status:");
  Serial.printf("  Battery saver: %s\n", batterySaver ? "on" : "off");
  Serial.printf("  OLED auto-off: %s\n", oledAutoOff ? "on" : "off");
  Serial.printf("  OLED is: %s\n", oledIsOn ? "on" : "off");
  Serial.printf("  CPU MHz: %u\n", getCpuFrequencyMhz());
  Serial.printf("  WiFi status: %d\n", WiFi.status());
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("  RSSI: %d dBm\n", WiFi.RSSI());
  }
  Serial.printf("  Photo mode: %s\n", photoModeName());
  Serial.printf("  Max JPEG bytes: %u\n", (unsigned)maxJpegBytesForApi());
  Serial.println();
}

void setBatterySaverMode(bool enabled) {
  batterySaver = enabled;

  if (batterySaver) {
    if (photoMode == PHOTO_HIGH || photoMode == PHOTO_AUTO) {
      photoMode = PHOTO_MEDIUM;
    }
    oledAutoOff = true;
    applyWiFiPowerPolicy();
    enterIdlePowerState();
    showOLEDText("Power: saver");
    Serial.println("Power mode: saver");
  } else {
    oledWake();
    oledAutoOff = false;
    setCpuForBurst();
    applyWiFiPowerPolicy();
    showOLEDText("Power: normal");
    Serial.println("Power mode: normal");
    Serial.println("Normal mode keeps WiFi/OLED more awake and allows higher photo profiles.");
  }

  savePowerPreference();
  savePhotoPreference();

  Serial.println("Power/photo settings saved to flash.");
  printPowerStatus();
}

// =========================
// SERIAL COMMANDS
// =========================
void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  snap              -> take photo and ask current prompt");
  Serial.println("  ask <prompt>      -> set prompt and take photo");
  Serial.println("  prompt            -> show current prompt");
  Serial.println("  photo auto        -> best OCR image, fallback if too large");
  Serial.println("  photo high        -> force highest capture profiles first");
  Serial.println("  photo medium      -> start around XGA");
  Serial.println("  photo low         -> start around SVGA/VGA");
  Serial.println("                     (photo mode is saved through resets)");
  Serial.println("  power             -> print battery/power status");
  Serial.println("  power saver       -> lower current, default");
  Serial.println("  power normal      -> better speed/quality, more current");
  Serial.println("                     (power mode is saved through resets)");
  Serial.println("  cam               -> print camera/memory status");
  Serial.println("  raw on/off        -> show/hide raw API JSON");
  Serial.println("  up                -> scroll OLED up");
  Serial.println("  down              -> scroll OLED down");
  Serial.println("  show              -> redraw last response");
  Serial.println("  test              -> OLED text test");
  Serial.println("  help              -> show this help");
  Serial.println("  setssid           -> change only WiFi name");
  Serial.println("  setpass           -> change only WiFi password");
  Serial.println("  setkey            -> change only OpenAI API key");
  Serial.println("  settings          -> show saved settings status");
  Serial.println();
}

void handleSerialCommand(String cmd) {
  noteActivity();
  oledWake();

  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.equalsIgnoreCase("help")) {
    printHelp();
    return;
  }

  if (cmd.equalsIgnoreCase("snap")) {
    captureAnalyzeAndDisplay(currentPrompt);
    return;
  }

  if (cmd.startsWith("ask ")) {
    currentPrompt = cmd.substring(4);
    currentPrompt.trim();
    if (currentPrompt.length() == 0) {
      currentPrompt = "Describe this image briefly.";
    }
    Serial.print("Prompt set to: ");
    Serial.println(currentPrompt);
    captureAnalyzeAndDisplay(currentPrompt);
    return;
  }

  if (cmd.equalsIgnoreCase("prompt")) {
    Serial.print("Current prompt: ");
    Serial.println(currentPrompt);
    return;
  }

  if (cmd.equalsIgnoreCase("cam")) {
    printCameraStatus();
    return;
  }

  if (cmd.equalsIgnoreCase("raw on")) {
    rawApiDebug = true;
    Serial.println("Raw API JSON debug: ON");
    return;
  }

  if (cmd.equalsIgnoreCase("raw off")) {
    rawApiDebug = false;
    Serial.println("Raw API JSON debug: OFF");
    return;
  }

  if (cmd.equalsIgnoreCase("power")) {
    printPowerStatus();
    return;
  }

  if (cmd.equalsIgnoreCase("power saver")) {
    setBatterySaverMode(true);
    return;
  }

  if (cmd.equalsIgnoreCase("power normal")) {
    setBatterySaverMode(false);
    return;
  }

  if (cmd.startsWith("photo ")) {
    String mode = cmd.substring(6);
    mode.trim();
    mode.toLowerCase();

    if (mode == "auto") {
      photoMode = PHOTO_AUTO;
    } else if (mode == "high") {
      photoMode = PHOTO_HIGH;
    } else if (mode == "medium" || mode == "med") {
      photoMode = PHOTO_MEDIUM;
    } else if (mode == "low") {
      photoMode = PHOTO_LOW;
    } else {
      Serial.println("Use: photo auto, photo high, photo medium, or photo low");
      return;
    }

    savePhotoPreference();

    Serial.print("Photo mode set to: ");
    Serial.println(photoModeName());
    Serial.println("Photo mode saved to flash.");
    return;
  }

  if (cmd.equalsIgnoreCase("up")) {
    scrollUp();
    return;
  }

  if (cmd.equalsIgnoreCase("down")) {
    scrollDown();
    return;
  }

  if (cmd.equalsIgnoreCase("show")) {
    showOLEDText(lastResponse);
    return;
  }

  if (cmd.equalsIgnoreCase("test")) {
    showOLEDText("This is a test of scrolling text on the OLED display. Use up and down.");
    return;
  }

  if (cmd.equalsIgnoreCase("setssid")) {
  wifiSSID = readSerialLineBlocking("Enter new WiFi name:");
  prefs.putString("ssid", wifiSSID);
  Serial.println("WiFi name updated. Restart or run reset.");
  return;
  }

  if (cmd.equalsIgnoreCase("setpass")) {
    wifiPASS = readSerialLineBlocking("Enter new WiFi password:");
    prefs.putString("pass", wifiPASS);
    Serial.println("WiFi password updated. Restart or run reset.");
    return;
  }

  if (cmd.equalsIgnoreCase("setkey")) {
    openAIKey = readSerialLineBlocking("Enter new OpenAI API key:");
    prefs.putString("api_key", openAIKey);
    Serial.println("API key updated.");
    return;
  }

  if (cmd.equalsIgnoreCase("settings")) {
    printSettingsStatus();
    return;
  }

  Serial.println("Unknown command. Type help");
}

// =========================
// SETUP / LOOP
// =========================
void setup() {
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_SNAP, INPUT_PULLUP);
  Serial.begin(115200);
  delay(1500);
  Serial.println("Boot");
  noteActivity();
  setCpuForIdle();

  if (!initOLED()) {
  Serial.println("OLED failed - stopping here");
    while (true) {
      delay(1000);
    }
  }

  showOLEDText("Loading settings...");
  loadSettings();
  printSettingsStatus();

  if (!connectWiFi()) {
    Serial.println("WiFi not connected");
  }

  Serial.println("Init camera...");
  showOLEDText("Init camera...");

  if (!initCamera()) {
    Serial.println("Camera failed");
    showOLEDText("Camera failed");
    return;
  }

  showOLEDText("Camera OK");
  delay(1000);

  // After normal boot prompts, leave the OLED blank until the next action.
  clearOLEDScreen();
  enterIdlePowerState();
}

void loop() {
  static String input = "";

  static bool lastUp = HIGH;
  static bool lastDown = HIGH;
  static bool lastSnap = HIGH;

  bool upNow = digitalRead(BTN_UP);
  bool downNow = digitalRead(BTN_DOWN);
  bool snapNow = digitalRead(BTN_SNAP);

  if (lastUp == HIGH && upNow == LOW) {
    scrollUp();
    delay(180);
  }

  if (lastDown == HIGH && downNow == LOW) {
    scrollDown();
    delay(180);
  }

  if (lastSnap == HIGH && snapNow == LOW) {
    captureAnalyzeAndDisplay(currentPrompt);
    delay(300);
  }

  lastUp = upNow;
  lastDown = downNow;
  lastSnap = snapNow;

  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (input.length() > 0) {
        handleSerialCommand(input);
        input = "";
      }
    } else {
      input += c;
    }
  }

  oledMaybeSleep();
  wifiMaybeSleep();
}