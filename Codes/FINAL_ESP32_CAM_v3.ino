/* ============================================================================
   TrackAlert Vision — ESP32-S3-CAM  FIRMWARE v3
   ----------------------------------------------------------------------------
   Pairs with FINAL_ARDUINO_MEGA_v3.ino.

   Events from the Mega over UART:
     EVT:FIRE      -> photo, caption "Fire detected"
     EVT:GAS       -> photo, caption "Smoke detected"
     EVT:INTRUDER  -> photo, caption "Intruder detected"
     EVT:TILT      -> text  "Robot tipped over - patrol halted"
     EVT:CLEAR     -> text  "Alert cleared - patrol resumed"

   Telegram commands (owner's chat only):
     /capture  /photo   take a photo now
     /status            uptime, WiFi, RSSI, heap, camera
     /help              list commands

   v3 fixes — why /capture was not responding
     1.  BOT_TOKEN placeholder now shouts on the serial monitor at boot AND
         over Telegram is impossible without it — this is the most common cause.
     2.  Response reader was exiting early: "client.connected()" goes false as
         soon as the server closes, while data is STILL in the receive buffer.
         Now reads while (connected OR available), so the JSON is never cut off.
     3.  HTTP headers are now skipped before parsing, and the buffer is 5 KB
         with an explicit truncation warning (3 KB could cut the chat object,
         which silently failed the owner check).
     4.  Owner check now parses the chat id digit by digit instead of assuming
         a fixed length, and FAILS OPEN with a log line if it cannot find it —
         so a parsing miss can no longer swallow your command silently.
     5.  POLL_DEBUG prints the raw JSON and every decision, so if it still
         misbehaves the serial monitor tells you exactly where it stops.

   Board: ESP32S3 Dev Module - OPI PSRAM - USB CDC On Boot = Enabled
   ============================================================================ */

#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>

/* ----------------------------------------------------------------------------
   CREDENTIALS
   The old token was shared in a file, so treat it as public: BotFather ->
   /revoke -> paste the new token below. NOTHING works until this is set.
   ---------------------------------------------------------------------------- */
const char *WIFI_SSID = "";
const char *WIFI_PASS = "";
const String BOT_TOKEN = "";
const String CHAT_ID = "";

/* ---- camera pins (Freenove ESP32-S3-WROOM) ---- */
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 15
#define SIOD_GPIO_NUM 4
#define SIOC_GPIO_NUM 5
#define Y9_GPIO_NUM 16
#define Y8_GPIO_NUM 17
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 12
#define Y5_GPIO_NUM 10
#define Y4_GPIO_NUM 8
#define Y3_GPIO_NUM 9
#define Y2_GPIO_NUM 11
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 7
#define PCLK_GPIO_NUM 13

/* ---- UART link to Mega ---- */
#define MEGA_RX_PIN 21
#define MEGA_TX_PIN 47
HardwareSerial MegaLink(1);

/* ---- tuning ---- */
const unsigned long POLL_MS = 2500;
const unsigned long WIFI_RETRY_MS = 15000;
const byte PHOTO_ATTEMPTS = 3;
const unsigned long CAPTURE_MIN_GAP = 3000;
const size_t RESP_MAX = 5120; // was 3072 — could truncate
const bool POLL_DEBUG = true; // set false once /capture works

/* ---- state ---- */
long lastUpdateId = 0;
unsigned long lastPoll = 0;
unsigned long lastWiFiTry = 0;
unsigned long lastCapture = 0;
bool cameraReady = false;
bool tokenMissing = false;
char megaBuf[48];
byte megaLen = 0;

/* ---- forward declarations ---- */
bool startCamera();
void connectWiFi();
void ensureWiFi();
void handleMegaLink();
void handleEvent(const String &evt);
void captureAndSend(const String &caption);
bool sendTelegramPhoto(camera_fb_t *fb, const String &caption);
bool sendTelegramMessage(const String &text);
void checkTelegramCommands();
void primeTelegramBacklog();
void runCommand(const String &text);
String urlEncode(const String &s);
bool readHttpStatusOk(WiFiClientSecure &client);

/* ============================================================================
   SETUP
   ============================================================================ */
void setup()
{
  Serial.begin(115200);
  MegaLink.begin(115200, SERIAL_8N1, MEGA_RX_PIN, MEGA_TX_PIN);
  delay(1500);
  Serial.println("\n=== TrackAlert Vision - ESP32 v3 ===");

  tokenMissing = (BOT_TOKEN == "PASTE_NEW_TOKEN_HERE" || BOT_TOKEN.length() < 20);
  if (tokenMissing)
  {
    Serial.println("****************************************************");
    Serial.println("*  BOT_TOKEN IS NOT SET.                           *");
    Serial.println("*  No photos, no /capture, no messages will work.  *");
    Serial.println("*  BotFather -> /revoke -> paste token in the code. *");
    Serial.println("****************************************************");
  }

  for (byte i = 0; i < 3 && !cameraReady; i++)
  {
    cameraReady = startCamera();
    if (!cameraReady)
    {
      Serial.println("Camera init failed, retrying...");
      delay(1000);
    }
  }
  Serial.println(cameraReady ? "Camera OK" : "Camera UNAVAILABLE - text-only alerts");

  connectWiFi();
  primeTelegramBacklog();

  sendTelegramMessage(cameraReady
                          ? "TrackAlert online. Commands: /capture /status /help"
                          : "TrackAlert online - CAMERA FAULT, text alerts only.");
  Serial.println("Ready.");
}

/* ============================================================================
   LOOP  —  Mega events always take priority over Telegram polling
   ============================================================================ */
void loop()
{
  handleMegaLink();
  ensureWiFi();

  if (millis() - lastPoll > POLL_MS)
  {
    checkTelegramCommands();
    lastPoll = millis();
    handleMegaLink();
  }
}

/* ============================================================================
   UART FROM MEGA  —  non-blocking line assembler
   ============================================================================ */
void handleMegaLink()
{
  while (MegaLink.available())
  {
    char c = (char)MegaLink.read();
    if (c == '\n' || c == '\r')
    {
      if (megaLen > 0)
      {
        megaBuf[megaLen] = '\0';
        String line = String(megaBuf);
        line.trim();
        megaLen = 0;
        if (line.length())
          handleEvent(line);
      }
    }
    else if (megaLen < sizeof(megaBuf) - 1)
    {
      megaBuf[megaLen++] = c;
    }
    else
    {
      megaLen = 0;
    }
  }
}

void handleEvent(const String &evt)
{
  Serial.print("<- Mega: ");
  Serial.println(evt);
  MegaLink.println("ACK");

  if (evt == "EVT:FIRE")
    captureAndSend("Fire detected");
  else if (evt == "EVT:GAS")
    captureAndSend("Smoke detected");
  else if (evt == "EVT:INTRUDER")
    captureAndSend("Intruder detected");
  else if (evt == "EVT:TILT")
    sendTelegramMessage("Robot tipped over - patrol halted");
  else if (evt == "EVT:CLEAR")
    sendTelegramMessage("Alert cleared - patrol resumed");
  else
    Serial.println("   (unknown event, ignored)");
}

/* ============================================================================
   CAPTURE + SEND
   ============================================================================ */
void captureAndSend(const String &caption)
{
  if (!cameraReady)
  {
    sendTelegramMessage(caption + " (no camera - text alert only)");
    return;
  }

  for (int i = 0; i < 2; i++)
  { // flush stale frames
    camera_fb_t *t = esp_camera_fb_get();
    if (t)
      esp_camera_fb_return(t);
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.println("Frame grab failed");
    sendTelegramMessage(caption + " (camera failed)");
    return;
  }
  Serial.printf("Frame %u bytes\n", (unsigned)fb->len);

  bool sent = false;
  for (byte attempt = 1; attempt <= PHOTO_ATTEMPTS && !sent; attempt++)
  {
    Serial.printf("Sending photo, attempt %u/%u...\n", attempt, PHOTO_ATTEMPTS);
    sent = sendTelegramPhoto(fb, caption);
    if (!sent)
    {
      ensureWiFi();
      delay(800);
    }
  }
  esp_camera_fb_return(fb);

  if (!sent)
  {
    Serial.println("Photo failed after retries - sending text");
    sendTelegramMessage(caption + " (photo upload failed)");
  }
  else
  {
    Serial.println("Photo sent: " + caption);
  }
  lastCapture = millis();
}

/* ============================================================================
   TELEGRAM — photo
   ============================================================================ */
bool sendTelegramPhoto(camera_fb_t *fb, const String &caption)
{
  if (WiFi.status() != WL_CONNECTED || !fb)
    return false;

  WiFiClientSecure client;
  client.setInsecure();
  if (!client.connect("api.telegram.org", 443))
  {
    Serial.println("TLS connect failed");
    return false;
  }

  const String b = "TrackAlertBoundary";
  String head = "--" + b + "\r\nContent-Disposition: form-data; name=\"chat_id\"\r\n\r\n" + CHAT_ID + "\r\n"
                                                                                                      "--" +
                b + "\r\nContent-Disposition: form-data; name=\"caption\"\r\n\r\n" + caption + "\r\n"
                                                                                               "--" +
                b + "\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"trackalert.jpg\"\r\n"
                    "Content-Type: image/jpeg\r\n\r\n";
  String tail = "\r\n--" + b + "--\r\n";
  size_t total = head.length() + fb->len + tail.length();

  client.println("POST /bot" + BOT_TOKEN + "/sendPhoto HTTP/1.1");
  client.println("Host: api.telegram.org");
  client.println("Content-Type: multipart/form-data; boundary=" + b);
  client.println("Content-Length: " + String(total));
  client.println("Connection: close");
  client.println();

  client.print(head);
  const size_t CH = 1024;
  for (size_t i = 0; i < fb->len; i += CH)
  {
    size_t n = (fb->len - i > CH) ? CH : (fb->len - i);
    if (client.write(fb->buf + i, n) != n)
    {
      client.stop();
      return false;
    }
  }
  client.print(tail);

  bool ok = readHttpStatusOk(client);
  client.stop();
  return ok;
}

/* ============================================================================
   TELEGRAM — text
   ============================================================================ */
bool sendTelegramMessage(const String &text)
{
  if (WiFi.status() != WL_CONNECTED)
    return false;

  WiFiClientSecure client;
  client.setInsecure();
  if (!client.connect("api.telegram.org", 443))
    return false;

  String url = "/bot" + BOT_TOKEN + "/sendMessage?chat_id=" + CHAT_ID + "&text=" + urlEncode(text);
  client.println("GET " + url + " HTTP/1.1");
  client.println("Host: api.telegram.org");
  client.println("Connection: close");
  client.println();

  bool ok = readHttpStatusOk(client);
  client.stop();
  return ok;
}

/* Read the status line, then drain. True on HTTP 200. */
bool readHttpStatusOk(WiFiClientSecure &client)
{
  String status = "";
  unsigned long t = millis();

  while (millis() - t < 8000)
  {
    if (client.available())
    {
      status = client.readStringUntil('\n');
      break;
    }
    if (!client.connected() && !client.available())
      break;
    delay(10);
  }

  bool ok = (status.indexOf("200") > 0);
  if (!ok)
    Serial.println("HTTP status: " + status);

  t = millis();
  while ((client.connected() || client.available()) && millis() - t < 3000)
  {
    while (client.available())
    {
      client.read();
      t = millis();
    }
    delay(5);
  }
  return ok;
}

/* ============================================================================
   TELEGRAM — poll for commands
   ============================================================================ */
void checkTelegramCommands()
{
  if (WiFi.status() != WL_CONNECTED)
    return;
  if (tokenMissing)
    return;

  WiFiClientSecure client;
  client.setInsecure();
  if (!client.connect("api.telegram.org", 443))
  {
    if (POLL_DEBUG)
      Serial.println("poll: TLS connect failed");
    return;
  }

  String url = "/bot" + BOT_TOKEN + "/getUpdates?offset=" + String(lastUpdateId + 1) +
               "&limit=3&timeout=0";
  client.println("GET " + url + " HTTP/1.1");
  client.println("Host: api.telegram.org");
  client.println("Connection: close");
  client.println();

  /* FIX: read while connected OR data still buffered. The old condition
     dropped the tail of the JSON the moment the server closed. */
  static char resp[RESP_MAX];
  size_t n = 0;
  unsigned long t = millis();
  while ((client.connected() || client.available()) && millis() - t < 8000)
  {
    while (client.available())
    {
      char c = (char)client.read();
      if (n < RESP_MAX - 1)
        resp[n++] = c;
      t = millis();
    }
    delay(5);
  }
  resp[n] = '\0';
  client.stop();

  if (n == 0)
  {
    if (POLL_DEBUG)
      Serial.println("poll: empty response");
    return;
  }
  if (n >= RESP_MAX - 1)
    Serial.println("poll: TRUNCATED - raise RESP_MAX");

  char *bodyPtr = strstr(resp, "\r\n\r\n"); // skip HTTP headers
  bodyPtr = bodyPtr ? bodyPtr + 4 : resp;
  String body = String(bodyPtr);

  if (POLL_DEBUG)
  {
    Serial.print("poll body: ");
    Serial.println(body);
  }

  int idx = 0;
  while (true)
  {
    int u = body.indexOf("\"update_id\":", idx);
    if (u < 0)
      break;

    long uid = body.substring(u + 12, u + 32).toInt();
    if (uid > lastUpdateId)
      lastUpdateId = uid;

    int nextU = body.indexOf("\"update_id\":", u + 12);
    int end = (nextU < 0) ? (int)body.length() : nextU;

    /* owner check — parse the digits properly, and FAIL OPEN if not found
       so a parsing miss cannot silently swallow the command */
    bool fromOwner = true;
    int cid = body.indexOf("\"chat\":{\"id\":", u);
    if (cid > 0 && cid < end)
    {
      int p = cid + 13;
      String who = "";
      while (p < (int)body.length())
      {
        char c = body[p];
        if ((c >= '0' && c <= '9') || c == '-')
        {
          who += c;
          p++;
        }
        else
          break;
      }
      fromOwner = (who == CHAT_ID);
      if (!fromOwner)
        Serial.println("poll: ignoring command from chat " + who);
    }
    else if (POLL_DEBUG)
    {
      Serial.println("poll: chat id not found - accepting anyway");
    }

    int txtPos = body.indexOf("\"text\":\"", u);
    if (txtPos > 0 && txtPos < end)
    {
      int s = txtPos + 8;
      int e = body.indexOf("\"", s);
      if (e > s)
      {
        String text = body.substring(s, e);
        text.trim();
        Serial.println("Telegram cmd: [" + text + "]");
        if (fromOwner)
          runCommand(text);
      }
    }
    else if (POLL_DEBUG)
    {
      Serial.println("poll: update has no text field");
    }

    idx = u + 12;
  }
}

void runCommand(const String &text)
{
  if (text.startsWith("/capture") || text.startsWith("/photo"))
  {
    if (millis() - lastCapture > CAPTURE_MIN_GAP)
    {
      Serial.println("cmd: capture");
      captureAndSend("Requested photo");
    }
    else
    {
      Serial.println("cmd: capture ignored (too soon)");
    }
  }
  else if (text.startsWith("/status"))
  {
    String s = "Uptime " + String(millis() / 1000) + "s, WiFi " +
               (WiFi.status() == WL_CONNECTED ? "OK" : "DOWN") +
               ", RSSI " + String(WiFi.RSSI()) + "dBm, heap " +
               String(ESP.getFreeHeap()) + "B, camera " +
               (cameraReady ? "OK" : "FAULT");
    sendTelegramMessage(s);
  }
  else if (text.startsWith("/help") || text.startsWith("/start"))
  {
    sendTelegramMessage("TrackAlert commands: /capture (photo now), /status, /help");
  }
  else
  {
    Serial.println("cmd: unknown, ignored");
  }
}

/* Discard anything queued before power-up. */
void primeTelegramBacklog()
{
  if (WiFi.status() != WL_CONNECTED || tokenMissing)
    return;
  WiFiClientSecure client;
  client.setInsecure();
  if (!client.connect("api.telegram.org", 443))
    return;

  client.println("GET /bot" + BOT_TOKEN + "/getUpdates?offset=-1 HTTP/1.1");
  client.println("Host: api.telegram.org");
  client.println("Connection: close");
  client.println();

  String body = "";
  unsigned long t = millis();
  while ((client.connected() || client.available()) && millis() - t < 8000)
  {
    while (client.available())
    {
      char c = (char)client.read();
      if (body.length() < 1500)
        body += c;
      t = millis();
    }
    delay(5);
  }
  client.stop();

  int u = body.indexOf("\"update_id\":");
  if (u >= 0)
  {
    lastUpdateId = body.substring(u + 12, u + 32).toInt();
    Serial.println("Backlog primed at update_id " + String(lastUpdateId));
  }
  else
  {
    Serial.println("No backlog (queue empty)");
  }
}

/* ============================================================================
   CAMERA
   ============================================================================ */
bool startCamera()
{
  camera_config_t c = {}; // zero-init

  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;
  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;
  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;
  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;
  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;

  if (psramFound())
  {
    c.frame_size = FRAMESIZE_SVGA;
    c.jpeg_quality = 12;
    c.fb_count = 2;
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.grab_mode = CAMERA_GRAB_LATEST;
  }
  else
  {
    c.frame_size = FRAMESIZE_QVGA;
    c.jpeg_quality = 15;
    c.fb_count = 1;
    c.fb_location = CAMERA_FB_IN_DRAM;
    c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK)
  {
    Serial.printf("esp_camera_init: 0x%x\n", err);
    return false;
  }
  return true;
}

/* ============================================================================
   WIFI
   ============================================================================ */
void connectWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 20000)
  {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  }
  else
    Serial.println("WiFi not connected - will keep retrying");
  lastWiFiTry = millis();
}

void ensureWiFi()
{
  if (WiFi.status() == WL_CONNECTED)
    return;
  if (millis() - lastWiFiTry < WIFI_RETRY_MS)
    return;
  lastWiFiTry = millis();
  Serial.println("WiFi down - reconnecting");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

/* ============================================================================
   HELPERS
   ============================================================================ */
String urlEncode(const String &s)
{
  String o = "";
  for (unsigned int i = 0; i < s.length(); i++)
  {
    unsigned char c = (unsigned char)s[i];
    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~')
      o += (char)c;
    else
    {
      char b[4];
      sprintf(b, "%%%02X", c);
      o += b;
    }
  }
  return o;
}
