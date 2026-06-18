

#if MEU_ID == BASE_ID
#include "secrets.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

// se não criar esse prototipo quebra no arduinoIDE porque ele cria o dele antes da biblioteca
static bool enviarPost(WiFiClientSecure& client, const String& body, const char* op);

// =====================  CONFIG  =====================
static const char* WIFI_SSID     = WIFI_SSID_SECRET;
static const char* WIFI_PASSWORD = WIFI_PASS_SECRET;
static const char* SCRIPT_URL    = SCRIPT_URL_SECRET;
static const char* TOKEN         = TOKEN_SECRET;

static const uint32_t INTERVALO_ATUAL_MS    = 5UL  * 60UL * 1000UL;   // 5 min
static const uint32_t INTERVALO_MEDIAS_MS   = 60UL * 60UL * 1000UL;   // 60 min
static const uint32_t HTTPS_TIMEOUT_MS      = 10000;

static const uint8_t  MAX_NOS = 30;

// =====================  ACUMULADORES  =====================
struct Acumulador {
  uint16_t soma_temp;          // soma direta
  uint16_t soma_hum;
  uint16_t  count;              // amostras na janela horária

  uint8_t  ultimo_temp;        // último valor (pra Atual)
  uint8_t  ultimo_hum;
  uint32_t ultima_leitura_ms;  // 0 = nó nunca enviou
};

static Acumulador acumuladores[MAX_NOS] = {0};
static portMUX_TYPE acumMux = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t xHttpMutex = NULL;

// =====================  POST HELPER  =====================

static bool enviarPost(WiFiClientSecure& client, const String& body, const char* op) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("[Sheets/%s] WiFi off, pulando\n", op);
    return false;
  }
  
  if (xSemaphoreTake(xHttpMutex, pdMS_TO_TICKS(30000)) != pdTRUE){
    return false;
  }

  String url = String(SCRIPT_URL) + "?token=" + String(TOKEN);

  HTTPClient https;
  https.setTimeout(HTTPS_TIMEOUT_MS);
  https.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

  if (!https.begin(client, url)) {
    Serial.printf("[Sheets/%s] begin() falhou\n", op);
    return false;
  }
  https.addHeader("Content-Type", "application/json");

  int code = https.POST(body);
  bool ok = false;

  if (code == HTTP_CODE_OK) {
    String resp = https.getString();
    ok = (resp.indexOf("ok") >= 0);
    Serial.printf("[Sheets/%s] %s (HTTP %d) resp='%s'\n",
                  op, ok ? "OK" : "REJEITADO", code, resp.c_str());
  } else {
    Serial.printf("[Sheets/%s] HTTP %d\n", op, code);
  }
  https.end();
  xSemaphoreGive(xHttpMutex);
  return ok;
}

// =====================  TASK: ATUAL (5 min)  =====================
static void taskAtual(void* params) {
  vTaskDelay(pdMS_TO_TICKS(30 * 1000));  // espera 30s pra acumular algo no boot


  for (;;) {
    WiFiClientSecure client;
    client.setInsecure();

    Acumulador snapshot[MAX_NOS];
    portENTER_CRITICAL(&acumMux);
    memcpy(snapshot, acumuladores, sizeof(acumuladores));
    portEXIT_CRITICAL(&acumMux);

    String body = "{\"op\":\"atual\",\"nodes\":[";
    bool primeiro = true;
    body.reserve(512);

    for (int i = 0; i < MAX_NOS; i++) {
      if (snapshot[i].ultima_leitura_ms == 0) continue;
      if (millis() - snapshot[i].ultima_leitura_ms > INTERVALO_ATUAL_MS) {
        if (!primeiro) body += ",";
        primeiro = false;
        body += "{\"id\":" + String(i + 1) + ",\"off\":1}";
      }
      else{
        if (!primeiro) body += ",";
        primeiro = false;
        body += "{\"id\":"  + String(i + 1);
        body += ",\"t\":"   + String(snapshot[i].ultimo_temp);
        body += ",\"h\":"   + String(snapshot[i].ultimo_hum);
        body += "}";
      }
    }
    body += "]}";

    if (!primeiro) {
      enviarPost(client, body, "atual");
    } else {
      Serial.println("[Sheets/atual] sem dados ainda, pulando");
    }

    vTaskDelay(pdMS_TO_TICKS(INTERVALO_ATUAL_MS));
  }
}

// =====================  TASK: MEDIAS_H (60 min)  =====================
static void taskMediasH(void* params) {
  vTaskDelay(pdMS_TO_TICKS(INTERVALO_MEDIAS_MS));  // espera 1h antes do 1º envio

  for (;;) {
    WiFiClientSecure client;
    client.setInsecure();

    Acumulador snapshot[MAX_NOS];
    portENTER_CRITICAL(&acumMux);
    memcpy(snapshot, acumuladores, sizeof(acumuladores));
    // zera somas e count, mantém ultimo_* pro Atual continuar funcionando
    for (int i = 0; i < MAX_NOS; i++) {
      acumuladores[i].soma_temp = 0;
      acumuladores[i].soma_hum  = 0;
      acumuladores[i].count     = 0;
    }
    portEXIT_CRITICAL(&acumMux);

    String body = "{\"op\":\"medias_h\",\"nodes\":[";
    bool primeiro = true;

    for (int i = 0; i < MAX_NOS; i++) {
      if (snapshot[i].count == 0) continue;

      // Média inteira (truncamento natural de divisão de int)
      uint8_t media_t = snapshot[i].soma_temp / snapshot[i].count;
      uint8_t media_h = snapshot[i].soma_hum  / snapshot[i].count;

      if (!primeiro) body += ",";
      primeiro = false;
      body += "{\"id\":" + String(i + 1);
      body += ",\"t\":"  + String(media_t);
      body += ",\"h\":"  + String(media_h);
      body += ",\"n\":"  + String(snapshot[i].count);
      body += "}";
    }
    body += "]}";

    if (!primeiro) {
      enviarPost(client, body, "medias_h");
    } else {
      Serial.println("[Sheets/medias_h] hora sem dados, pulando");
    }

    vTaskDelay(pdMS_TO_TICKS(INTERVALO_MEDIAS_MS));
  }
}

// =====================  API PÚBLICA  =====================
void iniciarSheets() {
  xHttpMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(taskAtual,   "atual",  8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskMediasH, "medias", 8192, NULL, 1, NULL, 0);
  Serial.println("[Sheets] Tasks Atual (5min) e MediasH (60min) iniciadas");
}


// Só registra no acumulador local — não envia nada imediatamente.
void enviarParaSheets(uint8_t nodeId, float temp, float hum) {
  if (nodeId == 0 || nodeId > MAX_NOS) return;
  if (isnan(temp) || isnan(hum)) return;
  uint8_t idx = nodeId - 1;
  
  portENTER_CRITICAL(&acumMux);
  acumuladores[idx].soma_temp        += (uint16_t)temp;
  acumuladores[idx].soma_hum         += (uint16_t)hum;
  acumuladores[idx].count++;
  acumuladores[idx].ultimo_temp       = (uint8_t)temp;
  acumuladores[idx].ultimo_hum        = (uint8_t)hum;
  acumuladores[idx].ultima_leitura_ms = millis();
  portEXIT_CRITICAL(&acumMux);
}

// =====================  WIFI  =====================
void conectarWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.printf("Conectando WiFi: %s", WIFI_SSID);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000UL) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi OK. IP=%s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nWiFi falhou");
  }
}

#endif  // MEU_ID == BASE_ID