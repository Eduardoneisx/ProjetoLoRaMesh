
#include "envio_sheets.h"
#include <Arduino.h>
#include <DHT.h>

// ===================== CONFIGURAÇÕES ========================
#define TTL_INICIAL     3
#define PIN_AUX         19
#define RX_LORA         26
#define TX_LORA         27

#define BEACON_INTERVAL_BASE  20000    // 90 sec(quando for pra valer)
#define SENSOR_INTERVAL_BASE  10000    // 60 sec por nó(quando for pra valer)
#define PAYLOAD_SIZE          5
#define ROUTETIMEOUT          (3 * BEACON_INTERVAL_BASE)

#define START_BYTE 0xAA

// =================== SENSORES =========================
#if MEU_ID != BASE_ID
#define DHTP 25
#define DHTT DHT22
DHT dht(DHTP,DHTT);
#define DUST_PIN 4

volatile unsigned long lowStart = 0;
volatile unsigned long lowPulseOccupancy = 0;

unsigned long sampleStart = 0;
#endif

// RSSI: limiar mínimo (dBm) pra aceitar atualização de rota
#define RSSI_MIN_LEARN        -110

// ===================== HASH CACHE DE DUPLICATAS ========================
#define HASH_SIZE    64
#define CACHE_TTL_MS 15000

struct HashSlot {
  uint8_t       src;
  uint8_t       msgId;
  bool          occupied;
  unsigned long seenAt;
};
HashSlot hashCache[HASH_SIZE];

uint8_t hashFunc(uint8_t src, uint8_t msgId) {
  return (src * 31 + msgId) & (HASH_SIZE - 1);
}

bool alreadySeen(uint8_t src, uint8_t msgId) {
  uint8_t idx = hashFunc(src, msgId);
  for (uint8_t i = 0; i < HASH_SIZE; i++) {
    uint8_t probe = (idx + i) & (HASH_SIZE - 1);
    if (!hashCache[probe].occupied) continue;
    if (millis() - hashCache[probe].seenAt > CACHE_TTL_MS) {
      hashCache[probe].occupied = false;
      continue;
    }
    if (hashCache[probe].src == src && hashCache[probe].msgId == msgId) return true;
  }
  return false;
}

void addToCache(uint8_t src, uint8_t msgId) {
  uint8_t idx = hashFunc(src, msgId);
  for (uint8_t i = 0; i < HASH_SIZE; i++) {
    uint8_t probe = (idx + i) & (HASH_SIZE - 1);
    if (!hashCache[probe].occupied ||
        millis() - hashCache[probe].seenAt > CACHE_TTL_MS) {
      hashCache[probe].src      = src;
      hashCache[probe].msgId    = msgId;
      hashCache[probe].seenAt   = millis();
      hashCache[probe].occupied = true;
      return;
    }
    if (hashCache[probe].src == src && hashCache[probe].msgId == msgId) {
      hashCache[probe].seenAt = millis();
      return;
    }
  }
  memset(hashCache, 0, sizeof(hashCache));
  hashCache[idx] = { src, msgId, true, millis() };
}

// ===================== ESTRUTURA DA MENSAGEM ========================
struct MeshMessage {
  uint8_t  startByte;
  uint8_t  msgId;
  uint8_t  src;
  uint8_t  ttl;
  uint8_t  srcHopsToBase;   // distância da ultima retransmissão (não do originador)
  uint8_t  type;
  uint8_t  payload[PAYLOAD_SIZE];
  uint8_t  checkSum;
} __attribute__((packed));

#define MSG_DATA    0
#define MSG_BEACON  1
// E220 com enableRSSI anexa 1 byte ao final de cada RX
#define MSG_TOTAL_RX_BYTES (sizeof(MeshMessage) + 1)

// ===================== CRC =====================
uint8_t calculaCrc(uint8_t* data, uint8_t length) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x07;
      else            crc <<= 1;
    }
  }
  return crc;
}

uint8_t calculaCrcMsg(MeshMessage &msg) {
  return calculaCrc((uint8_t*)&msg,
                    sizeof(msg) - sizeof(msg.checkSum));
}

// ===================== GLOBAIS ======================
uint8_t           nextMsgId      = 1;
unsigned long     nextBeaconTime = 0;
unsigned long     nextSensorTime = 0;
volatile uint8_t  hopsToBase     = 255;
volatile unsigned long confirmRoute = 0;

SemaphoreHandle_t xFwdMutex = NULL;  // protege fwdQueue

// ===================== PROTÓTIPOS ====================
uint8_t       calculaCrcMsg(MeshMessage &msg);
void          sendMessage(MeshMessage &msg);
void          handleData(MeshMessage &msg, int rssi_dbm);
void          processaSerial();
uint8_t       getNextMsgId();
unsigned long getJitteredInterval(unsigned long base);

#if MEU_ID != BASE_ID
void          enqueueFwd(MeshMessage &msg, unsigned long sendAt);
void          processFwdQueue();
void          handleBeacon(MeshMessage &msg, int rssi_dbm);
void          sendDataToBase();
void          checkRoute();
#endif

#if MEU_ID == BASE_ID
void          sendBeacon();
#endif
// ===================== FILA DE FWD =======================
#if MEU_ID != BASE_ID

#define FWD_QUEUE_SIZE 3 // tamanho da fila = 6

struct PendingFwd {
  MeshMessage   msg;
  unsigned long sendAt;
  bool          active;
};
PendingFwd fwdQueue[FWD_QUEUE_SIZE];

void enqueueFwd(MeshMessage &msg, unsigned long sendAt) {
  if (xSemaphoreTake(xFwdMutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
  for (uint8_t i = 0; i < FWD_QUEUE_SIZE; i++) {
    if (!fwdQueue[i].active) {
      fwdQueue[i] = { msg, sendAt, true };
      xSemaphoreGive(xFwdMutex);
      return;
    }
  }
  xSemaphoreGive(xFwdMutex);
  Serial.println("Fila de fwd cheia, descartando");
}

void processFwdQueue() {
  if (xSemaphoreTake(xFwdMutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
  for (uint8_t i = 0; i < FWD_QUEUE_SIZE; i++) {
    if (fwdQueue[i].active && (long)(millis() - fwdQueue[i].sendAt) >= 0) {
      MeshMessage toSend = fwdQueue[i].msg;
      fwdQueue[i].active = false;
      xSemaphoreGive(xFwdMutex);
      sendMessage(toSend);
      return; // uma por vez para não segurar o mutex durante TX
    }
  }
  xSemaphoreGive(xFwdMutex);
}
#endif
// ===================== FREERTOS TASKS =====================
void taskRX(void* pvParams) {
  for (;;) {
    processaSerial();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void taskPeriodic(void* pvParams) {
  for (;;) {
    #if MEU_ID == BASE_ID
      if ((long)(millis() - nextBeaconTime) >= 0) {
        sendBeacon();
        nextBeaconTime = millis() + getJitteredInterval(BEACON_INTERVAL_BASE);
      }
    #else
      if ((long)(millis() - nextSensorTime) >= 0) {
        sendDataToBase();
        nextSensorTime = millis() + getJitteredInterval(SENSOR_INTERVAL_BASE);
      }
      checkRoute();
      processFwdQueue();
    #endif
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RX_LORA, TX_LORA);
  pinMode(PIN_AUX, INPUT);
  
  #if MEU_ID != BASE_ID
  dht.begin();
  //Fururos outros sesores
  #endif
  
  #if MEU_ID == BASE_ID
    conectarWiFi();
    iniciarSheets();
  #endif
  
  randomSeed(analogRead(0) ^ MEU_ID ^ esp_random());
  nextMsgId = (uint8_t)random(1, 256);

  memset(hashCache, 0, sizeof(hashCache));

  Serial.printf("No %d iniciado. Base ID: %d\n", MEU_ID, BASE_ID);

  #if MEU_ID == BASE_ID
    hopsToBase = 0;
    Serial.println("Modo BASE ativado");
    nextBeaconTime = millis() + random(1000, 5000);
  #else
    xFwdMutex = xSemaphoreCreateMutex();
    if (xFwdMutex == NULL) {
      Serial.println("FATAL: sem memoria para mutex");
      while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    memset(fwdQueue, 0, sizeof(fwdQueue));
    nextSensorTime = millis() + random(1000, SENSOR_INTERVAL_BASE);
    Serial.printf("Primeiro envio em %lu ms\n", nextSensorTime - millis());
  #endif

  // Ambas as tasks no core 1 (app_cpu). Core 0 fica livre para o stack WiFi/BT.
  xTaskCreatePinnedToCore(taskRX,       "taskRX",       4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskPeriodic, "taskPeriodic", 4096, NULL, 1, NULL, 1);
}

// ==================== LOOP ======================
// Toda a lógica foi movida para taskRX e taskPeriodic.
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// =================== JITTER ====================
unsigned long getJitteredInterval(unsigned long base) {
  long jitter = (long)(base / 3);              // ±33%
  return base + random(-jitter, jitter);
}

// ===================== RECEPÇÃO COM RSSI =====================
void processaSerial() {
  
  while (Serial2.available() > 0) {
    if (Serial2.peek() == START_BYTE) break;
    Serial2.read();  // descarta byte que não é start
  }
  
  if (Serial2.available() < MSG_TOTAL_RX_BYTES) return;

  vTaskDelay(pdMS_TO_TICKS(2));
  
  //Serial.printf("[processaSerial] avail=%d, MSG_TOTAL=%d\n", avail, MSG_TOTAL_RX_BYTES);

  MeshMessage msg;
  Serial2.readBytes((uint8_t*)&msg, sizeof(MeshMessage));
  uint8_t rssiByte;
  Serial2.readBytes(&rssiByte, 1);
  int rssi_dbm = -(256 - (int)rssiByte);

  uint8_t crcCalc = calculaCrcMsg(msg);

  if (msg.checkSum != crcCalc) {
    Serial.println("  -> CRC FAIL");
    return;
  }
  if (alreadySeen(msg.src, msg.msgId)) {
    Serial.println("  -> JA VISTO, descartando");
    return;
  }
  addToCache(msg.src, msg.msgId);

  switch (msg.type) {
    case MSG_DATA:   handleData(msg, rssi_dbm);   break;
    case MSG_BEACON: 
            #if MEU_ID != BASE_ID
            handleBeacon(msg, rssi_dbm);
            #endif 
            break;
  }
}

// ====================== ENVIO ===================
// Com LBT habilitado no módulo, o channel sense é feito DENTRO do E220.
void sendMessage(MeshMessage &msg) {
  unsigned long start = millis();
  while (digitalRead(PIN_AUX) == LOW && (millis() - start < 1000)) vTaskDelay(pdMS_TO_TICKS(2));
  if (digitalRead(PIN_AUX) == LOW) {
    Serial.println("Modulo travado, descartando");
    return;
  }

  Serial2.write((uint8_t*)&msg, sizeof(MeshMessage));
  Serial2.flush();

  // espera o módulo absorver o buffer (AUX desce e sobe novamente).
  // Margem de 4s pra cobrir LBT máximo (2s) + airtime + processamento.
  vTaskDelay(pdMS_TO_TICKS(5));
  start = millis();
  while (digitalRead(PIN_AUX) == LOW && (millis() - start < 4000)) vTaskDelay(pdMS_TO_TICKS(2));
}

// ======================= HANDLERS ======================
void handleData(MeshMessage &msg, int rssi_dbm) {
  #if MEU_ID == BASE_ID //só base
    Serial.printf("Dado: src=%d TTL=%d rssi=%d temp=%d umid=%d\n",
                  msg.src, msg.ttl, rssi_dbm,
                  msg.payload[0], msg.payload[1]);

    enviarParaSheets(msg.src, msg.payload[0], msg.payload[1]);
    
  #else //só sensor
  // aprende rota só se o link for confiável
  if (rssi_dbm >= RSSI_MIN_LEARN) {
    uint8_t remoteHops = msg.srcHopsToBase;
    if (remoteHops + 1 < hopsToBase) {
      confirmRoute = millis(); // Condirma rota atual
      hopsToBase = remoteHops + 1;
      Serial.printf("Nova distancia: %d (via %d, RSSI=%d)\n",
                    hopsToBase, msg.src, rssi_dbm);
    }
    if(remoteHops + 1 == hopsToBase){
      confirmRoute = millis();
    }
  }

  if (msg.ttl == 0) return;

  // Gossip ponderado por RSSI:
  //  sinal forte    → vizinhos do origem provavelmente já receberam (baixa prob)
  //  sinal fraco    → somos talvez o último elo confiável (alta prob)
  uint8_t fwdProb;
  if (rssi_dbm > -70)                  fwdProb = 15;
  else if (rssi_dbm > RSSI_MIN_LEARN)  fwdProb = 30;
  else                                 fwdProb = 0;

  bool fwd = (msg.srcHopsToBase > hopsToBase) ||
             (msg.srcHopsToBase == hopsToBase && random(0, 100) < fwdProb);

  if (fwd) {
    msg.ttl--;
    msg.srcHopsToBase = hopsToBase;
    msg.checkSum = calculaCrcMsg(msg);
    enqueueFwd(msg, millis() + random(50, 300));
  } else {
    Serial.printf("Nao retransmitindo de %d (RSSI=%d, prob=%d)\n",
                  msg.src, rssi_dbm, fwdProb);
  }
  #endif
}

#if MEU_ID != BASE_ID
void handleBeacon(MeshMessage &msg, int rssi_dbm) {
  // Beacon NÃO propaga (TTL=1, só base envia). Só serve pra vizinhos diretos.
  // Nós distantes aprendem hopsToBase via pacotes de dados encaminhados.
  if (rssi_dbm >= RSSI_MIN_LEARN) {
    uint8_t remoteHops = msg.srcHopsToBase;
    if (remoteHops + 1 < hopsToBase) {
      hopsToBase = remoteHops + 1;
      confirmRoute = millis(); // Condirma rota atual
      Serial.printf("Nova distancia (beacon): %d (RSSI=%d)\n",
                    hopsToBase, rssi_dbm);
    }
    if (remoteHops + 1 == hopsToBase) {
    confirmRoute = millis(); 
    }
  }
}
#endif

// ======================= ENVIOS PERIÓDICOS ======================

#if MEU_ID == BASE_ID
void sendBeacon() {
  MeshMessage beacon;
  memset(&beacon, 0, sizeof(beacon));
  beacon.startByte     = START_BYTE;
  beacon.msgId         = getNextMsgId();
  beacon.src           = MEU_ID;
  beacon.ttl           = 1;
  beacon.srcHopsToBase = hopsToBase;
  beacon.type          = MSG_BEACON;
  beacon.checkSum      = calculaCrcMsg(beacon);
  sendMessage(beacon);
  Serial.printf("Beacon enviado (dist=%d)\n", hopsToBase);
}
#endif

#if MEU_ID != BASE_ID
void sendDataToBase() {
  if (hopsToBase == 255) {
    Serial.println("Sem rota, aguardando topologia convergir");
    return;
  }
  MeshMessage data;
  memset(&data, 0, sizeof(data));
  data.startByte     = START_BYTE;
  data.msgId         = getNextMsgId();
  data.src           = MEU_ID;
  data.ttl           = TTL_INICIAL;
  data.srcHopsToBase = hopsToBase;
  data.type          = MSG_DATA;
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(t) || isnan(h)) return;   // não transmite leitura inválida
  data.payload[0] = (uint8_t)t;
  data.payload[1] = (uint8_t)h;
  data.checkSum      = calculaCrcMsg(data);
  sendMessage(data);
  Serial.printf("Dado enviado (dist=%d): T=%d H=%d\n",
                hopsToBase, data.payload[0], data.payload[1]);
}
#endif

uint8_t getNextMsgId() { return nextMsgId++; }

#if MEU_ID != BASE_ID
void checkRoute(){
  if(hopsToBase == 255) return;

  if(millis() - confirmRoute  > ROUTETIMEOUT){
    hopsToBase = 255;
  }
}
#endif
