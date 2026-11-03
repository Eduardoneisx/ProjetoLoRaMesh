
#include "envio_sheets.h"
#include <Arduino.h>
#include <DHT.h>

// ===================== CONFIGURAÇÕES ========================
#define TTL_INICIAL     3
#define PIN_AUX         19
#define RX_LORA         16
#define TX_LORA         17

#define BEACON_INTERVAL_BASE  90000    // 90 sec(quando for pra valer)
#define SENSOR_INTERVAL_BASE  60000    // 60 sec por nó(quando for pra valer)
#define PAYLOAD_SIZE          5
#define ROUTETIMEOUT          (3 * BEACON_INTERVAL_BASE)

#define START_BYTE 0xAA

// =================== SENSORES =========================
#if MEU_ID != BASE_ID
#define DHTP 25
#define DHTT DHT22
static DHT dht(DHTP,DHTT);

#endif

// RSSI: limiar mínimo (dBm) pra aceitar atualização de rota
#define RSSI_MIN_LEARN        -105

// ===================== HASH CACHE DE DUPLICATAS ========================

// ===================== CACHE DE DUPLICACAO =====================
// Tabela hash de endereçamento aberto com sondagem linear.
// Tres estados por slot:
//   EMPTY  slot nunca utilizado -> encerra a cadeia de sondagem
//   USED   entrada valida (pode estar expirada, e entao vira DEAD)
//   DEAD   lapide: slot ja utilizado e liberado, nao encerra a cadeia
// Chamada apenas pela taskRX, portanto sem necessidade de mutex.

class HashCache {
public:
  bool alreadySeen(uint8_t src, uint8_t msgId) {
    uint8_t idx = hashFunc(src, msgId);
    for (uint8_t i = 0; i < HASH_SIZE; i++) {
      Slot &s = slots[(idx + i) & (HASH_SIZE - 1)];

      if (s.state == EMPTY) return false;          // fim da cadeia: nao esta na tabela

      if (s.state == USED) {
        if (expired(s)) {
          s.state = DEAD;                          // vira lapide, nao EMPTY
        } else if (s.src == src && s.msgId == msgId) {
          return true;
        }
      }
      // DEAD: segue sondando
    }
    return false;
  }

  void add(uint8_t src, uint8_t msgId) {
    if (nonEmpty > (HASH_SIZE * 3) / 4) compact();  // garante ao menos um EMPTY

    uint8_t idx  = hashFunc(src, msgId);
    uint8_t dest = HASH_SIZE;                       // primeiro slot reaproveitavel

    for (uint8_t i = 0; i < HASH_SIZE; i++) {
      uint8_t probe = (idx + i) & (HASH_SIZE - 1);
      Slot &s = slots[probe];

      if (s.state == USED && expired(s)) s.state = DEAD;

      if (s.state == DEAD) {
        if (dest == HASH_SIZE) dest = probe;
        continue;
      }
      if (s.state == EMPTY) {
        if (dest == HASH_SIZE) { dest = probe; nonEmpty++; }
        break;
      }
      if (s.src == src && s.msgId == msgId) {       // ja presente: renova o TTL
        s.seenAt = millis();
        return;
      }
    }
    slots[dest] = { src, msgId, USED, millis() };
  }

private:
  static constexpr unsigned long CACHE_TTL_MS = 15000;
  static constexpr uint8_t       HASH_SIZE    = 64;
  static_assert((HASH_SIZE & (HASH_SIZE - 1)) == 0,
                "HASH_SIZE precisa ser potencia de 2");

  enum State : uint8_t { EMPTY = 0, USED = 1, DEAD = 2 };

  struct Slot {
    uint8_t       src;
    uint8_t       msgId;
    State         state;
    unsigned long seenAt;
  };

  Slot    slots[HASH_SIZE] = {};   // EMPTY = 0
  uint8_t nonEmpty         = 0;    // USED + DEAD

  static bool expired(const Slot &s) {
    return millis() - s.seenAt > CACHE_TTL_MS;
  }

  uint8_t hashFunc(uint8_t src, uint8_t msgId) const {
    return (src * 31 + msgId) & (HASH_SIZE - 1);
  }

  // Reconstroi a tabela apenas com entradas vivas, eliminando as lapides.
  // Usa ~512 B de pilha; a taskRX precisa ter folga para isso.
  void compact() {
    Slot    live[HASH_SIZE];
    uint8_t n = 0;

    for (uint8_t i = 0; i < HASH_SIZE; i++)
      if (slots[i].state == USED && !expired(slots[i])) live[n++] = slots[i];

    memset(slots, 0, sizeof(slots));   // tudo volta a EMPTY
    nonEmpty = 0;

    for (uint8_t k = 0; k < n; k++) {
      uint8_t idx = hashFunc(live[k].src, live[k].msgId);
      for (uint8_t i = 0; i < HASH_SIZE; i++) {
        uint8_t probe = (idx + i) & (HASH_SIZE - 1);
        if (slots[probe].state == EMPTY) {
          slots[probe] = live[k];
          nonEmpty++;
          break;
        }
      }
    }
  }
};

static HashCache hashCache;

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
static uint8_t calculaCrc(uint8_t* data, uint8_t length) {
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

static uint8_t calculaCrcMsg(MeshMessage &msg) {
  return calculaCrc((uint8_t*)&msg,
                    sizeof(msg) - sizeof(msg.checkSum));
}

// ===================== GLOBAIS ======================
static uint8_t       nextMsgId      = 1;
static unsigned long nextBeaconTime = 0;
static unsigned long nextSensorTime = 0;

// ===================== PROTÓTIPOS ====================
static uint8_t       calculaCrcMsg(MeshMessage &msg);
static void          sendMessage(MeshMessage &msg);
static void          handleData(MeshMessage &msg, int rssi_dbm);
static void          processaSerial();
static uint8_t       getNextMsgId();
static unsigned long getJitteredInterval(unsigned long base);

#if MEU_ID != BASE_ID
static void          handleBeacon(MeshMessage &msg, int rssi_dbm);
static void          sendDataToBase();
#endif

#if MEU_ID == BASE_ID
static void          sendBeacon();
#endif
// ===================== ESTADO DE ROTA =======================
#if MEU_ID != BASE_ID

class RouteState {
public:

  bool init() {
    mutex = xSemaphoreCreateMutex();
    return mutex != NULL;
  }

  uint8_t hops() { return hopsToBase_; }

  bool hasRoute() { return hops() != NO_ROUTE; }

  // Atualiza hops/confirm atomicamente. Retorna true se achou rota mais curta.
  bool learn(uint8_t remoteHops) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;
    unsigned long now = millis();
    uint8_t newHops = remoteHops + 1;
    bool improved = false;

    if (newHops < hopsToBase_) {
      hopsToBase_ = newHops;
      confirmAt_ = now;
      improved = true;
    } else if (newHops == hopsToBase_) {
      confirmAt_ = now;
    }

    xSemaphoreGive(mutex);
    return improved;
  }

  void checkTimeout(unsigned long timeoutMs) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
    if (hopsToBase_ != NO_ROUTE && millis() - confirmAt_ > timeoutMs) {
      hopsToBase_ = NO_ROUTE;
    }
    xSemaphoreGive(mutex);
  }

private:
  static constexpr uint8_t NO_ROUTE = 255;
  uint8_t           hopsToBase_ = NO_ROUTE;
  unsigned long     confirmAt_ = 0;
  SemaphoreHandle_t mutex = NULL;
};

static RouteState routeState;

// ===================== FILA DE FWD =======================

class FwdQueue {
public:
  bool init() {
    mutex = xSemaphoreCreateMutex();
    return mutex != NULL;
  }

  void enqueue(MeshMessage &msg, unsigned long sendAt) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
    for (uint8_t i = 0; i < FWD_QUEUE_SIZE; i++) {
      if (!queue[i].active) {
        queue[i] = { msg, sendAt, true };
        xSemaphoreGive(mutex);
        return;
      }
    }
    xSemaphoreGive(mutex);
    Serial.println("Fila de fwd cheia, descartando");
  }

  void process() {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
    for (uint8_t i = 0; i < FWD_QUEUE_SIZE; i++) {
      if (queue[i].active && (long)(millis() - queue[i].sendAt) >= 0) {
        MeshMessage toSend = queue[i].msg;
        queue[i].active = false;
        xSemaphoreGive(mutex);
        sendMessage(toSend);
        return; // uma por vez para não segurar o mutex durante TX
      }
    }
    xSemaphoreGive(mutex);
  }

private:
  static constexpr uint8_t FWD_QUEUE_SIZE = 3;
  struct Entry {
    MeshMessage   msg;
    unsigned long sendAt;
    bool          active;
  };
  Entry             queue[FWD_QUEUE_SIZE];
  SemaphoreHandle_t mutex = NULL;
};

static FwdQueue fwdQueue;
#endif
// ===================== FREERTOS TASKS =====================
static void taskRX(void* pvParams) {
  for (;;) {
    processaSerial();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

static void taskPeriodic(void* pvParams) {
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
      routeState.checkTimeout(ROUTETIMEOUT);
      fwdQueue.process();
    #endif
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RX_LORA, TX_LORA);
  pinMode(PIN_AUX, INPUT_PULLUP);
  
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

  Serial.printf("No %d iniciado. Base ID: %d\n", MEU_ID, BASE_ID);

  #if MEU_ID == BASE_ID
    Serial.println("Modo BASE ativado");
    nextBeaconTime = millis() + random(1000, 5000);
  #else
    if (!routeState.init() || !fwdQueue.init()) {
      Serial.println("FATAL: sem memoria para mutex");
      while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    nextSensorTime = millis() + random(1000, SENSOR_INTERVAL_BASE);
    Serial.printf("Primeiro envio em %lu ms\n", nextSensorTime - millis());
  #endif

  // Ambas as tasks no core 1 (app_cpu). Core 0 fica livre para o stack WiFi/BT.
  xTaskCreatePinnedToCore(taskRX,       "taskRX",       4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskPeriodic, "taskPeriodic", 4096, NULL, 1, NULL, 1);
}

// ==================== LOOP ======================
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// =================== JITTER ====================
static unsigned long getJitteredInterval(unsigned long base) {
  long jitter = (long)(base / 3);              // ±33%
  return base + random(-jitter, jitter);
}

// ===================== RECEPÇÃO COM RSSI =====================
static void processaSerial() {
  
  while (Serial2.available() > 0) {
    if (Serial2.peek() == START_BYTE) break;
    Serial2.read();  // descarta byte que não é start
  }
  
  if (Serial2.available() < MSG_TOTAL_RX_BYTES) return;

  vTaskDelay(pdMS_TO_TICKS(2));
  

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
  if (hashCache.alreadySeen(msg.src, msg.msgId)) {
    Serial.println("  -> JA VISTO, descartando");
    return;
  }
  hashCache.add(msg.src, msg.msgId);

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
static void sendMessage(MeshMessage &msg) {
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
static void handleData(MeshMessage &msg, int rssi_dbm) {
  #if MEU_ID == BASE_ID //só base
    Serial.printf("Dado: src=%d TTL=%d rssi=%d temp=%d umid=%d\n",
                  msg.src, msg.ttl, rssi_dbm,
                  msg.payload[0], msg.payload[1]);

    enviarParaSheets(msg.src, msg.payload[0], msg.payload[1]);
    
  #else //só sensor
  if (rssi_dbm >= RSSI_MIN_LEARN) {
    if (routeState.learn(msg.srcHopsToBase)) {
      Serial.printf("Nova distancia: %d (via %d, RSSI=%d)\n",
                    routeState.hops(), msg.src, rssi_dbm);
    }
  }

  if (msg.ttl == 0) return;

  uint8_t localHops = routeState.hops();

  // Gossip ponderado por RSSI:
  //  sinal forte    → vizinhos do origem provavelmente já receberam (baixa prob)
  //  sinal fraco    → somos talvez o último elo confiável (alta prob)
  uint8_t fwdProb;
  if (rssi_dbm > -50)                  fwdProb = 15;
  else if (rssi_dbm > RSSI_MIN_LEARN)  fwdProb = 30;
  else                                 fwdProb = 0;

  bool fwd = (msg.srcHopsToBase > localHops) ||
             (msg.srcHopsToBase == localHops && random(0, 100) < fwdProb);

  if (fwd) {
    msg.ttl--;
    msg.srcHopsToBase = localHops;
    msg.checkSum = calculaCrcMsg(msg);
    fwdQueue.enqueue(msg, millis() + random(50, 300));
  } else {
    Serial.printf("Nao retransmitindo de %d (RSSI=%d, prob=%d)\n",
                  msg.src, rssi_dbm, fwdProb);
  }
  #endif
}

#if MEU_ID != BASE_ID
static void handleBeacon(MeshMessage &msg, int rssi_dbm) {
  // Nós distantes aprendem hopsToBase via pacotes de dados encaminhados.
  if (rssi_dbm >= RSSI_MIN_LEARN) {
    if (routeState.learn(msg.srcHopsToBase)) {
      Serial.printf("Nova distancia (beacon): %d (RSSI=%d)\n",
                    routeState.hops(), rssi_dbm);
    }
  }
}
#endif

// ======================= ENVIOS PERIÓDICOS ======================

#if MEU_ID == BASE_ID
static void sendBeacon() {
  MeshMessage beacon;
  memset(&beacon, 0, sizeof(beacon));
  beacon.startByte     = START_BYTE;
  beacon.msgId         = getNextMsgId();
  beacon.src           = MEU_ID;
  beacon.ttl           = 1;
  beacon.srcHopsToBase = 0;
  beacon.type          = MSG_BEACON;
  beacon.checkSum      = calculaCrcMsg(beacon);
  sendMessage(beacon);
  Serial.printf("Beacon enviado (dist=0)\n");
}
#endif

#if MEU_ID != BASE_ID
static void sendDataToBase() {
  if (!routeState.hasRoute()) {
    Serial.println("Sem rota, aguardando topologia convergir");
    return;
  }
  uint8_t localHops = routeState.hops();
  MeshMessage data;
  memset(&data, 0, sizeof(data));
  data.startByte     = START_BYTE;
  data.msgId         = getNextMsgId();
  data.src           = MEU_ID;
  data.ttl           = TTL_INICIAL;
  data.srcHopsToBase = localHops;
  data.type          = MSG_DATA;
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  //if (isnan(t) || isnan(h)) return;   // não transmite leitura inválida
  data.payload[0] = (uint8_t)t;
  data.payload[1] = (uint8_t)h;
  data.checkSum      = calculaCrcMsg(data);
  sendMessage(data);
  Serial.printf("Dado enviado (dist=%d): T=%d H=%d\n",
                localHops, data.payload[0], data.payload[1]);
}
#endif

static uint8_t getNextMsgId() { return nextMsgId++; }
