#include <esp_now.h>
#include <WiFi.h>

typedef struct message {
  int senderID;         // 0 = Device A
  char msgType[20];     // "STATUS_OK", "HELP_REQUEST", etc.
  float posX;
  float posY;
} message;

message incoming, outgoing;

// -------- Device A Settings --------
uint8_t peerAddress[] = {0xCC, 0xDB, 0xA7, 0x9D, 0x73, 0xA8};  // Device B MAC
int myID = 0;  // Device A ID

bool isStuck = false;
unsigned long lastStatusTime = 0;

// === Receive Callback ===
void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
  memcpy(&incoming, data, sizeof(incoming));
  Serial.printf("\n📩 From Robot %d: %s\n", incoming.senderID, incoming.msgType);

  if (strcmp(incoming.msgType, "HELP_ACK") == 0) {
    Serial.println("🤝 Helper acknowledged. Waiting...");
  }
  if (strcmp(incoming.msgType, "HELP_DONE") == 0) {
    Serial.println("🏁 Help completed. Resuming task.");
    isStuck = false;
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.print("My MAC: "); Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!"); while (true) delay(1000);
  }
  esp_now_add_peer(peerAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  esp_now_register_recv_cb(onReceive);

  outgoing.senderID = myID;
  outgoing.posX = 0;
  outgoing.posY = 0;
}

void loop() {
  unsigned long now = millis();

  // Periodic STATUS
  if (now - lastStatusTime > 3000 && !isStuck) {
    lastStatusTime = now;
    strcpy(outgoing.msgType, "STATUS_OK");
    esp_now_send(peerAddress, (uint8_t*)&outgoing, sizeof(outgoing));
    Serial.println("📡 STATUS_OK sent.");
  }

  // Simulated stuck event after 15 s
  if (!isStuck && now > 15000) {
    Serial.println("🚨 Robot stuck! Sending HELP_REQUEST...");
    strcpy(outgoing.msgType, "HELP_REQUEST");
    esp_now_send(peerAddress, (uint8_t*)&outgoing, sizeof(outgoing));
    isStuck = true;
  }
}
