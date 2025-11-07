DEVICE A 

// Stage2_sender_leds.ino  (Device A)
#include <WiFi.h>
#include <esp_now.h>

int ledPins[] = {12,13,14,15,16,17,18,19};
const int numLEDs = 8; int ledIndex = 0;
uint8_t peerAddress[] = {0xCC,0xDB,0xA7,0x9D,0x73,0xA8}; // Device B MAC
int myID = 0;
typedef struct { int senderID; int counter; } msg_t;
msg_t m;

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status){
  if(status == ESP_NOW_SEND_SUCCESS){
    digitalWrite(ledPins[ledIndex], !digitalRead(ledPins[ledIndex]));
    ledIndex = (ledIndex + 1) % numLEDs;
  }
  Serial.printf("send status: %s\n", status==ESP_NOW_SEND_SUCCESS?"OK":"FAIL");
}

void setup(){
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); Serial.println(WiFi.macAddress());
  for(int i=0;i<numLEDs;i++){ pinMode(ledPins[i], OUTPUT); digitalWrite(ledPins[i], LOW); }
  if(esp_now_init()!=ESP_OK){Serial.println("esp_now fail"); while(1) delay(1000);}
  esp_now_register_send_cb(onDataSent);
  esp_now_peer_info_t p={}; memcpy(p.peer_addr, peerAddress, 6); p.channel=0; p.encrypt=false;
  esp_now_add_peer(&p);
  m.senderID=myID;
}

void loop(){
  static int c=0; m.counter=c++;
  esp_err_t r = esp_now_send(peerAddress, (uint8_t*)&m, sizeof(m));
  Serial.printf("sent %d -> %d\n", m.counter, r);
  delay(2000);
}

--------------------------------------------------------------------------------------------------------------------------------------------------------------
  --------------------------------------------------------------------------------------------------------------------------------------------------------------------

  DEVICE B

  // Stage2_receiver_leds.ino  (Device B)
#include <WiFi.h>
#include <esp_now.h>

int ledPins[] = {12,13,14,15,16,17,18,19};
const int numLEDs = 8; int ledIndex=0;
uint8_t peerAddress[] = {0xCC,0xDB,0xA7,0x97,0x7E,0xDC}; // Device A MAC
typedef struct { int senderID; int counter; } msg_t; msg_t in;

void onReceive(const esp_now_recv_info *info, const uint8_t *data, int len){
  memcpy(&in, data, sizeof(in));
  Serial.printf("RX from %d : %d\n", in.senderID, in.counter);
  digitalWrite(ledPins[ledIndex], !digitalRead(ledPins[ledIndex]));
  ledIndex = (ledIndex + 1) % numLEDs;
}

void setup(){
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); Serial.println(WiFi.macAddress());
  for(int i=0;i<numLEDs;i++){ pinMode(ledPins[i], OUTPUT); digitalWrite(ledPins[i], LOW); }
  if(esp_now_init()!=ESP_OK){Serial.println("esp_now fail"); while(1) delay(1000);}
  esp_now_peer_info_t p={}; memcpy(p.peer_addr, peerAddress, 6); p.channel=0; p.encrypt=false;
  esp_now_add_peer(&p);
  esp_now_register_recv_cb(onReceive);
}

void loop(){ delay(100); }

