/* Anchor_4anchor_network.ino  */
#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

// === EDIT UNIQUE PER ANCHOR ===
char ANCHOR_ADDR[] = "81:00:5B:D5:A9:9A:E2:9C";  // unique address
uint16_t ADELAY = 16610;                         // calibrated antenna delay
float ANCHOR_X = 0.0;                            // coordinate X (m)
float ANCHOR_Y = 0.0;                            // coordinate Y (m)
// ===============================

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4
const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS  = 4;

void setup() {
  Serial.begin(115200);
  delay(1000);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  DW1000.setAntennaDelay(ADELAY);
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  DW1000Ranging.startAsAnchor(ANCHOR_ADDR, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
  Serial.print("Anchor "); Serial.println(ANCHOR_ADDR);
  Serial.print("Antenna delay: "); Serial.println(ADELAY);
  Serial.print("Position: "); Serial.print(ANCHOR_X); Serial.print(", "); Serial.println(ANCHOR_Y);
}

void loop() { DW1000Ranging.loop(); }

void newRange() {
  Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print(", ");
  Serial.println(DW1000Ranging.getDistantDevice()->getRange(), 3);
}
void newDevice(DW1000Device *device) {
  Serial.print("Device added: "); Serial.println(device->getShortAddress(), HEX);
}
void inactiveDevice(DW1000Device *device) {
  Serial.print("Delete inactive device: "); Serial.println(device->getShortAddress(), HEX);
}

------------------------------------------------------------------------------------------------------------------------------------------------------------
  ----------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* Tag_robot_4anchor_trilateration.ino  */
#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4
const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS  = 4;

// === fill in anchor short addresses (hex) ===
// these are the first two bytes of each anchor's long address
const uint16_t ANCHOR_SHORTS[4] = { 0x8100, 0x8200, 0x8300, 0x8400 };

// === anchor coordinates in meters ===
const float ANCHOR_COORDS[4][2] = {
  {0.0,    0.0},     // A
  {3.048,  0.0},     // B
  {0.0,    3.048},   // C
  {3.048,  3.048}    // D
};

float lastRange[4] = { NAN, NAN, NAN, NAN };
unsigned long lastTs[4] = { 0,0,0,0 };
const int RANGE_TIMEOUT_MS = 1000;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Tag start (4-anchor mode)");
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
}

void loop() {
  DW1000Ranging.loop();
  unsigned long now = millis();
  int have = 0;
  for (int i=0;i<4;i++)
    if (!isnan(lastRange[i]) && (now - lastTs[i] < RANGE_TIMEOUT_MS)) have++;

  if (have >= 3) {
    float x, y;
    if (leastSquares2D(ANCHOR_COORDS, lastRange, 4, x, y)) {
      Serial.print(now); Serial.print(",");
      Serial.print(x,3); Serial.print(",");
      Serial.print(y,3);
      for (int i=0;i<4;i++) { Serial.print(","); Serial.print(lastRange[i],3); }
      Serial.println();
    }
    delay(50);
  }
}

void newRange() {
  uint16_t s = DW1000Ranging.getDistantDevice()->getShortAddress();
  float r = DW1000Ranging.getDistantDevice()->getRange();
  for (int i=0;i<4;i++) if (s == ANCHOR_SHORTS[i]) {
    lastRange[i]=r; lastTs[i]=millis();
  }
}

void newDevice(DW1000Device *d){ Serial.print("Device added: "); Serial.println(d->getShortAddress(),HEX);}
void inactiveDevice(DW1000Device *d){ Serial.print("Lost device: "); Serial.println(d->getShortAddress(),HEX); }

// -------- Least-squares solver for 2D with N>=3 anchors ----------
bool leastSquares2D(const float anchors[][2], const float ranges[], int n, float &x, float &y) {
  if (n < 3) return false;
  // reference anchor (0)
  float x1 = anchors[0][0], y1 = anchors[0][1], r1 = ranges[0];
  // build linear system: A * [x y]^T = b
  float ATA11=0, ATA12=0, ATA22=0, ATb1=0, ATb2=0;
  for (int i=1;i<n;i++) {
    float xi=anchors[i][0], yi=anchors[i][1], ri=ranges[i];
    float Ai1 = 2*(xi - x1);
    float Ai2 = 2*(yi - y1);
    float bi  = r1*r1 - ri*ri - x1*x1 + xi*xi - y1*y1 + yi*yi;
    ATA11 += Ai1*Ai1;
    ATA12 += Ai1*Ai2;
    ATA22 += Ai2*Ai2;
    ATb1  += Ai1*bi;
    ATb2  += Ai2*bi;
  }
  float det = ATA11*ATA22 - ATA12*ATA12;
  if (fabs(det) < 1e-9) return false;
  x = (ATb1*ATA22 - ATA12*ATb2) / det;
  y = (ATA11*ATb2 - ATb1*ATA12) / det;
  return true;
}
