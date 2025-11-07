#include <Arduino.h>
#include "esp_mac.h" // Native ESP32 Mac header

void setup() {
  Serial.begin(115200);
  delay(2000); // Longer delay to ensure Serial is ready
  Serial.println("\n\n--- Direct Hardware MAC Read ---");

  uint8_t baseMac[6];
  // Attempt to read directly from standard EFUSE
  esp_err_t ret = esp_efuse_mac_get_default(baseMac);

  if (ret == ESP_OK) {
    Serial.print("Factory MAC: ");
    printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
           baseMac[0], baseMac[1], baseMac[2],
           baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Error: Could not read MAC from hardware EFUSE.");
  }
  Serial.println("--------------------------------");
}

void loop() {
}
