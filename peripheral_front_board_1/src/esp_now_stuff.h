#ifndef ESP_NOW_STUFF_
#define ESP_NOW_STUFF_
#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "helpers_h.h"

// Set your Board ID (ESP32 Sender #1 = BOARD_ID 1, ESP32 Sender #2 = BOARD_ID 2, etc)
#define BOARD_ID 2

const char *ssid = "WiFi-C5BF";
//pippo.toCharArray(myData.a, 85);  



uint8_t broadcastAddress[] = {0x7C, 0x9E, 0xBD, 0x48, 0x0F, 0xA4};

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
//  if (status == 0)
//  {
//    success = "Delivery Success :)";
//  }
//  else
//  {
//    success = "Delivery Fail :(";
//  }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{

  // Serial.print("Bytes received: ");
  memcpy(&myData, incomingData, sizeof(myData));
//  Serial.print("Bytes received: ");
//  Serial.println(len);
//  Serial.print("data recieved: ");
//  Serial.println(myData.status);
//  Serial.println();
}

int32_t getWiFiChannel(const char *ssid)
{
//  Serial.println(ssid);
  if (int32_t n = WiFi.scanNetworks())
  {
//    Serial.println(n);
    for (uint8_t i = 0; i < n; i++)
    {
      if (!strcmp(ssid, WiFi.SSID(i).c_str()))
      {
        return WiFi.channel(i);
      }
    }
  }
  return 0;
}

void esp_now_setup(){
  WiFi.mode(WIFI_STA);

  int32_t channel = getWiFiChannel(ssid);
//  Serial.println("here1");
//  WiFi.printDiag(Serial); // Uncomment to verify channel number before
//  Serial.println("here");
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
//  WiFi.printDiag(Serial); // Uncomment to verify channel change after

  Serial.println(" initializing ESP-NOW");
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  else
  {
    Serial.println("ESP now intialised");
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);

  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
  }
}

void esp_now_loop(void *params){

  for(;;){
//    xSemaphoreTake(Semaphore, portMAX_DELAY);
//    xSemaphoreGive(Semaphore);
    vTaskDelay(250 / portTICK_PERIOD_MS); 
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  }
}

#endif
