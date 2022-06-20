#ifndef ESP_NOW_STUFF_H
#define ESP_NOW_STUFF_H

#include <Arduino.h>
#include <esp_now.h>
// #include <motor_ctrl_stuff.h>
#include <ssid_stuff.h> //this file contains my ssid and password.

TaskHandle_t send_status_task;

QueueHandle_t send_status_queu;

esp_now_peer_info_t peerInfo = {};

typedef struct test_struct
{
    String status;
    // String x2;
    // String x3;

} test_struct;

test_struct myData;

// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
uint8_t broadcastAddress1[] = {0x24, 0xD7, 0xEB, 0x0F, 0x97, 0x54};
uint8_t broadcastAddress2[] = {0x24, 0xDF, 0xEB, 0x0F, 0x98, 0x44};

// these are in ssid_stuff.h.
// const char *ssid = "***";
// const char *password = "***";

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    char macStr[18];
    Serial.print("Packet to: ");
    // Copies the sender mac address to a string
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.print(macStr);
    Serial.print(" send status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void register_peers()
{
    // Serial.begin(115200);
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_send_cb(OnDataSent);

    // register peer

    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    // register first peer
    memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer 2");
        digitalWrite(LED_BUILTIN, HIGH); // return;
    }
    // register second peer
    memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer 3");
        digitalWrite(LED_BUILTIN, HIGH); // return;
    }
    //  /// register third peer
    //  memcpy(peerInfo.peer_addr, broadcastAddress3, 6);
    //  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    //    Serial.println("Failed to add peer");
    //    return;
    //  }
}

// send the motor status to control the lights so far
int status_send_mtr_status(String sttus)
{
    myData.status = sttus;
    esp_err_t result = esp_now_send(broadcastAddress1, (uint8_t *)&myData, sizeof(myData));

    if (result == ESP_OK)
    {
        // Serial.println("Sent with success");
    }
    else
    {
        // Serial.println("Error sending the data");
    }

    return result;
}

void send_mtr_status_rtos(void *params)
{

    for (;;)
    {
        test_struct mydata;
        xQueueReceive(send_status_queu, &mydata, portMAX_DELAY);
        esp_err_t result = esp_now_send(broadcastAddress1, (uint8_t *)&mydata, sizeof(mydata));
        if (result == ESP_OK)
        {
            // Serial.println("Sent with success");
        }
        else
        {
            // Serial.println("Error sending the data");
        }
    }
}

void esp_now_setup(){
    register_peers();
    send_status_queu = xQueueCreate(3, sizeof(test_struct));
    xTaskCreate(
        send_mtr_status_rtos, /* Function to implement the task */
        "update_motor_status",    /* Name of the task */
        2048,            /* Stack size in words */
        NULL,            /* Task input parameter */
        0,               /* Priority of the task */
        &send_status_task     /* Task handle. */);              
        /* Core where the task should run */
}

#endif