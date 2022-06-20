// this is for the code for the periperal board that has the lights and the screen.
//  light will be connected on pin 13.

#include <Arduino.h>
#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Arduino_JSON.h>
#ifndef PSTR
#define PSTR // Make Arduino Due happy
#endif
#include <ssid_stuff.h>

#define PIN 13
#define NUMPIXELS 4

#define white 255
#define red 22672
#define orange 15661
#define yellow 14746
#define green 1638
#define blue 45875
#define cyan 38502
#define pink 27034
#define purple 31949

TaskHandle_t flash_colours_task;

// QueueHandle_t queue;
// QueueHandle_t queue_ret;

// UBaseType_t n_messages;

JSONVar motor_status_json;

String variable = "stop";

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


void flash_leds_loop(void *parameter)
{

  bool col = true;

  for (;;)
  {
    vTaskDelay(350 / portTICK_PERIOD_MS);
    // Serial.println("flashing leds");
    // Serial.println(variable);
    if (col)
    {
      // Serial.println(col);
      col = !col;
      pixels.fill(pixels.gamma32(pixels.ColorHSV((uint16_t)(int)motor_status_json[variable][0])), 0, ((int)(NUMPIXELS / 2)));
      pixels.fill(pixels.gamma32(pixels.ColorHSV((uint16_t)(int)motor_status_json[variable][1])), ((int)(NUMPIXELS / 2)), ((int)(NUMPIXELS / 2)));
      pixels.show();
    }
    else
    {
      col = !col;
      pixels.fill(pixels.gamma32(pixels.ColorHSV((uint16_t)(int)motor_status_json[variable][1])), 0, ((int)(NUMPIXELS / 2)));
      pixels.fill(pixels.gamma32(pixels.ColorHSV((uint16_t)(int)motor_status_json[variable][0])), ((int)(NUMPIXELS / 2)), ((int)(NUMPIXELS / 2)));
      pixels.show();
    }
  }
}

int32_t getWiFiChannel(const char *ssid)
{
  if (int32_t n = WiFi.scanNetworks())
  {
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

typedef struct data_struct
{
  String status = "stop";

} data_struct;

data_struct myData;


enum state
{
  fwd = 0,
  rev,
  lft,
  rgt,
  fwdrgt,
  fwdlft,
  revrgt,
  revlft,
  stp,
  e_stop,
  e_stop_clear
};

void init_msj()
{
  motor_status_json["forward"][0] = ((int)green);
  motor_status_json["back"][0] = ((int)red);
  motor_status_json["left"][0] = ((int)blue);
  motor_status_json["right"][0] = ((int)orange);
  motor_status_json["forwardleft"][0] = ((int)yellow);
  motor_status_json["backleft"][0] = ((int)purple);
  motor_status_json["forwardright"][0] = ((int)blue);
  motor_status_json["backright"][0] = ((int)yellow);
  motor_status_json["stop"][0] = ((int)red);
  motor_status_json["e_stop"][0] = ((int)red);
  motor_status_json["e_stop_clear"][0] = ((int)pink);

  motor_status_json["forward"][1] = ((int)blue);
  motor_status_json["back"][1] = ((int)orange);
  motor_status_json["left"][1] = ((int)red);
  motor_status_json["right"][1] = ((int)green);
  motor_status_json["forwardleft"][1] = ((int)cyan);
  motor_status_json["backleft"][1] = ((int)pink);
  motor_status_json["forwardright"][1] = ((int)orange);
  motor_status_json["backright"][1] = ((int)green);
  motor_status_json["stop"][1] = ((int)pink);
  motor_status_json["e_stop"][1] = ((int)pink);
  motor_status_json["e_stop_clear"][1] = ((int)pink);
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{

  // Serial.print("Bytes received: ");
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("data recieved: ");
  Serial.println(myData.status);
  Serial.println();

  variable = myData.status;

  // update flash colours task

  // || !strcmp(variable, "backward")|| !strcmp(variable, "forward") || !strcmp(variable, "left") || !strcmp(variable, "right")){
}

void setup()
{
  Serial.begin(115200);
  pixels.begin();
  pixels.setBrightness(200);

  pixels.fill(pixels.gamma32(pixels.ColorHSV(pink)), 0, 10);
  //  pixels.setPixelColor(0, pixels.gamma32(pixels.ColorHSV((1*i) * 256 * 8 + i * 128 * 2)));

  pixels.show();
  //

  init_msj();

  WiFi.mode(WIFI_STA);

  int32_t channel = getWiFiChannel(ssid);

  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.printDiag(Serial); // Uncomment to verify channel change after

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

  xTaskCreatePinnedToCore(flash_leds_loop, /* Function to implement the task */
        "flash_leds",    /* Name of the task */
        2048,            /* Stack size in words */
        NULL,            /* Task input parameter */
        0,               /* Priority of the task */
        &flash_colours_task,     /* Task handle. */
        1);              /* Core where the task should run */
}

void loop()
{
  // put your main code here, to run repeatedly:
  vTaskDelete(NULL);
}