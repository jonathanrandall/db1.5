

// Import required libraries
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include "motor_ctrl_stuff.h"
#include "ssid_stuff.h"
#include "encoders_stuff.h"
#include "OLED_stuff.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "imu_stuff.h"
#include "goal_seek_suff.h"

/*
#define LEFT_MTR_DIR 26
#define LEFT_MTR_PWM 27
#define RIGHT_MTR_DIR 12
#define RIGHT_MTR_PWM 32

#define encoder0PinA 33      // encoder 1
#define encoder0PinB 4

#define encoder1PinA 16     // encoder 2
#define encoder1PinB 17
*/

// const char *ssid = "";  //defined in ssid.h
// const char *password = "";
// Create AsyncWebServer object on port 80

#define EM_STOP_INT_PIN 18
#define EM_STOP_INT_CLEAR_PIN 19

AsyncWebServer server(80);
// Create a WebSocket object
AsyncWebSocket ws("/ws");

unsigned long last_update;
unsigned long ws_clean;
unsigned long clean_up_every = 10000;

TaskHandle_t clean_up_ws;
TaskHandle_t stp_robot_moving;
TaskHandle_t debug_loop_task;

SemaphoreHandle_t Semaphore_ws;

// set up json value
bool em_stop_int = false;
bool em_stop_cleared = false;

// enum state {fwd=0, rev=1, lft=2, rgt=3, stp=4,e_stop,e_stop_clear};

JSONVar motor_status_json;

float goal_dist;

void IRAM_ATTR emergency_stop()
{
  em_stop_int = true;
  // actstate = e_stop;

  // notifyClients();
}

void IRAM_ATTR clear_emergency_stop()
{
  em_stop_cleared = true;
  // delay(5);
  // robot_set_and_send_command(e_stop_clear);
  // actstate = e_stop_clear;
  // set_actstate(e_stop_clear);
  // notifyClients();
}

void initSPIFFS()
{
  if (!SPIFFS.begin(true))
  {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}

// Initialize WiFi
void initWiFi()
{
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

// motor status json
void init_msj()
{
  motor_status_json["forward"] = ((int)fwd);
  motor_status_json["back"] = ((int)rev);
  motor_status_json["left"] = ((int)lft);
  motor_status_json["right"] = ((int)rgt);
  motor_status_json["forwardleft"] = ((int)fwdlft);
  motor_status_json["backleft"] = ((int)revlft);
  motor_status_json["forwardright"] = ((int)fwdrgt);
  motor_status_json["backright"] = ((int)revrgt);
  motor_status_json["stop"] = ((int)stp);
  motor_status_json["e_stop"] = ((int)e_stop);
  motor_status_json["e_stop_clear"] = ((int)e_stop_clear);
}

void notifyClients()
{
  str_status = status_names[(int)actstate];
  String status = "{\"speed\":" + String(motor_speed) + ",\"status\":" + "\"" + str_status + "\"" + ",\"goal_dist\":" + "\"" + goal_dist + "\"}";
  // Serial.println(status);
  // ws.cleanupClients();
  ws.textAll(status);
}

void notifyClients(const char *status)
{

  str_status = String(status);

  int stmp = motor_status_json[status];

  robot_set_and_send_command((state)stmp);
  Serial.println(actstate);
  // notifyClients();
}

// mostly cut and pasted from random nerd tutorials.
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
  {
    data[len] = 0;
    // Serial.println((char *)data);
    JSONVar received_object = JSON.parse((char *)data);
    const char *action = received_object["action"];
    if (strcmp(action, "speed") == 0)
    {
      // Serial.println(received_object["value"]);
      motor_speed = (int)atoi(received_object["value"]);
      // motor_speed = map(motor_speed, 0, 100, 0,255)
      // Serial.println(motor_speed);
      // robot_set_speed();
      notifyClients();
    }
    if (strcmp(action, "goal_dist") == 0)
    {
      // Serial.println(received_object["value"]);
      goal_dist = (int)atoi(received_object["value"]);

      notifyClients();
    }
    if (strcmp(action, "status") == 0)
    {
      last_update = millis();
      const char *status = received_object["value"];
      str_status = String(status);
      // Serial.println(str_status);
      if (motor_status_json.hasOwnProperty(status))
      {
        int stmp = motor_status_json[status];
        robot_set_and_send_command((state)stmp);
        str_status = status_names[(int)actstate];
        // notifyClients();
        // if (millis() - ws_clean > (clean_up_every))
        // {
        //   ws.cleanupClients();
        //   ws_clean = millis();
        // }
      }
      else
      {
        // do nothing
      }

      // Serial.println(stmp);
    }
    if (strcmp(action, "goal_status") == 0)
    {
      Serial.println("goal status received");
      last_update = millis();
      const char *status = received_object["value"];
      str_status = String(status);
      // Serial.println(str_status);
      if (motor_status_json.hasOwnProperty(status))
      {
        try
        {
          Serial.println("trying to delete taks");

          if (goal_task_running)
            if (update_robot_pos_task != NULL)
            {
              vTaskDelete(update_robot_pos_task); // freeRTOS
              update_robot_pos_task = NULL; //you need to set this to NULL for some reason.
            }
        }
        catch (...)
        {
          Serial.println("exdeption");
        }

        xSemaphoreTake(Semaphore_prev_time, portMAX_DELAY);
        goal_task_running = true;
        xSemaphoreGive(Semaphore_prev_time);
        // this will be multiplied by pi/2
        int stmp = motor_status_json[status];
        if (stmp >= (int)stp)
        {
          robot_set_and_send_command((state)stmp);
          str_status = status_names[(int)actstate];
        }
        else
        {

          stmp = motor_status_direction[status];
          // set robot new target.
          
          target_pos.theta = ((float) ((int) stmp)) * pi / 4;
          Serial.println(target_pos.theta);
          target_pos.y = goal_dist * sin(target_pos.theta);
          target_pos.x = goal_dist * cos(target_pos.theta);
          robot_pos = {0.0, 0.0, 0.0};

          robot_set_and_send_command(fwd);
          str_status = status_names[(int)actstate];

          Serial.println("starting goal task");

          xTaskCreatePinnedToCore(
              update_robot_pos_loop,  /* Function to implement the task */
              "update_robot_pos",     /* Name of the task */
              1024 * 4,               /* Stack size in words */
              NULL,                   /* Task input parameter */
              0,                      /* Priority of the task */
              &update_robot_pos_task, /* Task handle. */
              1);                     /* Core where the task should run */
        }
        // notifyClients();
        // if (millis() - ws_clean > (clean_up_every))
        // {
        //   ws.cleanupClients();
        //   ws_clean = millis();
        // }
      }
      else
      {
        // do nothing
      }

      // Serial.println(stmp);
    }
    if (strcmp(action, "open") == 0)
    {
      // const char *status = received_object["value"];
      str_status = status_names[(int)actstate];
      ws.cleanupClients(); // everytime a new connection is opened.

      notifyClients();
    }
    if (strcmp(action, "update") == 0)
    {
      // const char *status = received_object["value"];
      // if it has been more than 90ms since last recieved don't do anything
      if (millis() - last_update > (message_time - 10))
      {
        // then ew update
        last_update = millis();
        const char *status = received_object["value"];

        str_status = String(status);

        if (motor_status_json.hasOwnProperty(status))
        {
          xSemaphoreTake(Semaphore_prev_time, portMAX_DELAY);
          previous_time = millis();
          xSemaphoreGive(Semaphore_prev_time);

          // int stmp = motor_status_json[status];
          // robot_set_and_send_command((state)stmp);
          // str_status = status_names[(int)actstate];
        }
        else
        {
          // do nothing
        }
      }
    }
  }
}
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    break;
  case WS_EVT_DISCONNECT:
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
    break;
  case WS_EVT_DATA:
    handleWebSocketMessage(arg, data, len);
    break;
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  }
}
void initWebSocket()
{
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void clean_ws(void *parameters)
{
  for (;;)
  {
    vTaskDelay(100000 / portTICK_PERIOD_MS);
    xSemaphoreTake(Semaphore_ws, portMAX_DELAY);
    ws.cleanupClients();
    xSemaphoreGive(Semaphore_ws);
  }
}

// if we haven't recieved any commands for 500ms, then we will stop the robot. This is because if we set a command on a website
// and then get disconnected, it will not keep running.
void call_stp(void *parameters)
{
  unsigned long pt;
  for (;;)
  {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    xSemaphoreTake(Semaphore_prev_time, portMAX_DELAY);
    pt = previous_time;
    xSemaphoreGive(Semaphore_prev_time);

    if (millis() - pt > 500)
    {
      // Serial.println(millis()-pt);
      robot_set_and_send_command(stp);
      // Serial.println("Stopping");
    }
  }
}

void debug_loop(void *parameters)
{

  for (;;)
  {
    vTaskDelay(500 / portTICK_PERIOD_MS);

    if (em_stop_cleared)
    {
      if (actstate == e_stop)
        robot_set_and_send_command(e_stop_clear);
      // notifyClients();
      em_stop_cleared = false;
      Serial.println("in em stop clear");
    }

    if (em_stop_int)
    {
      notifyClients();
      em_stop_int = false;
      robot_set_and_send_command(e_stop);
      Serial.println("in em stop int");
    }

    // Serial.print("left: ");
    // Serial.println(encoder1Pos);
    // Serial.println(encoder0Pos);
    if (false)
      if ((float)abs(encoder1Pos) > one_m)
      {
        last_update = millis();

        str_status = "stop";
        Serial.println(str_status);
        // int stmp = motor_status_json["stop"];
        // Serial.println(stmp);
        robot_set_and_send_command((state)stp);
        encoder1Pos = 0;
        // notifyClients();//this gets called in set and set command
      }
  }
}

void setup()
{
  // put your setup code here, to run once:
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  actstate = e_stop_clear; // e_stop;

  robot_pos = {0.0, 0.0, 0.0}; // pi / 2};
  target_pos = {0.0, 300.0, pi / 2};
  theta = robot_pos.theta;

  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  // oled stuff to display ip
  oled.clearDisplay();
  oled.setFont(&FreeSans9pt7b);
  oled.setTextColor(WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 17);

  oled.println(F("Starting..."));
  oled.display();
  // delay(2000);

  init_msj();
  init_msd();
  initSPIFFS();

  initWiFi();

  initWebSocket();

  esp_now_setup();

  pinMode(EM_STOP_INT_CLEAR_PIN, INPUT_PULLDOWN);
  pinMode(EM_STOP_INT_PIN, INPUT_PULLDOWN);

  attachInterrupt(EM_STOP_INT_PIN, emergency_stop, RISING);
  attachInterrupt(EM_STOP_INT_CLEAR_PIN, clear_emergency_stop, RISING);

  // set_actstate(e_stop);//start in emergency stop. So we need a clear stop before we can move.

  // register_peers();

  // init_encoders();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.html", "text/html"); });

  server.on("/goal_seek", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index_goal.html", "text/html"); });

  server.on("/manual_drive", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.html", "text/html"); });

  server.serveStatic("/", SPIFFS, "/");

  // Start server
  server.begin();

  robot_setup();

  previous_time = millis();
  last_update = millis();

  Semaphore_prev_time = xSemaphoreCreateMutex();
  Semaphore_ws = xSemaphoreCreateMutex();

  // xTaskCreatePinnedToCore(
  //     clean_ws,      /* Function to implement the task */
  //     "clean_up_ws", /* Name of the task */
  //     1000,          /* Stack size in words */
  //     NULL,          /* Task input parameter */
  //     0,             /* Priority of the task */
  //     &clean_up_ws,  /* Task handle. */
  //     1);            /* Core where the task should run */

  if (false)
    xTaskCreatePinnedToCore(
        call_stp,          /* Function to implement the task */
        "call_stp",        /* Name of the task */
        15360,             /* Stack size in words */
        NULL,              /* Task input parameter */
        0,                 /* Priority of the task */
        &stp_robot_moving, /* Task handle. */
        1);                /* Core where the task should run */

  xTaskCreatePinnedToCore( // set true if you want to debug
      debug_loop,          /* Function to implement the task */
      "debug_loop",        /* Name of the task */
      2000,                /* Stack size in words */
      NULL,                /* Task input parameter */
      0,                   /* Priority of the task */
      &debug_loop_task,    /* Task handle. */
      1);
  check_sonar_queue = xQueueCreate(1, sizeof(data_struct_rcv));

  xTaskCreatePinnedToCore(
      check_sonar_loop,   /* Function to implement the task */
      "check_sonar_loop", /* Name of the task */
      2000,               /* Stack size in words */
      NULL,               /* Task input parameter */
      0,                  /* Priority of the task */
      &check_sonar_task,  /* Task handle. */
      1);

  init_encoders();

  notifyClients(); // emergency stop
  oled.clearDisplay();
  oled.setFont(&FreeSans9pt7b);
  oled.setTextColor(WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 17);

  oled.println(WiFi.localIP());
  oled.display();
}

void loop()
{
  // put your main code here, to run repeatedly:
  vTaskDelete(NULL);
  // ws.cleanupClients();
  // if (millis() - previous_time > move_interval)
  // {
  //   // robot_set_and_send_command(stp);
  // }
}