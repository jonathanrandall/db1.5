#ifndef HELPERS_H_
#define HELPERS_H_
#include <Arduino.h>

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "sonar_jr.h"

#define NUM_READINGS 10

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// at the moment i've got three in the middle, two at each edge and two facing the bottom. I think I'll give the onces facing the bottom a miss now.

#define TRIGGER_PIN_RS 27 // left front purple 5
#define ECHO_PIN_RS 26  //white 18
#define TRIGGER_PIN_LS 2 // left side grren
#define ECHO_PIN_LS 4  //yellow
#define TRIGGER_PIN_RF 13 // right front gray
#define ECHO_PIN_RF 14 //purple
#define TRIGGER_PIN_LF 5 // right side blue
#define ECHO_PIN_LF 18 //yellow

// #define TRIGGER_PIN_RB 16 // right bottom
// #define ECHO_PIN_RB 17
// #define TRIGGER_PIN_LB ? // left bottom
// #define ECHO_PIN_LB ?

#define TRIGGER_PIN_MID 32 // left bottom green 
#define ECHO_PIN_MID 33 //white

#define SERVOL_PIN 23
#define SERVOR_PIN 25

#define NUM_SONARS 5

QueueHandle_t servo_sonar_queue_l;
QueueHandle_t servo_sonar_queue_r;

typedef struct data_struct_send
{
  uint8_t id = 2; // id for data
  bool status = false;
  char place_holder[12];
  float distances[10];
} data_struct_send;

data_struct_send myData;

typedef struct servo_sonar
{
  Servo servo;
  uint8_t id;
  //  uint8_t indices[3];
  float distances[3];
} servo_sonar;

servo_sonar left_servo_sonar;
servo_sonar right_servo_sonar;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

TaskHandle_t Task0;
TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;
TaskHandle_t servo_sonar_task0;
TaskHandle_t servo_sonar_task1;
TaskHandle_t esp_now_loop_task;
TaskHandle_t main_loop_tsk;
TaskHandle_t control_servo_task;
SemaphoreHandle_t Semaphore;
Servo servo_l;
Servo servo_r;

sonar_jr sonarlf(TRIGGER_PIN_LF, ECHO_PIN_LF);    // 3
sonar_jr sonarls(TRIGGER_PIN_LS, ECHO_PIN_LS);    // mounted on the servo 0,1,2
sonar_jr sonarrf(TRIGGER_PIN_RF, ECHO_PIN_RF);    // 5
sonar_jr sonarrs(TRIGGER_PIN_RS, ECHO_PIN_RS);    // mounted on the servo 6,7,8
sonar_jr sonarmid(TRIGGER_PIN_MID, ECHO_PIN_MID); // 4
// sonar_jr sonarrb(TRIGGER_PIN_RB, ECHO_PIN_RB); // 9
// sonar_jr sonarlb(TRIGGER_PIN_LB, ECHO_PIN_LB); // 0

sonar_jr sonars_all[NUM_SONARS] = {sonarlf, sonarls, sonarrf, sonarrs, sonarmid};

uint8_t sonar_indices[NUM_SONARS] = {3, 0, 5, 6, 4};
// sonrs 1,3 are on the servos.

// float distances_all[NUM_SONARS] = {0,0,0,0,0,0};
// float distances_to_send[NUM_READINGS] = {0};

void sonar1_loop(void *params);
void main_loop(void *params);
void sonar_servo_loop(void *params);

void sonar1_loop(void *params)
{
  int s11;
  s11 = *((int *)params);
  Serial.println(s11);
  for (;;)
  {

    myData.distances[sonar_indices[s11]] = sonars_all[s11].get_distance1();

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void sonar1_loop_mid(void *params)
{
  // lf, rf, mid = 0,2,4
  int front_sonars[3] = {0,2,4};
  // float r1, r2, r3;
  for (;;)
  {

    for (int i = 0; i < 3; i++)
    {
      // r1 = sonars_all[front_sonars[i]].get_distance();

      myData.distances[sonar_indices[front_sonars[i]]] = sonars_all[front_sonars[i]].get_distance1();
      vTaskDelay(5 / portTICK_PERIOD_MS);
    }
  }
}

void sonar1_loop_all(void *params)
{
  // lf, rf, mid = 0,2,4
  int front_sonars[3] = {0,2,4};
  int slid = 1;
  int srid = 3;
  // float r1, r2, r3;
  for (;;)
  {

    for (int i = 0; i < 3; i++)
    {
      // r1 = sonars_all[front_sonars[i]].get_distance();

      myData.distances[sonar_indices[front_sonars[i]]] = sonars_all[front_sonars[i]].get_distance();
      vTaskDelay(20 / portTICK_PERIOD_MS);
    }

    myData.distances[sonar_indices[slid]] = sonars_all[slid].get_distance();
    vTaskDelay(20 / portTICK_PERIOD_MS);
    myData.distances[sonar_indices[srid]] = sonars_all[srid].get_distance();


  }
}

void sonar_servo_loop_l(void *params)
{
  
  // s11 = *((servo_sonar *)params);
  // Serial.println(s11.id);
  int s11id = 1;  //left_servo_sonar.id;
  int it_order[4] = {0,1,2,1}; //cause I want it to go back and forward
  for (;;)
  {
    float res;

    for (int j = 0; j < 1; j++)
    {
      // left_servo_sonar.servo.write(it_order[j] * 26 + 10);
      // i've commented out the sonar suff. It's one more thing for an error.
      // servo_l.write(it_order[j] * 26 + 10);
      
      // vTaskDelay(240 / portTICK_PERIOD_MS); // give servo time to finish
       res = sonars_all[s11id].get_distance1();
      // if(res!=0)
       myData.distances[sonar_indices[s11id] +it_order[j]] = res;
     
      vTaskDelay(5 / portTICK_PERIOD_MS);
      //      Serial.println(myData.distances[sonar_indices[s11.id]+j]);
    }
  }
}

void control_servo_loop(void *params){
  
  int it_order[4] = {0,1,2,1}; //cause I want it to go back and forward
  for (;;)
  {
    

    for (int j = 0; j < 4; j++)
    {
      
      servo_r.write(it_order[j] * 26 + 10);
     
      vTaskDelay(240 / portTICK_PERIOD_MS); // give servo time to finish
      
    }
  }
}

void sonar_servo_loop_r(void *params)
{
  
  // s11 = *((servo_sonar *)params);
  // Serial.println(s11.id);
  int s11id = 3; //right_servo_sonar.id;
  int it_order[4] = {0,1,2,1}; //cause I want it to go back and forward
  for (;;)
  {
    float res;

    for (int j = 0; j < 1; j++)  //changed j to 1 because i'm not using the servos.
    {
      // right_servo_sonar.servo.write(it_order[j] * 26 + 10); //it_order[j] 
      servo_r.write(it_order[j] * 26 + 10);
      // s11.servo.write(0);
      vTaskDelay(200 / portTICK_PERIOD_MS); // give servo time to finish
      res = sonars_all[s11id].get_distance1();
      // if(res!=0) 
      myData.distances[sonar_indices[s11id] +it_order[j]] = res;
     
      vTaskDelay(5 / portTICK_PERIOD_MS);
      //      Serial.println(myData.distances[sonar_indices[s11.id]+j]);
    }
  }
}

void main_loop(void *params)
{

  for (;;)
  {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    display.clearDisplay();
    display.setCursor(0, 5);
    // Display distance in cm
    for (int i = 0; i < 10; i++)
    {
      display.setCursor((int)(i % 3) * 41 + 5, (int)(floor((float)i / 3.0) * 22));
      display.print(myData.distances[i], 0);
    }

    display.display();
  }
}

#endif
