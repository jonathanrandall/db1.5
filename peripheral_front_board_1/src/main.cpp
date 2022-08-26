#include <Arduino.h>

#include "helpers_h.h"
#include "esp_now_stuff.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

int s1 = 0; // lf (left front)
int s2 = 2; // rf
int s3 = 4; // rb
int s4 = 5; // lb

unsigned long last_time_esp_now;
unsigned long time_to_esp_now = 100;

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Setup started.");

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  // Serial.println("finish");
  delay(500);

  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(WHITE);

  display.clearDisplay();
  display.setCursor(0, 2);
  // Display distance in cm
  display.print("Starting...");

  display.setCursor(0, 15);

  display.display();

  esp_now_setup();

  servo_l.attach(SERVOL_PIN);
  servo_r.attach(SERVOR_PIN);

  left_servo_sonar.id = 1;
  // left_servo_sonar.servo.attach(SERVOL_PIN);

  right_servo_sonar.id = 3;
  // right_servo_sonar.servo.attach(SERVOR_PIN);

  // Semaphore = xSemaphoreCreateMutex();
  for (int i = 0; i < NUM_SONARS; i++)
  {
    start_sonar_jr(sonars_all[i]);
  }

  // for (int i = 0; i<10; i++) myData.distances[i] = 0;

  for (int i = 0; i < NUM_SONARS; i++)
  {
    // r1 = sonars_all[front_sonars[i]].get_distance();

    myData.distances[sonar_indices[i]] = 0.7 * sonars_all[i].get_distance();
    delay(5);
  }

  // xTaskCreatePinnedToCore(
  //     sonar1_loop,    /* Function to implement the task */
  //     "sonar1_loop1", /* Name of the task */
  //     2048,           /* Stack size in words */
  //     (void *)&s1,    /* Task input parameter */
  //     1,              /* Priority of the task */
  //     &Task0,         /* Task handle. */
  //     1);             /* Core where the task should run */

  //  xTaskCreatePinnedToCore(
  //      sonar1_loop, /* Function to implement the task */
  //      "sonar1_loop1", /* Name of the task */
  //      2048, /* Stack size in words */
  //      (void *)&s2, /* Task input parameter */
  //      1, /* Priority of the task */
  //      &Task1, /* Task handle. */
  //      1); /* Core where the task should run */
  //
  //   xTaskCreatePinnedToCore(
  //      sonar1_loop, /* Function to implement the task */
  //      "sonar1_loop2", /* Name of the task */
  //      2048, /* Stack size in words */
  //      (void *)&s3, /* Task input parameter */
  //      1, /* Priority of the task */
  //      &Task2, /* Task handle. */
  //      1); /* Core where the task should run */

  // xTaskCreatePinnedToCore(
  //     sonar1_loop_all,   /* Function to implement the task */
  //     "sonar1_loop_mid", /* Name of the task */
  //     2048,              /* Stack size in words */
  //     NULL,              /* Task input parameter */
  //     0,                 /* Priority of the task */
  //     &Task4,            /* Task handle. */
  //     1);                /* Core where the task should run */

  // xTaskCreatePinnedToCore(
  //     sonar_servo_loop_l,          /* Function to implement the task */
  //     "sonar_servo_loop1",       /* Name of the task */
  //     1024,                      /* Stack size in words */
  //     NULL, /* Task input parameter */
  //     0,                         /* Priority of the task */
  //     &servo_sonar_task0,        /* Task handle. */
  //     1);                        /* Core where the task should run */

  // xTaskCreatePinnedToCore(
  //     sonar_servo_loop_r,          /* Function to implement the task */
  //     "sonar_servo_loop2",       /* Name of the task */
  //     1024,                      /* Stack size in words */
  //     NULL, /* Task input parameter */
  //     0,                         /* Priority of the task */
  //     &servo_sonar_task1,        /* Task handle. */
  //     1);                        /* Core where the task should run */

  // xTaskCreatePinnedToCore(
  //     control_servo_loop,          /* Function to implement the task */
  //     "control_servo_loop",       /* Name of the task */
  //     1024,                      /* Stack size in words */
  //     NULL, /* Task input parameter */
  //     0,                         /* Priority of the task */
  //     &control_servo_task,        /* Task handle. */
  //     1);                        /* Core where the task should run */

  //  xTaskCreatePinnedToCore(
  //      sonar_servo_loop, /* Function to implement the task */
  //      "sonar_servo_loop2", /* Name of the task */
  //      2048, /* Stack size in words */
  //      (void *)&right_servo_sonar, /* Task input parameter */
  //      1, /* Priority of the task */
  //      &servo_sonar_task1, /* Task handle. */
  //      1); /* Core where the task should run */

  delay(1000); // get settled in

  // xTaskCreatePinnedToCore(
  //     esp_now_loop,       /* Function to implement the task */
  //     "esp_now_loop",     /* Name of the task */
  //     2048 * 2,           /* Stack size in words */
  //     NULL,               /* Task input parameter */
  //     0,                  /* Priority of the task */
  //     &esp_now_loop_task, /* Task handle. */
  //     1);                 /* Core where the task should run */
  //                         //

  // xTaskCreatePinnedToCore(
  //     main_loop,      /* Function to implement the task */
  //     "main_loop",    /* Name of the task */
  //     4096,           /* Stack size in words */
  //     NULL,           /* Task input parameter */
  //     0,              /* Priority of the task */
  //     &main_loop_tsk, /* Task handle. */
  //     1);             /* Core where the task should run */
}

void loop()
{
  // put your main code here, to run repeatedly:
  // vTaskDelete(NULL);
  // lf, rf, mid = 0,2,4
  int front_sonars[3] = {0, 2, 4};
  int slid = 1;
  int srid = 3;
  // float r1, r2, r3;
  for (int i = 0; i < 2; i++)
  {
    // r1 = sonars_all[front_sonars[i]].get_distance();

    myData.distances[sonar_indices[front_sonars[i]]] = 0.7 * sonars_all[front_sonars[i]].get_distance1() + 0.3 * myData.distances[sonar_indices[front_sonars[i]]];
    delay(5);
  }

  myData.distances[sonar_indices[slid]] = 0.7 * sonars_all[slid].get_distance1() + 0.3 * myData.distances[sonar_indices[slid]];
  delay(5);
  myData.distances[sonar_indices[srid]] = 0.7 * sonars_all[srid].get_distance1() + 0.3 * myData.distances[sonar_indices[srid]];
  delay(5);
  display.clearDisplay();
  display.setCursor(0, 5);
  // Display distance in cm
  for (int i = 0; i < 10; i++)
  {
    display.setCursor((int)(i % 3) * 41 + 5, (int)(floor((float)i / 3.0) * 22));
    display.print(myData.distances[i], 0);
  }

  display.display();
  if (true || (time_to_esp_now < (millis() - last_time_esp_now)))
  {
    last_time_esp_now = millis();
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  }
}