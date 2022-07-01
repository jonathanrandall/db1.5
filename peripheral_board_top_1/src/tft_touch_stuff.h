#ifndef TFT_TOUCH_STUFF_
#define TFT_TOUCH_STUFF_

/******
 *
 *  Watch https://www.youtube.com/watch?v=rq5yPJbX_uk&t=714s for instructions on how to use this library.
 * Don't forget to change the following in the User_Setup.h file of the TFT_espi library.
#define TFT_MISO 19 green
#define TFT_MOSI 23  white
#define TFT_SCLK 18 orange
#define TFT_CS   15  // Chip select control pin yellow
#define TFT_DC    2  // Data Command control pin //blue
#define TFT_RST   4  // Reset pin (could connect to RST pin) purple
//#define TFT_RST  -1  // Set TFT_RST to -1 if display RESET is connected to ESP32 board RST

#define TOUCH_CS 21     // Chip select pin (T_CS) of touch screen grey

//TDO green to MISO 19
//TDIN white to 23
*/

#include "FS.h"
#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>

TFT_eSPI tft = TFT_eSPI(); // Invoke custom library

// This is the file name used to store the touch coordinate
// calibration data. Change the name to start a new calibration.
#define CALIBRATION_FILE "/TouchCalData3"

// Set REPEAT_CAL to true instead of false to run calibration
// again, otherwise it will only be done once.
// Repeat calibration if you change the screen rotation.
#define REPEAT_CAL false

bool SwitchOn = false;

// Comment out to stop drawing black spots
#define BLACK_SPOT

// Switch position and size
#define FRAME_X 100
#define FRAME_Y 50
#define FRAME_W 120
#define FRAME_H 60

// Red zone size
#define REDBUTTON_X FRAME_X
#define REDBUTTON_Y FRAME_Y
#define REDBUTTON_W (FRAME_W/2)
#define REDBUTTON_H FRAME_H

// Green zone size
#define GREENBUTTON_X (REDBUTTON_X + REDBUTTON_W)
#define GREENBUTTON_Y FRAME_Y
#define GREENBUTTON_W (FRAME_W/2)
#define GREENBUTTON_H FRAME_H

void touch_calibrate()
{
    uint16_t calData[5];
    uint8_t calDataOK = 0;

    // check file system exists
    if (!SPIFFS.begin())
    {
        Serial.println("Formatting file system");
        SPIFFS.format();
        SPIFFS.begin();
    }

    // check if calibration file exists and size is correct
    if (SPIFFS.exists(CALIBRATION_FILE))
    {
        if (REPEAT_CAL)
        {
            // Delete if we want to re-calibrate
            SPIFFS.remove(CALIBRATION_FILE);
        }
        else
        {
            File f = SPIFFS.open(CALIBRATION_FILE, "r");
            if (f)
            {
                if (f.readBytes((char *)calData, 14) == 14)
                    calDataOK = 1;
                f.close();
            }
        }
    }

    if (calDataOK && !REPEAT_CAL)
    {
        // calibration data valid
        tft.setTouch(calData);
    }
    else
    {
        // data not valid so recalibrate
        tft.fillScreen(TFT_BLACK);
        tft.setCursor(20, 0);
        tft.setTextFont(2);
        tft.setTextSize(1);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);

        tft.println("Touch corners as indicated");

        tft.setTextFont(1);
        tft.println();

        if (REPEAT_CAL)
        {
            tft.setTextColor(TFT_RED, TFT_BLACK);
            tft.println("Set REPEAT_CAL to false to stop this running again!");
        }

        tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.println("Calibration complete!");

        // store data
        File f = SPIFFS.open(CALIBRATION_FILE, "w");
        if (f)
        {
            f.write((const unsigned char *)calData, 14);
            f.close();
        }
    }
}

void drawFrame()
{
    tft.drawRect(FRAME_X, FRAME_Y, FRAME_W, FRAME_H, TFT_BLACK);
}

// Draw a red button
void redBtn()
{
    tft.fillRect(REDBUTTON_X, REDBUTTON_Y, REDBUTTON_W, REDBUTTON_H, TFT_RED);
    tft.fillRect(GREENBUTTON_X, GREENBUTTON_Y, GREENBUTTON_W, GREENBUTTON_H, TFT_DARKGREY);
    drawFrame();
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("START", GREENBUTTON_X + (GREENBUTTON_W / 2), GREENBUTTON_Y + (GREENBUTTON_H / 2));
    tft.drawString("STOP", REDBUTTON_X + (REDBUTTON_W / 2) + 1, REDBUTTON_Y + (REDBUTTON_H / 2));
    SwitchOn = true;
    
}

// Draw a green button
void greenBtn()
{
    tft.fillRect(GREENBUTTON_X, GREENBUTTON_Y, GREENBUTTON_W, GREENBUTTON_H, TFT_GREEN);
    tft.fillRect(REDBUTTON_X, REDBUTTON_Y, REDBUTTON_W, REDBUTTON_H, TFT_DARKGREY);
    drawFrame();
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("STOP", REDBUTTON_X + (REDBUTTON_W / 2) + 1, REDBUTTON_Y + (REDBUTTON_H / 2));
     tft.drawString("START", GREENBUTTON_X + (GREENBUTTON_W / 2), GREENBUTTON_Y + (GREENBUTTON_H / 2));
    SwitchOn = false;
}

void init_tft()
{
    tft.init();

    // Set the rotation before we calibrate
    tft.setRotation(1);

    // call screen calibration
    touch_calibrate();

    // clear screen
    tft.fillScreen(TFT_BLACK);
}

void monitor_button_old(void *parameter)
{
    uint16_t x, y;

    for (;;)
    {
        vTaskDelay(300/portTICK_PERIOD_MS);
        Serial.println("in this functions");
        if (tft.getTouch(&x, &y))
        {
            Serial.println("Red btn hitfds");


            if (!SwitchOn)
            {
                if ((x > REDBUTTON_X) && (x < (REDBUTTON_X + REDBUTTON_W)))
                {
                    if ((y > REDBUTTON_Y) && (y <= (REDBUTTON_Y + REDBUTTON_H)))
                    {
                        Serial.println("Red btn hit");
                        redBtn();
                        x=0;
                        vTaskDelay(300/portTICK_PERIOD_MS);
                        
                    }
                }
            }
            else // Record is off (SwitchOn == false)
            {
                if ((x > GREENBUTTON_X) && (x < (GREENBUTTON_X + GREENBUTTON_W)))
                {
                    if ((y > GREENBUTTON_Y) && (y <= (GREENBUTTON_Y + GREENBUTTON_H)))
                    {
                        Serial.println("Green btn hit");
                        greenBtn();
                        y=0;
                        vTaskDelay(300/portTICK_PERIOD_MS);
                    }
                }
            }

            // Serial.println(SwitchOn);
        }
    }
}

#endif