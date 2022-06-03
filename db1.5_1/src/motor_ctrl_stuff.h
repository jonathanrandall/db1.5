#ifndef MOTOR_CTRL_STUFF
#define MOTOR_CTRL_STUFF

#include <Arduino.h>

TaskHandle_t robot_move;

QueueHandle_t queue;
QueueHandle_t queue_ret;

UBaseType_t n_messages;

#define LEFT_MTR_DIR 26
#define LEFT_MTR_PWM 27
#define RIGHT_MTR_DIR 12
#define RIGHT_MTR_PWM 32
#define LEFT_FWD HIGH
#define RIGHT_FWD LOW
#define LEFT_BACK LOW
#define RIGHT_BACK HIGH

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
state actstate = stp;
String str_status;
String status_names[11] = {"forward", "back", "left", "right", "forward_right", "forward_left","back_right", 
                            "back_left", "stop", "e_stop", "e_stop_clear"};

state st1 = stp;

// Placeholder for functions
void robot_setup();
void robot_stop();
void robot_fwd();
void robot_back();
void robot_left();
void robot_right();
uint8_t robo = 0;

int speed = 255;
int noStop = 0;
int motorstatus = 0;

const int freq = 4000;
const int mtr_left_pwm_channel = 8;
const int mtr_right_pwm_channel = 4;
const int lresolution = 8;
unsigned long max_motor_speed = 255;

volatile int motor_speed = 100;
volatile unsigned long previous_time = 0;
volatile unsigned long move_interval = 350;
volatile unsigned long message_time = 100;

int mtr_lft_state[] = {LEFT_FWD, LEFT_BACK, LEFT_BACK, LEFT_FWD,LEFT_FWD,LEFT_FWD,LEFT_BACK,LEFT_BACK}; // fwd, rev, lft, right
int mtr_rgt_state[] = {RIGHT_FWD, RIGHT_BACK, RIGHT_FWD, RIGHT_BACK,RIGHT_FWD,RIGHT_FWD,RIGHT_BACK,RIGHT_BACK};

// void robot_set_speed_lr(int ls = motor_speed, int rs = motor_speed);

void robot_stop()
{
    ledcWrite(mtr_right_pwm_channel, 0);
    ledcWrite(mtr_left_pwm_channel, 0);
    // Serial.println("fobot stop");
    if ((int)actstate < (int)stp)
        actstate = stp;
}

void robot_set_speed_lr(int ls = motor_speed, int rs = motor_speed)
{
    if ((int)actstate < (int)stp)
    {
        ledcWrite(mtr_right_pwm_channel, rs);
        ledcWrite(mtr_left_pwm_channel, ls);
    }
}

void robot_set_speed(int ms = motor_speed)
{
    // Serial.print("inside robot set speed: ");
    // Serial.println((int) actstate);
    if ((int)actstate < (int)stp)
    {
        if(((int)actstate>(int)rgt)){
            int ls;
            int rs;
            if(actstate == fwdlft || actstate == revlft){
                ls = min(30,ms-30);
                rs = max(255, ms+30);
            } else {
                rs = min(30,ms-30);
                ls = max(255, ms+30);
            }
            robot_set_speed_lr(ls, rs);
        } else {
            ledcWrite(mtr_right_pwm_channel, ms);
            ledcWrite(mtr_left_pwm_channel, ms);
        }
        
    }
}


//set left and right seperately.


unsigned long robot_set_and_send_command(state st)
{
    // check emergency stop
    previous_time = millis();
    // Serial.println("robot set and send");
    // Serial.println(st);
    // Serial.println(actstate);

    uint8_t ret_val = 0;

    if (st == e_stop)
    {
        robot_stop();
        return 3;
    }

    if (st == actstate)
    {
        return 2; // don't need to set it if already set
        // Serial.println("no need to send");
    }
    else
    {
        actstate = st;
        // Serial.println("actstate update" + String(actstate));
        // n_messages = uxQueueMessagesWaiting( queue );
        // Serial.println("putting message in queu");
        if (xQueueSend(queue, &st, message_time))
        {
            Serial.println("sending");
        }                                                 // portMAX_DELAY);
        xQueueReceive(queue_ret, &ret_val, message_time); // hand shake
    }
    // previous_time = millis(); // not sure if this line should be here
    if (ret_val)
        return 1;
    return 0;
}

void robot_move_loop(void *parameter)
{

    for (;;)
    {
        state st;
        uint8_t rv = 1;
        // delay(1);
        
        xQueueReceive(queue, &st, portMAX_DELAY);
        // Serial.println("first order recieved: " + String((int)st));
        if ((int)st < (int)stp)
        {
            // Serial.println("order recieved");

            digitalWrite(LEFT_MTR_DIR, mtr_lft_state[(int)st]);
            digitalWrite(RIGHT_MTR_DIR, mtr_rgt_state[(int)st]);
            robot_set_speed();
        }
        else
        {
            robot_stop();
        }

        xQueueSend(queue_ret, &rv, message_time);
        // if(st==e_stop){
        //     robot_stop();
        //     vTaskDelete(NULL);
        // }
    }
}

void robot_setup()
{
    // Pins for Motor Controller
    pinMode(LEFT_MTR_DIR, OUTPUT);
    pinMode(LEFT_MTR_PWM, OUTPUT);
    pinMode(RIGHT_MTR_DIR, OUTPUT);
    pinMode(RIGHT_MTR_PWM, OUTPUT);

    queue = xQueueCreate(1, sizeof(state));
    queue_ret = xQueueCreate(1, sizeof(uint8_t));
    // Make sure we are stopped
    // robot_stop();

    // Motor uses PWM Channel 8
    ledcAttachPin(LEFT_MTR_PWM, mtr_left_pwm_channel);
    ledcSetup(mtr_left_pwm_channel, freq, lresolution);
    ledcWrite(mtr_left_pwm_channel, 0);

    ledcAttachPin(RIGHT_MTR_PWM, mtr_right_pwm_channel);
    ledcSetup(mtr_right_pwm_channel, freq, lresolution);
    ledcWrite(mtr_right_pwm_channel, 0);

    xTaskCreatePinnedToCore(
        robot_move_loop, /* Function to implement the task */
        "robot_move",    /* Name of the task */
        4000,            /* Stack size in words */
        NULL,            /* Task input parameter */
        0,               /* Priority of the task */
        &robot_move,     /* Task handle. */
        1);              /* Core where the task should run */
}

void robot_fwd()
{
    if (actstate == e_stop || actstate == stp)
    {
        robot_stop();
        return;
    }
    if (actstate != fwd)
    {
        robot_set_and_send_command(fwd);
    }
}

void robot_back()
{
    if (actstate == e_stop || actstate == stp)
    {
        robot_stop();
        return;
    }
    if (actstate != rev)
    {
        robot_set_and_send_command(rev);
    }
}

void robot_left()
{
    if (actstate == e_stop || actstate == stp)
    {
        robot_stop();
        return;
    }
    if (actstate != lft)
    {
        robot_set_and_send_command(lft);
    }
}

void robot_right()
{
    if (actstate == e_stop || actstate == stp)
    {
        robot_stop();
        return;
    }
    if (actstate != rgt)
    {
        robot_set_and_send_command(rgt);
    }
}

// general move command
void robot_move_(state st)
{
    if (actstate == e_stop)
    {
        robot_stop();
        return;
    }
    if (actstate != st)
    {
        actstate = st;
        digitalWrite(LEFT_MTR_DIR, mtr_lft_state[(int)st]);
        digitalWrite(RIGHT_MTR_DIR, mtr_rgt_state[(int)st]);
        robot_set_speed(true);
    }
}

#endif
