#ifndef GOAL_SEEK_SUFF_H
#define GOAL_SEEK_SUFF_H

#include "motor_ctrl_stuff.h"

QueueHandle_t send_dist_to_task;
TaskHandle_t robot_move_dist_task;
TaskHandle_t update_robot_pos_task;

JSONVar motor_status_direction;

bool goal_task_running = false;

void init_msd()
{
  motor_status_direction["forward"] = ((int)0);
  motor_status_direction["back"] = ((int)4);
  motor_status_direction["left"] = ((int)-2);
  motor_status_direction["right"] = ((int)2);
  motor_status_direction["forwardleft"] = ((int)-1);
  motor_status_direction["backleft"] = ((int)-3);
  motor_status_direction["forwardright"] = ((int)1);
  motor_status_direction["backright"] = ((int)3);
  motor_status_direction["stop"] = ((int)stp);
  motor_status_direction["e_stop"] = ((int)e_stop);
  motor_status_direction["e_stop_clear"] = ((int)e_stop_clear);
}

// typedef struct pos
// {
//     float x;
//     float y;
//     float theta;
// } pos;

// pos robot_pos;
// pos target_pos;

// potential fields components

float alpha = 10.0;
float beta = 1.0;

bool at_goal(pos p1, pos p2, float close = 4.0)
{
    return (abs(p1.y - p2.y) < close && abs(p1.x - p2.x) < close);
}

double constrainAngle(double x){
    x = fmod(x + pi,2*pi);
    if (x < 0)
        x += 2*pi;
    return x - pi;
}

void robot_turn_theta(float thet)
{

    // read command.
    //  theta = ypr[0] + theta;
    state old_state = actstate;
    state new_state = (thet<0?rgt:lft);
    Serial.println("new states = " + String(new_state)+", theta = "+String(thet));
    encoder0Pos = 0;
    float d2 = abs(((float)encoder0Pos)) * pulse_dist;
    robot_set_and_send_command(new_state);
    while (d2 < turning_circle * abs(thet) / (2.0 * pi))
    {
        d2 = abs(((float)encoder0Pos)) * pulse_dist;
    }
    encoder0Pos = 0;
    robot_set_and_send_command(old_state);
}

void robot_move_dist_loop(void *params){
    //move a certain distance
    for(;;){
        float dist;
        // float d1;
        xQueueReceive(send_dist_to_task, &dist, portMAX_DELAY); //TODO

        long encoderpos_tmp = (long) encoder0Pos;
        state old_state = actstate;
        state mov_state = (dist<0?rev:fwd);
        robot_set_and_send_command(mov_state);
        while(abs(((float)(encoder0Pos-encoderpos_tmp)) * pulse_dist) < abs(dist)){
            vTaskDelay(50/portTICK_PERIOD_MS);
        }
        robot_set_and_send_command(old_state);
    }
}

void update_robot_pos_loop(void *params)
{
    

    for (;;)
    {
        vTaskDelay(150 / portTICK_PERIOD_MS);
        Serial.println("in goal");
        Serial.print("status = ");
        Serial.println(actstate);
        Serial.print("robot post: ");
        Serial.println(String(robot_pos.theta)+", "+String(robot_pos.x)+", "+String(robot_pos.y));
        Serial.print("target post: ");
        Serial.println(String(target_pos.theta)+", "+String(target_pos.x)+", "+String(target_pos.y));
        data_struct_rcv dsr;
        float min_dist = 500.0;
        int min_i = -1;

        if (at_goal(robot_pos, target_pos)){
            Serial.println("at goal");
            robot_turn_theta((constrainAngle(target_pos.theta)));
            robot_set_and_send_command(stp);
            xSemaphoreTake(Semaphore_prev_time, portMAX_DELAY);
            goal_task_running = false;
            xSemaphoreGive(Semaphore_prev_time);
            vTaskDelay(5/portTICK_PERIOD_MS);
            vTaskDelete(NULL);
            // notifyClients();
        }

        // xQueueReceive(check_sonar_queue, &dsr, portMAX_DELAY);

        float d1 = (((float)encoder0Pos)) * pulse_dist;
        float d2 = (((float)encoder1Pos)) * pulse_dist;

        robot_pos.x += d2 * cos(robot_pos.theta);
        robot_pos.y += d2 * sin(robot_pos.theta);
        encoder0Pos = 0;
        encoder1Pos = 0;

        target_pos.theta = atan2(target_pos.y - robot_pos.y, target_pos.x - robot_pos.x) - robot_pos.theta; // theta is direction robot is facing.
        target_pos.theta = constrainAngle(target_pos.theta);
        for (int i = 0; i < 9; i++)
        {
            if (myData_rcv.distances[i] < min_dist && myData_rcv.distances[i] > 2.0 && i != 4)
            {
                min_dist = myData_rcv.distances[i];
                min_i = i;
            }
        }
        if (min_dist < 20.0){
            //we need to change direction
            //find min between left an right.
            //we add a little bit of object avoidance to object tracking.
            float min = alpha/(min_dist - beta);
            if (min > 2.0) min = 2.0;
            w1 = (min_i<4?-1.0:1.0)*min; //right turn is negtive.
                                                            //left turn is positive
            Serial.println("w1 = "+String(w1));

            delta_theta = (1.0*target_pos.theta+w1*pi/4)/(1.0+abs(w1));
            delta_theta = constrainAngle(delta_theta);

        }

        else{
            delta_theta =(constrainAngle(target_pos.theta));// 
        }

        if (abs(delta_theta) > pi/16){
            robot_pos.theta += delta_theta;
            robot_pos.theta = (constrainAngle(robot_pos.theta)); 
            if (actstate == fwd) robot_turn_theta((constrainAngle(delta_theta)));
        }
    }
}

void check_sonar_loop(void *parameter)
{
//can't run at the same time as the function above.
    for (;;)
    {
        data_struct_rcv dsr;
        uint8_t rv = 1;
        float min_dist = 500.0;
        int min_i = -1;
        // delay(1);

        xQueueReceive(check_sonar_queue, &dsr, portMAX_DELAY);

        for (int i = 0; i < 9; i++)
        {
            if (dsr.distances[i] < min_dist && dsr.distances[i] > 2.0 && i!=4)
            {
                min_dist = dsr.distances[i];
                min_i = i;
            }
        }

        if (min_dist < 6.0)
        {
            Serial.println("min dist less than 6");
            //robot_set_and_send_command(stp);
            // notifyClients();
        }
        // Serial.println("first order recieved: " + String((int)st));
        // if ((int)st < (int)stp)
        // {
        //     // Serial.println("order recieved");

        //     digitalWrite(LEFT_MTR_DIR, mtr_lft_state[(int)st]);
        //     digitalWrite(RIGHT_MTR_DIR, mtr_rgt_state[(int)st]);
        //     robot_set_speed();
        // }
        // else
        // {
        //     robot_stop();
        // }

        // xQueueSend(queue_ret, &rv, message_time);
        // if(st==e_stop){
        //     robot_stop();
        //     vTaskDelete(NULL);
        // }
    }
}

#endif