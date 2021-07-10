#ifndef __PID_CONTROLER_H
#define __PID_CONTROLER_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define DEFAULT_KP 1.0f
#define DEFAULT_KI 0.0f
#define DEFAULT_KD 0.0f
#define DEFAULT_CYCLE 10.0f //MS


typedef struct pid_controller_t pid_controller_t;

struct pid_controller_t {
    float Kp;
    float Ki;
    float Kd;
    float cycle;
    float goal_value;
    float ctrl_value;
    float now_value;

    float error;
    float error_1;
    float error_2;


    uint8_t updata_flags;

    void (*set_parameters)(pid_controller_t *this, float Kp, float Ki, float Kd, float cycle);
    void (*get_parameters)(pid_controller_t *this, float *Kp, float *Ki, float *Kd, float *cycle);

    void (*set_goal_value)(pid_controller_t *this, float goal_value);
    float (*get_goal_value)(pid_controller_t *this);

    float (*get_now_value)(pid_controller_t *this);
    void (*set_now_value)(pid_controller_t *this, float now_value);

    uint8_t (*is_updata)(pid_controller_t *this);

    void (*run)(pid_controller_t *this);

    float (*get_ctrl_value)(pid_controller_t *this);
};

pid_controller_t* new_pid_controller();

#endif