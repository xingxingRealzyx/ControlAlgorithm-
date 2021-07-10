#include "pid_controller.h"

static pid_controller_t *pid_controller = NULL;

static void pid_set_parameters(pid_controller_t *this, float Kp, float Ki, float Kd, float cycle)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

static void pid_get_parameters(pid_controller_t *this, float *Kp, float *Ki, float *Kd, float *cycle)
{
    *Kp = this->Kp;
    *Ki = this->Ki;
    *Kd = this->Kd;
}

static void pid_set_goal_value(pid_controller_t *this, float goal_value)
{
    this->goal_value = goal_value;
    
}

static float pid_get_goal_value(pid_controller_t *this)
{
    return this->goal_value;
}

static void pid_set_now_value(pid_controller_t *this, float now_value)
{
    this->now_value = now_value;
}

static float pid_get_now_value(pid_controller_t *this)
{
    return this->now_value;
}

static uint8_t pid_is_updata(pid_controller_t *this)
{
    return this->updata_flags;
}

static void pid_run(pid_controller_t *this)
{
    this->updata_flags   = 0;

    this->error = this->goal_value - this->now_value;

    /* pid controller */
    float delta = this->Kp * (this->error - this->error_1) + 
                  this->Ki * this->error + 
                  this->Kd * (this->error - 2 * this->error_1 + this->error_2);
    
    this->ctrl_value += delta;

    this->error_2 = this->error_1;
    this->error_1 = this->error;

    this->updata_flags   = 1;
}

static float pid_get_ctrl_value(pid_controller_t *this)
{
    return this->ctrl_value;
}

pid_controller_t* new_pid_controller()
{
    pid_controller = malloc(sizeof(pid_controller_t));
    if (pid_controller == NULL)
    {
        return NULL;
    }
    
    pid_controller->Kp             = DEFAULT_KP;
    pid_controller->Ki             = DEFAULT_KI;
    pid_controller->Kd             = DEFAULT_KD;
    pid_controller->cycle          = DEFAULT_CYCLE;

    pid_controller->goal_value     = 0.0;
    pid_controller->ctrl_value     = 0.0;
    pid_controller->now_value      = 0.0;
    pid_controller->error          = 0.0;
    pid_controller->error_1        = 0.0;
    pid_controller->error_2        = 0.0;

    pid_controller->updata_flags   = 0;

    pid_controller->set_parameters = pid_set_parameters;
    pid_controller->get_parameters = pid_get_parameters;
    pid_controller->set_goal_value = pid_set_goal_value;
    pid_controller->get_goal_value = pid_get_goal_value;
    pid_controller->get_now_value  = pid_get_now_value;
    pid_controller->set_now_value  = pid_set_now_value;
    pid_controller->is_updata      = pid_is_updata;
    pid_controller->run            = pid_run;
    pid_controller->get_ctrl_value = pid_get_ctrl_value;

    return pid_controller;
}

// test

/* system:　y = 2 * x + 1*/
// static float control_object(float x)
// {
//     return x * x * 2.0 + 4.0 * x + 10.0;
// }

// int main()
// {
//     pid_controller_t *pid = new_pid_controller();

//     pid->Kp = 0.01;
//     pid->Ki = 0.05;
//     pid->Kd = 0.00001;
//     pid->cycle = 10000.0;

//     while(1)
//     {
//         pid->set_goal_value(pid, 100.0);
//         pid->run(pid);
//         if(pid->is_updata(pid))
//         {
//             pid->set_now_value(pid, control_object(pid->ctrl_value));
//             printf("goal_value = %f ctrl_value = %f now_value = %f\r\n", pid->get_goal_value(pid), pid->get_ctrl_value(pid), pid->get_now_value(pid));
//             if (fabs(pid->get_goal_value(pid) - pid->get_now_value(pid)) < 0.001)
//             {
//                 break;
//             }
            
//         }
//        usleep((int)pid->cycle);
//     }

//     return 0;
// }
