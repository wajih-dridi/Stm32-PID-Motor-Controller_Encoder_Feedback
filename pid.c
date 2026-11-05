#include "pid.h"


/*
 * Limit a value between minimum and maximum.
 */
static float clamp(float value, float min, float max)
{
    if (value > max)
    {
        return max;
    }

    if (value < min)
    {
        return min;
    }

    return value;
}


/*
 * Initialize the PID controller.
 */
void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integral = 0.0f;
    pid->previous_error = 0.0f;

    pid->output_min = output_min;
    pid->output_max = output_max;
}


/*
 * Execute one PID calculation.
 *
 * setpoint    = desired value
 * measurement = actual measured value
 * dt          = control-loop period in seconds
 */
float PID_Update(PID_t *pid,
                 float setpoint,
                 float measurement,
                 float dt)
{
    float error;
    float derivative;
    float integral_candidate;
    float output;


    /*
     * Safety check.
     */
    if (dt <= 0.0f)
    {
        return 0.0f;
    }


    /*
     * Calculate error.
     *
     * Error = Desired speed - Actual speed
     */
    error = setpoint - measurement;


    /*
     * Calculate derivative.
     */
    derivative = (error - pid->previous_error) / dt;


    /*
     * Calculate a possible new integral.
     */
    integral_candidate =
        pid->integral + (error * dt);


    /*
     * Calculate preliminary PID output.
     */
    output =
        (pid->kp * error) +
        (pid->ki * integral_candidate) +
        (pid->kd * derivative);


    /*
     * Anti-windup.
     *
     * If the output is saturated, don't continue
     * integrating in the direction that makes
     * saturation worse.
     */
    if (output > pid->output_max)
    {
        output = pid->output_max;

        if (error < 0.0f)
        {
            pid->integral = integral_candidate;
        }
    }
    else if (output < pid->output_min)
    {
        output = pid->output_min;

        if (error > 0.0f)
        {
            pid->integral = integral_candidate;
        }
    }
    else
    {
        /*
         * Output is inside the allowed range,
         * therefore accept the new integral.
         */
        pid->integral = integral_candidate;
    }


    /*
     * Recalculate the output using the accepted
     * integral value.
     */
    output =
        (pid->kp * error) +
        (pid->ki * pid->integral) +
        (pid->kd * derivative);


    /*
     * Final output limitation.
     */
    output = clamp(output,
                   pid->output_min,
                   pid->output_max);


    /*
     * Save current error for the next iteration.
     */
    pid->previous_error = error;


    return output;
}


/*
 * Reset PID internal state.
 */
void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
}
