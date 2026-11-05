#include "motor_control.h"


/*
 * IMPORTANT:
 *
 * Change this value according to your encoder.
 *
 * Here we assume:
 *
 * 100 pulses/revolution
 * × 4 quadrature counting
 *
 * = 400 counts/revolution
 */
#define ENCODER_COUNTS_PER_REV 400.0f


/*
 * Convert encoder count difference into RPM.
 */
static float Encoder_To_RPM(int16_t delta,
                            float dt)
{
    float revolutions;
    float revolutions_per_second;
    float rpm;


    /*
     * Convert encoder counts to revolutions.
     */
    revolutions =
        (float)delta / ENCODER_COUNTS_PER_REV;


    /*
     * Convert revolutions during the
     * sampling period into revolutions/second.
     */
    revolutions_per_second =
        revolutions / dt;


    /*
     * Convert revolutions/second to RPM.
     */
    rpm =
        revolutions_per_second * 60.0f;


    return rpm;
}


/*
 * Initialize the complete motor controller.
 */
void MotorController_Init(MotorController_t *motor)
{
    /*
     * Initial PID gains.
     *
     * These are starting values only.
     * They will need tuning on the real motor.
     */
    PID_Init(&motor->pid,
             0.8f,       /* Kp */
             0.2f,       /* Ki */
             0.02f,      /* Kd */
             0.0f,       /* Minimum output */
             100.0f);    /* Maximum output */


    /*
     * Initial values.
     */
    motor->target_rpm = 0.0f;

    motor->actual_rpm = 0.0f;

    motor->pwm_percent = 0.0f;

    motor->previous_encoder = 0;
}


/*
 * Set the desired motor speed.
 */
void MotorController_SetTarget(MotorController_t *motor,
                                float rpm)
{
    /*
     * Don't allow negative speed in this
     * first version.
     */
    if (rpm < 0.0f)
    {
        rpm = 0.0f;
    }


    /*
     * Limit maximum requested speed.
     */
    if (rpm > 3000.0f)
    {
        rpm = 3000.0f;
    }


    motor->target_rpm = rpm;
}


/*
 * Execute one control-loop iteration.
 */
void MotorController_Update(MotorController_t *motor,
                            uint16_t encoder_count,
                            float dt)
{
    int16_t delta;


    /*
     * Calculate the number of encoder counts
     * since the previous control-loop iteration.
     *
     * The cast to int16_t also allows the calculation
     * to handle normal 16-bit counter wrap-around.
     */
    delta =
        (int16_t)(encoder_count -
                  motor->previous_encoder);


    /*
     * Save current encoder count for next iteration.
     */
    motor->previous_encoder =
        encoder_count;


    /*
     * Convert encoder movement into RPM.
     */
    motor->actual_rpm =
        Encoder_To_RPM(delta, dt);


    /*
     * Run PID controller.
     *
     * target_rpm = desired speed
     * actual_rpm = measured speed
     */
    motor->pwm_percent =
        PID_Update(&motor->pid,
                   motor->target_rpm,
                   motor->actual_rpm,
                   dt);
}
