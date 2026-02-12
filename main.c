#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>
#include <soc.h>
#include <zephyr/console/console.h>
#include <stdint.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <stdio.h>
#include <string.h>

#define ADC_DEV_NODE DT_ALIAS(adc0)

#define ADC_CHANNEL_A 3  /* PC2 U phase */
#define ADC_CHANNEL_B 5  /* PA0 V phase */
#define ADC_CHANNEL_C 6  /* PA1 W phase */
#define ADC_RESOLUTION 12U
#define ADC_REF_MV 3300U
#define SENSOR_THRESHOLD_MV 1900
#define ADC_BUFFER_SIZE 1

/* Motor electrical properties */
#define POLE_PAIRS       8

// thread parameters
#define STACK_USER_INPUT 1024
#define PRIO_USER_INPUT  6

#define STACK_PID        1024
#define PRIO_PID         5

#define STACK_PWM        1024
#define PRIO_PWM         4

#define STACK_HALL_MON   1024
#define PRIO_HALL_MON    3

/* Globals near top */
static int rpm_target = 0;
static int measured_rpm = 0;
static int prev_state = 0;
static int counter_clockwise = 0;
static struct k_mutex rpm_lock;

#if DT_NODE_HAS_STATUS(ADC_DEV_NODE, okay)
static const struct device *adc_dev = DEVICE_DT_GET(ADC_DEV_NODE);
#else
static const struct device *adc_dev = NULL;
#endif

static int16_t adc_buffer[ADC_BUFFER_SIZE];

static inline int32_t adc_raw_to_mv(int32_t raw)
{
    int32_t max_raw = (1 << ADC_RESOLUTION) - 1;
    return (raw * (int32_t)ADC_REF_MV) / max_raw;
}

static int read_adc_mv(uint8_t channel, int32_t *out_mv)
{
    if (!adc_dev || !device_is_ready(adc_dev)) return -ENODEV;

    struct adc_channel_cfg ch_cfg = {
        .gain = ADC_GAIN_1,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id = channel,
    };

    int rc = adc_channel_setup(adc_dev, &ch_cfg);
    if (rc) return rc;

    const struct adc_sequence seq = {
        .channels = BIT(channel),
        .buffer = adc_buffer,
        .buffer_size = sizeof(adc_buffer),
        .resolution = ADC_RESOLUTION,
    };

    rc = adc_read(adc_dev, &seq);
    if (rc) return rc;

    int32_t raw = adc_buffer[0];
    *out_mv = adc_raw_to_mv(raw);
    return 0;
}

/* Convert voltage reading to digital sensor value (0 or 1) */
static inline int sensor_from_mv(int32_t mv)
{
    return (mv > SENSOR_THRESHOLD_MV) ? 1 : 0;
}

const struct device *pwm_dev = DEVICE_DT_GET(DT_NODELABEL(pwm1));

#define PERIOD_NS (50000)  // 1 / 20kHz = 50µs = 50000ns

void enable_tim1_complementary_outputs(void) {
    // Enable outputs on CH1–3 and CH1N–3N
    TIM1->BDTR |= TIM_BDTR_MOE; // enable main output
    //TIM1->CCER |= TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE; // enable CHxN
}

void set_commutation_step(uint8_t step)
{
    // Clear all outputs
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                    TIM_CCER_CC2E | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E | TIM_CCER_CC3NE);

    switch (step) {
    case 1: // W+ V-
        TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC2NE;
        break;//
    case 5: // U+ V-
        TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2NE;
        break;
    case 4: // U+ W-
        TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC3NE;
        break;
    case 6: // V+ W-
        TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC3NE;
        break;
    case 2: // V+ U-
        TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC1NE;
        break;
    case 3: // W+ U-
        TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC1NE;
        break;
    // +8 for counter clockwise
    case 9: // U+ V-
        TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2NE;
        break;
    case 13: // W+ V-
        TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC2NE;
        break;
    case 12: // W+ U-
        TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC1NE;
        break;
    case 14: // V+ U-
        TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC1NE;
        break;
    case 10: // V+ W-
        TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC3NE;
        break;
    case 11: // U+ W-
        TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC3NE;
        break;
    }
}

void all_pwm_set(int pulse){

    pwm_set(pwm_dev, 1, PERIOD_NS, pulse, 0); // channel 1
    pwm_set(pwm_dev, 2, PERIOD_NS, pulse, 0); // channel 2
    pwm_set(pwm_dev, 3, PERIOD_NS, pulse, 0); // channel 3

    enable_tim1_complementary_outputs();
}

int rpm_to_pulse(int rpm){
    // 326 min (3160)
    // 4744 max (47340)
    // printk("Recieved rpm = %d\r\n", rpm);
    int pulse =0;
    pulse = (rpm * 8.836)+3160;
    //printk("new pulse value = %d\r\n", pulse);
    return pulse;
}

typedef struct {
    float kp;
    float ki;
    float kd;
    float prev_error;
    float integral;
    float out_min;
    float out_max;
} pid_struct;

static pid_struct rpm_pid;

void pid_init(pid_struct *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->prev_error = 0.0f;
    pid->integral   = 0.0f;

    pid->out_min = out_min;
    pid->out_max = out_max;
}

float pid_compute(pid_struct *pid, float target, float measured, float dt)
{
    float error = target - measured;

    // Integral term
    pid->integral += error * pid->ki * dt;

    // Anti-windup
    if (pid->integral > pid->out_max) pid->integral = pid->out_max;
    if (pid->integral < pid->out_min) pid->integral = pid->out_min;

    // Derivative term
    float derivative = (error - pid->prev_error) / dt;

    // PID output
    float output = pid->kp * error + pid->integral + pid->kd * derivative;

    // Clamp to limits
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    pid->prev_error = error;

    return output;
}

int get_rpm_from_terminal(void)
{
    char buffer[16];
    int index = 0;
    int rpm = 0;

    printk("Enter desired RPM 0-5000 (digits only):\r\n");

    while (1) {
        int c = console_getchar();  // Blocking read from UART

        if (c == '\r') {
            buffer[index] = '\0';
            rpm = atoi(buffer);     // Convert string to int
            if (rpm < 0){
                rpm = 0;
            } else if (rpm > 5000){
                rpm = 5000;
            }
            printk("\r\nReceived RPM = %d\r\n", rpm);
            break;
        } else if (c >= '0' && c <= '9') {
            if (index < sizeof(buffer) - 1) {
                buffer[index++] = (char)c;
                console_putchar(c); // Echo back
            }
        } else if (c == 0x7F || c == '\b') { // Handle backspace
            if (index > 0) {
                index--;
                printk("\b \b");
            }
        }

        k_msleep(2); // Small delay to prevent input flooding
    }

    return rpm;
}

void pid_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    pid_init(&rpm_pid, 0.2, 0.05, 0.01, 0, 100);

    while (1) {
        int rpm_pid_val = 50 * pid_compute(&rpm_pid, rpm_target, measured_rpm, 0.1f);
        all_pwm_set(rpm_to_pulse(rpm_pid_val));
        set_commutation_step(prev_state);
        k_msleep(100);
    }
}

K_THREAD_STACK_DEFINE(pid_stack, STACK_PID);
static struct k_thread pid_tid;

void user_input_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    console_init();
    printk("BLDC Control Ready.\r\n");

    
    while (1) {
        int rpm = get_rpm_from_terminal();

        k_mutex_lock(&rpm_lock, K_FOREVER);
        if (rpm_target != rpm){
            rpm_target = rpm;
            all_pwm_set(rpm_to_pulse(rpm_target));
            set_commutation_step(prev_state);
        }
        k_mutex_unlock(&rpm_lock);

        printk("\r\nNew target RPM = %d\r\n", rpm_target);

        k_msleep(10);
    }
}

K_THREAD_STACK_DEFINE(user_stack, STACK_USER_INPUT);
static struct k_thread user_tid;

void pwm_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    printk("PWM thread started.\r\n");

    while (1) {
        int32_t mv;
        int U = 0, V = 0, W = 0;

        if (read_adc_mv(ADC_CHANNEL_A, &mv) == 0) U = sensor_from_mv(mv);
        if (read_adc_mv(ADC_CHANNEL_B, &mv) == 0) V = sensor_from_mv(mv);
        if (read_adc_mv(ADC_CHANNEL_C, &mv) == 0) W = sensor_from_mv(mv);

        uint8_t state = (counter_clockwise << 3) | (U << 2) | (V << 1) | W;

        if (state != prev_state){
            if (state == 0){
                set_commutation_step(prev_state);
            } else {
                k_mutex_lock(&rpm_lock, K_FOREVER);
                prev_state = state;
                k_mutex_unlock(&rpm_lock);

                set_commutation_step(state);
                printk("Sensors: C=%d U=%d V=%d W=%d\n", counter_clockwise, U, V, W);
            }
        }

        k_msleep(5);
    }

}

K_THREAD_STACK_DEFINE(pwm_stack, STACK_PWM);
static struct k_thread pwm_tid;

void hall_monitor_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    uint8_t last_state = 0xff;
    uint32_t last_timestamp_ms = k_uptime_get_32();
    uint32_t local_measured_rpm = 0;

    printk("Hall monitor thread started.\r\n");

    while (1) {
        int32_t mv_a = 0, mv_b = 0, mv_c = 0;
        int a = 0, b = 0, c = 0;

        if (read_adc_mv(ADC_CHANNEL_A, &mv_a) == 0) a = sensor_from_mv(mv_a);
        if (read_adc_mv(ADC_CHANNEL_B, &mv_b) == 0) b = sensor_from_mv(mv_b);
        if (read_adc_mv(ADC_CHANNEL_C, &mv_c) == 0) c = sensor_from_mv(mv_c);

        uint8_t state = (a << 2) | (b << 1) | c;

        uint32_t now = k_uptime_get_32();

        if (state != last_state) {
            /* compute time since last change */
            uint32_t dt = now - last_timestamp_ms;
            last_timestamp_ms = now;

            if (dt > 0) {
                /* dt is ms per state transition
                 * electrical RPM = 60000 / (dt_ms * transitions_per_elec_rev)
                 * transitions_per_elec_rev for 6-step = 6
                 */
                float erpm = 60000.0f / ((float)dt * 6.0f);
                /* mechanical RPM = electrical RPM / pole_pairs */
                local_measured_rpm = (uint32_t)(erpm / (float)POLE_PAIRS + 0.5f);
            }

            /* publish measured rpm safely */
            k_mutex_lock(&rpm_lock, K_FOREVER);
            measured_rpm = (int)local_measured_rpm;
            k_mutex_unlock(&rpm_lock);

            /* print hall state + measured rpm */
            printk("Measured RPM: %u\n",
                   local_measured_rpm);

            last_state = state;
        }

        k_msleep(20); /* poll interval */
    }
}

K_THREAD_STACK_DEFINE(hall_stack, STACK_HALL_MON);
static struct k_thread hall_tid;

int main()
{
    if (!device_is_ready(pwm_dev)) {
    printk("Error: PWM device not ready\n");
    }

    if (!adc_dev || !device_is_ready(adc_dev)) {
        printk("Warning: ADC device not ready.\n");
    } else {
        printk("ADC ready\n");
    }

    /* called once in main */
    k_mutex_init(&rpm_lock);

    k_thread_create(&user_tid, user_stack, K_THREAD_STACK_SIZEOF(user_stack),
                    user_input_thread, NULL, NULL, NULL,
                    PRIO_USER_INPUT, 0, K_NO_WAIT);

    k_thread_create(&pwm_tid, pwm_stack, K_THREAD_STACK_SIZEOF(pwm_stack),
                    pwm_thread, NULL, NULL, NULL,
                    PRIO_PWM, 0, K_NO_WAIT);

    k_thread_create(&hall_tid, hall_stack, K_THREAD_STACK_SIZEOF(hall_stack),
                    hall_monitor_thread, NULL, NULL, NULL,
                    PRIO_HALL_MON, 0, K_NO_WAIT);

    k_thread_create(&pid_tid, pid_stack, K_THREAD_STACK_SIZEOF(pid_stack),
                    pid_thread, NULL, NULL, NULL,
                    PRIO_PID, 0, K_NO_WAIT);

    return 0;
}