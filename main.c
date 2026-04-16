#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <stm32wbxx.h> // Required for bare-metal TIM1 register access

// --- Hardware Setup & Globals ---
static const struct gpio_dt_spec hall_u = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), hall_u_gpios);
static const struct gpio_dt_spec hall_v = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), hall_v_gpios);
static const struct gpio_dt_spec hall_w = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), hall_w_gpios);

static struct gpio_callback hall_port_a_cb;
static struct gpio_callback hall_port_c_cb;

volatile bool is_running = false;
volatile uint8_t current_hall_state = 0;

// --- New RPM Tracking Globals ---
volatile uint32_t previous_ticks = 0;
volatile uint32_t delta_ticks = 0;
volatile bool new_rpm_data = false;
volatile uint32_t current_rpm = 0;

#define HISTORY_SIZE 8
volatile uint32_t delta_history[HISTORY_SIZE] = {0};
volatile uint8_t delta_idx = 0;
volatile uint32_t smoothed_rpm = 0;

#define RPM_CONSTANT 2500000 
#define TIMEOUT_TICKS 1000000 
#define RPM_CONSTANT_FILTERED 15000000

#define TIMER_CLOCK_HZ  32000000  
#define TARGET_PWM_FREQ 21000    
#define PWM_PERIOD_ARR  (TIMER_CLOCK_HZ / TARGET_PWM_FREQ)

#define DUTY_LOW_95     ((PWM_PERIOD_ARR * 95) / 100) 
#define DUTY_TARGET     ((PWM_PERIOD_ARR * 60) / 100) 
#define DUTY_START      ((PWM_PERIOD_ARR * 10)  / 100) 
#define DUTY_STEP       ((PWM_PERIOD_ARR * 5)  / 1000)
#define STEP_DELAY_MS   20                            

volatile uint32_t active_duty = DUTY_START;
bool counter_clockwise = false;

// --- Commutation Engine ---
void commutate(uint8_t step)
{
    current_hall_state = step;
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                    TIM_CCER_CC2E | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E | TIM_CCER_CC3NE);

    if (!is_running) {
        TIM1->CCR1 = DUTY_LOW_95;
        TIM1->CCR2 = DUTY_LOW_95;
        TIM1->CCR3 = DUTY_LOW_95;
        TIM1->CCER = TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE;
        return;
    }

    if (counter_clockwise) {
        // CORRECTED REVERSE: -60-degree software shift 
        // Compensates for physical Hall sensor offset to run CW smoothly
        switch (step) {
        case 5: 
            TIM1->CCR3 = active_duty;
            TIM1->CCR1 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC1NE;
            break;
        case 4: 
            TIM1->CCR3 = active_duty;
            TIM1->CCR2 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC2NE;
            break;
        case 6: 
            TIM1->CCR1 = active_duty;
            TIM1->CCR2 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2NE;
            break;
        case 2: 
            TIM1->CCR1 = active_duty;
            TIM1->CCR3 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC3NE;
            break;
        case 3: 
            TIM1->CCR2 = active_duty;
            TIM1->CCR3 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC3NE;
            break;
        case 1: 
            TIM1->CCR2 = active_duty;
            TIM1->CCR1 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC1NE;
            break;
        default: break;
        }
    } else {
        switch (step) {
        case 5: 
            TIM1->CCR1 = active_duty;
            TIM1->CCR2 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2NE;
            break;
        case 4: 
            TIM1->CCR1 = active_duty;
            TIM1->CCR3 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC3NE;
            break;
        case 6: 
            TIM1->CCR2 = active_duty;
            TIM1->CCR3 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC3NE;
            break;
        case 2: 
            TIM1->CCR2 = active_duty;
            TIM1->CCR1 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC1NE;
            break;
        case 3: 
            TIM1->CCR3 = active_duty;
            TIM1->CCR1 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC1NE;
            break;
        case 1: 
            TIM1->CCR3 = active_duty;
            TIM1->CCR2 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC2NE;
            break;
        default: break;
        }
    }
}

void setup_tim2_freerun(void) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    TIM2->PSC = 31;
    TIM2->ARR = 0xFFFFFFFF;
    TIM2->EGR |= TIM_EGR_UG;
    TIM2->CR1 |= TIM_CR1_CEN;
}

// --- Helper Functions ---
uint8_t get_hall_state(void) {
    // CRITICAL FIX: Updated register math to match your new pinout!
    uint8_t hu = (GPIOC->IDR & (1 << 2)) ? 1 : 0; // PC2
    uint8_t hv = (GPIOA->IDR & (1 << 0)) ? 1 : 0; // PA0
    uint8_t hw = (GPIOA->IDR & (1 << 1)) ? 1 : 0; // PA1
    return (hu << 2) | (hv << 1) | hw;
}

#define DEBOUNCE_TICKS 100 
void hall_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    uint32_t current_ticks = TIM2->CNT;
    uint32_t delta = current_ticks - previous_ticks;

    if (delta < DEBOUNCE_TICKS) { return; }

    delta_history[delta_idx] = delta;
    delta_idx = (delta_idx + 1) % HISTORY_SIZE; 
    previous_ticks = current_ticks;
    new_rpm_data = true;

    uint8_t state = get_hall_state();
    commutate(state);
}

// --- Hardware Initialization ---
void setup_motor_pwm(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    TIM1->PSC = 0; 
    TIM1->ARR = PWM_PERIOD_ARR; 

    TIM1->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
    TIM1->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;

    TIM1->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 |= TIM_CCMR2_OC3PE;

    uint32_t dtg_value = 32; 
    TIM1->BDTR &= ~TIM_BDTR_DTG;             
    TIM1->BDTR |= (dtg_value & TIM_BDTR_DTG);

    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1 |= TIM_CR1_CEN;
}

void setup_hall_interrupts(void) {
    gpio_pin_configure_dt(&hall_u, GPIO_INPUT);
    gpio_pin_configure_dt(&hall_v, GPIO_INPUT);
    gpio_pin_configure_dt(&hall_w, GPIO_INPUT);

    gpio_pin_interrupt_configure_dt(&hall_u, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&hall_v, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&hall_w, GPIO_INT_EDGE_BOTH);

    // CRITICAL FIX: Grouping the pins correctly based on their Port!
    // U is alone on Port C. V and W are together on Port A.
    gpio_init_callback(&hall_port_c_cb, hall_isr, BIT(hall_u.pin));
    gpio_init_callback(&hall_port_a_cb, hall_isr, BIT(hall_v.pin) | BIT(hall_w.pin));

    gpio_add_callback(hall_u.port, &hall_port_c_cb); 
    gpio_add_callback(hall_v.port, &hall_port_a_cb); // Covers both V and W
}

void update_rpm_task(void) {
    if (new_rpm_data) {
        unsigned int key = irq_lock();
        uint32_t local_deltas[HISTORY_SIZE];
        for (int i = 0; i < HISTORY_SIZE; i++) {
            local_deltas[i] = delta_history[i];
        }
        new_rpm_data = false;
        irq_unlock(key);

        uint32_t sum_delta = 0;
        for (int i = 0; i < HISTORY_SIZE; i++) {
            sum_delta += local_deltas[i];
        }

        if (sum_delta > 0) {
            uint32_t instant_rpm = 15000000 / sum_delta;
            delta_history[delta_idx] = instant_rpm;
            delta_idx = (delta_idx + 1) % HISTORY_SIZE;

            uint64_t rpm_sum = 0; 
            for (int i = 0; i < HISTORY_SIZE; i++) {
                rpm_sum += delta_history[i];
            }
            current_rpm = (uint32_t)(rpm_sum / HISTORY_SIZE);
            if (current_rpm > 5000 || !is_running){
                current_rpm = 0;
            }
        }
    }

    if ((TIM2->CNT - previous_ticks) > TIMEOUT_TICKS) {
        current_rpm = 0;
        for (int i = 0; i < HISTORY_SIZE; i++) delta_history[i] = 0;
    }
}

// --- Background Soft-Start & Telemetry Thread ---
void control_thread_fn(void) {
    int print_counter = 0;

    while (1) {
        update_rpm_task();
        if (is_running && (active_duty < DUTY_TARGET)) {
            active_duty += DUTY_STEP;
            if (active_duty > DUTY_TARGET) {
                active_duty = DUTY_TARGET;
            }
            commutate(get_hall_state());
        }

        if (++print_counter >= 25) {
            uint32_t current_percent = (active_duty * 100) / PWM_PERIOD_ARR;
            if (!is_running) current_percent = 95; 
            
            printk("[TELEMETRY] Motor: %s | Duty: %d%% | Hall State: %d | RPM: %d\n", 
                   is_running ? "RUNNING" : "STOPPED", 
                   current_percent, 
                   current_hall_state, current_rpm);
            print_counter = 0;
        }
        
        k_msleep(STEP_DELAY_MS);
    }
}

K_THREAD_DEFINE(control_thread_id, 1024, control_thread_fn, NULL, NULL, NULL, 7, 0, 0);

// --- Main Loop ---
int main(void) {
    const struct device *console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    unsigned char s;

    /* Start the Native USB interface for Telemetry */
    if (usb_enable(NULL)) {
        return 0;
    }
    k_sleep(K_MSEC(1000)); // Brief pause to allow host PC to enumerate COM port

    setup_tim2_freerun();
    setup_hall_interrupts();
    setup_motor_pwm();

    commutate(get_hall_state());

    printk("\n--- BLDC Control System Ready ---\n");
    printk("PWM Frequency: 21 kHz\n");
    printk("Bootstrap Active: Low-sides holding at 85%%.\n");
    printk("Press 's' to START.\n");
    printk("Press [SPACE] (or any other key) to STOP.\n");
    printk("---------------------------------\n");

    while (1) {
        /* Safely poll for USB Serial commands */
        if (uart_poll_in(console_dev, &s) == 0) {
            if (s == '\n' || s == '\r') {
                continue; 
            }

            if (s == 's' || s == 'S') {
                if (!is_running) {
                    is_running = true;
                    printk("\n>>> COMMAND: Motor STARTING. Ramping from 5%% to 50%%...\n");
                    active_duty = DUTY_START;
                    commutate(get_hall_state());
                } else {
                    printk("\n>>> Note: Motor is already running.\n");
                }
            } 
            else {
                if (is_running) {
                    is_running = false;
                    printk("\n>>> COMMAND: Motor STOPPED. Bootstrap Active.\n");
                    commutate(get_hall_state());
                }
            }
        }
        k_msleep(10); // Prevent tight spinning
    }
    return 0;
}