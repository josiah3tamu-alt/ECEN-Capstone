#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/console/console.h>
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

// Add these for the Circular Buffer
#define HISTORY_SIZE 6
volatile uint32_t delta_history[HISTORY_SIZE] = {0};
volatile uint8_t delta_idx = 0;

#define RPM_CONSTANT 2500000 
#define TIMEOUT_TICKS 1000000 // 1 second timeout if 1 tick = 1us
#define RPM_CONSTANT_FILTERED 15000000

// --- PWM Frequency & Duty Cycle Configuration ---
// Note: If your Zephyr board config pushes the STM32WB to 64MHz, change this to 64000000
#define TIMER_CLOCK_HZ  32000000  
#define TARGET_PWM_FREQ 21000    

// The math: ARR = (Timer Clock / Target Frequency)
// At 32MHz and 21kHz, ARR evaluates to exactly 1280.
#define PWM_PERIOD_ARR  (TIMER_CLOCK_HZ / TARGET_PWM_FREQ)

// Dynamic Duty Cycle Calculations
#define DUTY_LOW_95     ((PWM_PERIOD_ARR * 95) / 100) // 95% low-side (5% high-side)
#define DUTY_TARGET     ((PWM_PERIOD_ARR * 70) / 100) // 50% target running duty
#define DUTY_START      ((PWM_PERIOD_ARR * 30)  / 100) // 5% starting duty
#define DUTY_STEP       ((PWM_PERIOD_ARR * 5)  / 1000)// Increase by 0.5% every step
#define STEP_DELAY_MS   20                            // Milliseconds between steps

volatile uint32_t active_duty = DUTY_START;
bool counter_clockwise = true;

// --- Commutation Engine ---
void commutate(uint8_t step)
{
    current_hall_state = step;
    // Disable all outputs first (this natively acts as our "Deactivate" state)
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                    TIM_CCER_CC2E | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E | TIM_CCER_CC3NE);

    if (!is_running) {
        // STOP STATE: All Low-sides at 85% for Bootstrap charging
        TIM1->CCR1 = DUTY_LOW_95;
        TIM1->CCR2 = DUTY_LOW_95;
        TIM1->CCR3 = DUTY_LOW_95;
        
        TIM1->CCER = TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE;
        return;
    }

    // RUNNING STATE: Apply the current dynamically ramping duty cycle
    if (counter_clockwise){
        switch (step) {
        case 5: // U+ W-
            TIM1->CCR1 = active_duty;
            TIM1->CCR3 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC3NE;
            break;
        case 4: // U+ V-
            TIM1->CCR1 = active_duty;
            TIM1->CCR2 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2NE;
            break;
        case 6: // W+ V-
            TIM1->CCR3 = active_duty;
            TIM1->CCR2 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC2NE;
            break;
        case 2: // W+ U-
            TIM1->CCR3 = active_duty;
            TIM1->CCR1 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC1NE;
            break;
        case 3: // V+ U-
            TIM1->CCR2 = active_duty;
            TIM1->CCR1 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC1NE;
            break;
        case 1: // V+ W-
            TIM1->CCR2 = active_duty;
            TIM1->CCR3 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC3NE;
            break;
        default:
            // Passing 0 or 7 disables all phases, which handles deactivation cleanly.
            break;
        }
    } else {
        switch (step) {
        case 5: // V+ W-
            TIM1->CCR2 = active_duty;
            TIM1->CCR3 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC3NE;
            break;
        case 4: // U+ W- 
            TIM1->CCR1 = active_duty;
            TIM1->CCR3 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC3NE;
            break;
        case 6: // U+ V-
            TIM1->CCR1 = active_duty;
            TIM1->CCR2 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2NE;
            break;
        case 2: // W+ V-
            TIM1->CCR3 = active_duty;
            TIM1->CCR2 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC2NE;
            break;
        case 3: // W+ U-
            TIM1->CCR3 = active_duty;
            TIM1->CCR1 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC1NE;
            break;
        case 1: // V+ U-
            TIM1->CCR2 = active_duty;
            TIM1->CCR1 = DUTY_LOW_95;
            TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC1NE;
            break;
        default:
            break;
        }
    }
}

void setup_tim2_freerun(void) {
    // 1. Enable the clock for TIM2 on APB1
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    
    // 2. Set Prescaler to get a 1 MHz clock (Assuming 32MHz APB1 clock: 32 - 1 = 31)
    // Adjust this if your Zephyr clock tree is configured differently!
    TIM2->PSC = 31;
    
    // 3. Set Auto-Reload Register to max 32-bit value
    TIM2->ARR = 0xFFFFFFFF;
    
    // 4. Generate an update event to load the prescaler immediately
    TIM2->EGR |= TIM_EGR_UG;
    
    // 5. Enable the timer counter
    TIM2->CR1 |= TIM_CR1_CEN;
}

// --- Hall Sensor ISR ---
void hall_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    uint32_t current_ticks = TIM2->CNT;
    
    // Calculate delta and push to the circular buffer
    uint32_t delta = current_ticks - previous_ticks;
    delta_history[delta_idx] = delta;
    
    // Increment index and wrap around using modulo
    delta_idx = (delta_idx + 1) % HISTORY_SIZE; 
    
    previous_ticks = current_ticks;
    new_rpm_data = true;

    uint8_t hu = (GPIOC->IDR & (1 << 2)) ? 1 : 0; 
    uint8_t hv = (GPIOC->IDR & (1 << 3)) ? 1 : 0; 
    uint8_t hw = (GPIOA->IDR & (1 << 1)) ? 1 : 0; 
    uint8_t state = (hu << 2) | (hv << 1) | hw;
    
    commutate(state);
}

// --- Helper Functions ---
uint8_t get_hall_state(void) {
    uint8_t hu = (GPIOC->IDR & (1 << 2)) ? 1 : 0; 
    uint8_t hv = (GPIOC->IDR & (1 << 3)) ? 1 : 0; 
    uint8_t hw = (GPIOA->IDR & (1 << 1)) ? 1 : 0; 
    return (hu << 2) | (hv << 1) | hw;
}

// --- Hardware Initialization ---
void setup_motor_pwm(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    TIM1->PSC = 0; 
    TIM1->ARR = PWM_PERIOD_ARR; // Set to exactly 25 kHz

    // Configure Channels 1, 2, 3 to PWM Mode 1
    TIM1->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
    TIM1->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;

    // Enable Preload
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 |= TIM_CCMR2_OC3PE;

    // 3. Set Dead-Time (~1us for a 32MHz clock -> DTG = 32)
    uint32_t dtg_value = 32; 
    TIM1->BDTR &= ~TIM_BDTR_DTG;             
    TIM1->BDTR |= (dtg_value & TIM_BDTR_DTG);
    
    // makes the channels inverse of each other
    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);
    TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);
    TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE);

    // 4. Initialize duty cycles to 0
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    // Main Output Enable (MOE)
    TIM1->BDTR |= TIM_BDTR_MOE;

    // Start Timer
    TIM1->CR1 |= TIM_CR1_CEN;
}

void setup_hall_interrupts(void) {
    gpio_pin_configure_dt(&hall_u, GPIO_INPUT);
    gpio_pin_configure_dt(&hall_v, GPIO_INPUT);
    gpio_pin_configure_dt(&hall_w, GPIO_INPUT);

    gpio_pin_interrupt_configure_dt(&hall_u, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&hall_v, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&hall_w, GPIO_INT_EDGE_BOTH);

    gpio_init_callback(&hall_port_c_cb, hall_isr, BIT(hall_u.pin) | BIT(hall_v.pin));
    gpio_init_callback(&hall_port_a_cb, hall_isr, BIT(hall_w.pin));

    gpio_add_callback(hall_u.port, &hall_port_c_cb); 
    gpio_add_callback(hall_w.port, &hall_port_a_cb); 
}

void update_rpm_task(void) {
    if (new_rpm_data) {
        new_rpm_data = false;
        
        // 1. Safely sum the last 6 deltas
        uint32_t sum_delta = 0;
        for (int i = 0; i < HISTORY_SIZE; i++) {
            sum_delta += delta_history[i];
        }
        
        // 2. Calculate the smoothed RPM
        // (Make sure sum_delta is large enough to avoid divide-by-zero)
        if (sum_delta > 0) {
            current_rpm = RPM_CONSTANT_FILTERED / sum_delta;
        }
    }
    
    // Timeout check
    if ((TIM2->CNT - previous_ticks) > TIMEOUT_TICKS) {
        current_rpm = 0;
        // Optional: clear the buffer so old data doesn't skew the next startup
        for(int i=0; i < HISTORY_SIZE; i++) delta_history[i] = 0;
    }
}

// --- Background Soft-Start & Telemetry Thread ---
void control_thread_fn(void) {
    int print_counter = 0;

    while (1) {
        update_rpm_task();
        // 1. Process Soft-Start Ramp
        if (is_running && (active_duty < DUTY_TARGET)) {
            active_duty += DUTY_STEP;
            if (active_duty > DUTY_TARGET) {
                active_duty = DUTY_TARGET;
            }
            commutate(get_hall_state());
        }

        // 2. Telemetry Printout (Every 500ms = 25 ticks of 20ms)
        if (++print_counter >= 25) {
            // Calculate actual percentage to print to the terminal
            uint32_t current_percent = (active_duty * 100) / PWM_PERIOD_ARR;
            if (!is_running) current_percent = 95; // Display bootstrap duty when stopped
            
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
    console_init();

    setup_tim2_freerun();
    
    setup_hall_interrupts();
    setup_motor_pwm();

    commutate(get_hall_state());

    printk("System Ready (PWM Frequency: 21 kHz)\n");
    printk("Bootstrap Active: Low-sides holding at 85%%.\n");
    printk("Press 's' to START.\n");
    printk("Press [ENTER] (or any other key) to STOP.\n");

    while (1) {
        char s = console_getchar(); // Blocks until a SINGLE character is pressed
        
        // 1. Handle the "Double Input" from terminal emulators
        // Terminals often send Carriage Return ('\r') followed by Line Feed ('\n').
        // We simply ignore the Line Feed so it doesn't trigger our logic twice.
        if (s == '\n') {
            continue; 
        }

        // 2. Start Command
        if (s == 's' || s == 'S') {
            if (!is_running) {
                is_running = true;
                active_duty = DUTY_START; // Reset back to 5% before starting
                printk("\n>>> COMMAND: Motor STARTING. Ramping from 5%% to 50%%...\n");
                commutate(get_hall_state());
            } else {
                printk("\n>>> Note: Motor is already running.\n");
            }
        } 
        // 3. Stop Command (Enter '\r', spacebar, or any other panic key)
        else {
            if (is_running) {
                is_running = false;
                printk("\n>>> COMMAND: Motor STOPPED. Bootstrap Active.\n");
                commutate(get_hall_state());
            }
        }
    }
    return 0;
}