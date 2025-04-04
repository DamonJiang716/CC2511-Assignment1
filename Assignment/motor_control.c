/** motor_control.c - 步进电机和主轴控制模块实现 */
#include "motor_control.h"

// --- Hardware Pin Definitions ---
#define X_STEP_PIN        2     // X axis STEP pin
#define X_DIR_PIN         3     // X axis DIR pin
#define Y_STEP_PIN        4     // Y axis STEP pin
#define Y_DIR_PIN         5     // Y axis DIR pin
#define Z_STEP_PIN        6     // Z axis STEP pin
#define Z_DIR_PIN         7     // Z axis DIR pin
#define EN_PIN            8     // Global ENABLE pin for stepper drivers
#define M0_PIN            9     // Microstepping control pin M0
#define M1_PIN           10     // Microstepping control pin M1
#define M2_PIN           11     // Microstepping control pin M2
#define SPINDLE_PWM_PIN  14     // PWM output pin for spindle (e.g., GP14)

// --- Control Parameter Macros ---
#define MICROSTEP_MODE        16     // Microstepping mode (valid: 1, 2, 4, 8, 16, 32)
#define MIN_STEPPER_SPEED    100     // Minimum stepper speed (steps/second)
#define MAX_STEPPER_SPEED   1000     // Maximum stepper speed (steps/second)
#define STEP_PULSE_US         10     // Duration of STEP high pulse (in microseconds)
#define STEP_DEFAULT_SPEED   500     // Default stepper speed (steps/second)
#define MANUAL_STEP_SIZE      10     // Manual jog step size (used by arrow keys)

// Convert speed (steps/sec) to delay per step (µs)
#define SPEED_TO_DELAY_US(speed) (1000000u / (speed))

// --- Static Global Variables ---
static StepperMotor motors[3];           // Stepper motor status for X, Y, Z axes
static uint32_t step_delay_us = 0;       // Current step pulse delay (microseconds)
static uint8_t spindle_speed_percent = 0; // Current spindle speed in percentage (0–100)

/** Internal helper: set DRV8825 microstepping mode via M0, M1, M2 */
static void apply_microstep_pins(int microstep) {
    switch (microstep) {
        case 1:  // Full step
            gpio_put(M0_PIN, 0); gpio_put(M1_PIN, 0); gpio_put(M2_PIN, 0);
            break;
        case 2:  // Half step
            gpio_put(M0_PIN, 1); gpio_put(M1_PIN, 0); gpio_put(M2_PIN, 0);
            break;
        case 4:  // 1/4 step
            gpio_put(M0_PIN, 0); gpio_put(M1_PIN, 1); gpio_put(M2_PIN, 0);
            break;
        case 8:  // 1/8 step
            gpio_put(M0_PIN, 1); gpio_put(M1_PIN, 1); gpio_put(M2_PIN, 0);
            break;
        case 16: // 1/16 step
            gpio_put(M0_PIN, 0); gpio_put(M1_PIN, 0); gpio_put(M2_PIN, 1);
            break;
        case 32: // 1/32 step
            gpio_put(M0_PIN, 1); gpio_put(M1_PIN, 0); gpio_put(M2_PIN, 1);
            break;
        default:
            // Ignore invalid microstep values
            break;
    }
}

/** Initialize the motor control module */
void motor_init() {
    // Initialize stepper motor structs (set pins and max steps)
    motors[AXIS_X].step_pin = X_STEP_PIN;
    motors[AXIS_X].dir_pin  = X_DIR_PIN;
    motors[AXIS_X].position = 0;
    motors[AXIS_X].max_steps = 10000;

    motors[AXIS_Y].step_pin = Y_STEP_PIN;
    motors[AXIS_Y].dir_pin  = Y_DIR_PIN;
    motors[AXIS_Y].position = 0;
    motors[AXIS_Y].max_steps = 10000;

    motors[AXIS_Z].step_pin = Z_STEP_PIN;
    motors[AXIS_Z].dir_pin  = Z_DIR_PIN;
    motors[AXIS_Z].position = 0;
    motors[AXIS_Z].max_steps = 10000;

    // Initialize GPIOs
    gpio_init(X_STEP_PIN); gpio_set_dir(X_STEP_PIN, GPIO_OUT); gpio_put(X_STEP_PIN, 0);
    gpio_init(X_DIR_PIN);  gpio_set_dir(X_DIR_PIN, GPIO_OUT);  gpio_put(X_DIR_PIN, 0);
    gpio_init(Y_STEP_PIN); gpio_set_dir(Y_STEP_PIN, GPIO_OUT); gpio_put(Y_STEP_PIN, 0);
    gpio_init(Y_DIR_PIN);  gpio_set_dir(Y_DIR_PIN, GPIO_OUT);  gpio_put(Y_DIR_PIN, 0);
    gpio_init(Z_STEP_PIN); gpio_set_dir(Z_STEP_PIN, GPIO_OUT); gpio_put(Z_STEP_PIN, 0);
    gpio_init(Z_DIR_PIN);  gpio_set_dir(Z_DIR_PIN, GPIO_OUT);  gpio_put(Z_DIR_PIN, 0);

    // Initialize ENABLE pin
    gpio_init(EN_PIN);
    gpio_set_dir(EN_PIN, GPIO_OUT);
    gpio_put(EN_PIN, 0); // Enable stepper drivers (active low)

    // Initialize microstepping control pins
    gpio_init(M0_PIN); gpio_set_dir(M0_PIN, GPIO_OUT);
    gpio_init(M1_PIN); gpio_set_dir(M1_PIN, GPIO_OUT);
    gpio_init(M2_PIN); gpio_set_dir(M2_PIN, GPIO_OUT);
    motor_set_microstep(MICROSTEP_MODE);

    // Set default step delay according to configured speed
    if (STEP_DEFAULT_SPEED < MIN_STEPPER_SPEED)
        step_delay_us = SPEED_TO_DELAY_US(MIN_STEPPER_SPEED);
    else if (STEP_DEFAULT_SPEED > MAX_STEPPER_SPEED)
        step_delay_us = SPEED_TO_DELAY_US(MAX_STEPPER_SPEED);
    else
        step_delay_us = SPEED_TO_DELAY_US(STEP_DEFAULT_SPEED);

    // Initialize spindle PWM output
    gpio_set_function(SPINDLE_PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(SPINDLE_PWM_PIN);
    uint chan = pwm_gpio_to_channel(SPINDLE_PWM_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 125.0f);  // 125 MHz / 125 = 1 MHz base clock
    pwm_set_wrap(slice_num, 999);            // 1 MHz / 1000 ≈ 1 kHz PWM frequency
    pwm_init(slice_num, &config, true);
    pwm_set_chan_level(slice_num, chan, 0);  // Set duty cycle to 0 (off)
    spindle_speed_percent = 0;
}

/** Enable or disable stepper motor drivers */
void motor_enable(bool enable) {
    gpio_put(EN_PIN, enable ? 0 : 1);  // ENABLE is active LOW
}

/** Set the microstepping mode */
void motor_set_microstep(int microstep) {
    apply_microstep_pins(microstep);
    // (Optional: store microstep value in a global variable if needed)
}

/** Move a specific axis by a number of steps */
void motor_move_steps(AxisIndex axis, int32_t steps) {
    if (steps == 0) return;

    bool dir = true;  // true = forward, false = reverse
    if (steps < 0) {
        dir = false;
        steps = -steps;
    }

    StepperMotor *motor = &motors[axis];
    int32_t target_position = motor->position + (dir ? steps : -steps);

    if (target_position > motor->max_steps) {
        target_position = motor->max_steps;
        steps = motor->max_steps - motor->position;
    } else if (target_position < 0) {
        target_position = 0;
        steps = motor->position;
    }

    if (steps == 0) {
        motor->position = target_position;
        return;
    }

    gpio_put(motor->dir_pin, dir ? 1 : 0);
    sleep_us(5); // Ensure direction signal is stable (DRV8825 spec)

    for (uint32_t i = 0; i < (uint32_t)steps; ++i) {
        gpio_put(motor->step_pin, 1); // Start pulse
        sleep_us(STEP_PULSE_US);
        gpio_put(motor->step_pin, 0); // End pulse
        if (step_delay_us > STEP_PULSE_US)
            sleep_us(step_delay_us - STEP_PULSE_US);
        else
            sleep_us(1); // Fallback delay
    }

    motor->position = target_position;
}

/** Set spindle speed (PWM duty cycle in percentage) */
void spindle_set_speed(uint8_t percent) {
    if (percent > 100) percent = 100;
    uint slice_num = pwm_gpio_to_slice_num(SPINDLE_PWM_PIN);
    uint chan = pwm_gpio_to_channel(SPINDLE_PWM_PIN);
    uint16_t level = percent * 10; // With wrap=999, 100% ≈ 1000
    if (level > 1000) level = 1000;
    pwm_set_chan_level(slice_num, chan, level);
    spindle_speed_percent = percent;
}
StepperMotor motor_get_status(AxisIndex axis) {
    return motors[axis];  
}
uint32_t motor_get_step_delay() {
    return step_delay_us;
}

/** Get current spindle speed percentage */
uint8_t spindle_get_speed() {
    return spindle_speed_percent;
}
