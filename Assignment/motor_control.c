/** motor_control.c - 步进电机和主轴控制模块实现 */
#include "motor_control.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// --- Hardware Pin Definitions ---
// GND to Pico pin 18
#define X_STEP_PIN        15     // X axis STEP pin              // STEP to GPIO15   // STEP to GPIO   // STEP to GPIO2
#define X_DIR_PIN         14     // X axis DIR pin               // DIR to GPIO14    // DIR to GPIO1   // DIR to GPIO3
#define Y_STEP_PIN        13     // Y axis STEP pin
#define Y_DIR_PIN         12     // Y axis DIR pin
#define Z_STEP_PIN        11     // Z axis STEP pin
#define Z_DIR_PIN         10     // Z axis DIR pin
#define EN_PIN            9     // Global ENABLE pin for stepper drivers    // ENABLE to GPIO13 // ENABLE to GPIO4 // ENABLE to GPIO8
#define X_M0_PIN            8     // Microstepping control pin M0 for X axis driver          
#define X_M1_PIN            7     // Microstepping control pin M1 for X axis driver
#define X_M2_PIN            6     // Microstepping control pin M2 for X axis driver
#define Y_M0_PIN            5     // Microstepping control pin M0 for Y axis driver
#define Y_M1_PIN            4     // Microstepping control pin M1 for Y axis driver 
#define Y_M2_PIN            3     // Microstepping control pin M2 for Y axis driver
#define Z_M0_PIN          0       // Microstepping control pin M0 for Z axis driver   // deadass going to ground (pin 13) ?
#define Z_M1_PIN          0       // Microstepping control pin M1 for Z axis driver   // GND too (pin 8) ? Using unwired GP0 for now
#define Z_M2_PIN           18     // Microstepping control pin M2 for Z axis driver
#define SPINDLE_PWM_PIN   17      // PWM enable/disbale output pin for spindle 

// --- Control Parameter Macros ---
#define MICROSTEP_MODE        16     // Microstepping mode (valid: 1, 2, 4, 8, 16, 32)
#define MIN_STEPPER_SPEED    100     // Minimum stepper speed (steps/second)
#define MAX_STEPPER_SPEED   1000     // Maximum stepper speed (steps/second)
#define STEP_PULSE_US         1000     // Duration of STEP high pulse (in microseconds)
#define STEP_DEFAULT_SPEED   500     // Default stepper speed (steps/second)
#define MANUAL_STEP_SIZE      10     // Manual jog step size (used by arrow keys)

// Convert speed (steps/sec) to delay per step (µs)
#define SPEED_TO_DELAY_US(speed) (1000000u / (speed))

// --- Static Global Variables ---
static StepperMotor motors[3];           // Stepper motor status for X, Y, Z axes
static uint32_t step_delay_us = 0;       // Current step pulse delay (microseconds)
static uint8_t spindle_speed_percent = 0; // Current spindle speed in percentage (0–100)

// /** Internal helper: set DRV8825 microstepping mode via M0, M1, M2 */
// static void apply_microstep_pins(int microstep) {
//     switch (microstep) {
//         case 1:  // Full step
//             gpio_put(M0_PIN, 0); gpio_put(M1_PIN, 0); gpio_put(M2_PIN, 0);
//             break;
//         case 2:  // Half step
//             gpio_put(M0_PIN, 1); gpio_put(M1_PIN, 0); gpio_put(M2_PIN, 0);
//             break;
//         case 4:  // 1/4 step
//             gpio_put(M0_PIN, 0); gpio_put(M1_PIN, 1); gpio_put(M2_PIN, 0);
//             break;
//         case 8:  // 1/8 step
//             gpio_put(M0_PIN, 1); gpio_put(M1_PIN, 1); gpio_put(M2_PIN, 0);
//             break;
//         case 16: // 1/16 step
//             gpio_put(M0_PIN, 0); gpio_put(M1_PIN, 0); gpio_put(M2_PIN, 1);
//             break;
//         case 32: // 1/32 step
//             gpio_put(M0_PIN, 1); gpio_put(M1_PIN, 0); gpio_put(M2_PIN, 1);
//             break;
//         default:
//             // Ignore invalid microstep values
//             break;
//     }
// }

/** Reset position to (x,y,z) - called from main.c as a way to alter motors struct without visibility issues */
void motor_reset_position(int x, int y, int z) {
    motors[AXIS_X].position = x;
    motors[AXIS_Y].position = y;
    motors[AXIS_Z].position = z;
}

/** Initialize the motor control module */
void motor_init() {
    // === Initialize stepper motor structs ===
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

    // === Initialize STEP and DIR GPIOs ===
    gpio_init(X_STEP_PIN); gpio_set_dir(X_STEP_PIN, GPIO_OUT); gpio_put(X_STEP_PIN, 0);
    gpio_init(X_DIR_PIN);  gpio_set_dir(X_DIR_PIN, GPIO_OUT);  gpio_put(X_DIR_PIN, 0);
    gpio_init(Y_STEP_PIN); gpio_set_dir(Y_STEP_PIN, GPIO_OUT); gpio_put(Y_STEP_PIN, 0);
    gpio_init(Y_DIR_PIN);  gpio_set_dir(Y_DIR_PIN, GPIO_OUT);  gpio_put(Y_DIR_PIN, 0);
    gpio_init(Z_STEP_PIN); gpio_set_dir(Z_STEP_PIN, GPIO_OUT); gpio_put(Z_STEP_PIN, 0);
    gpio_init(Z_DIR_PIN);  gpio_set_dir(Z_DIR_PIN, GPIO_OUT);  gpio_put(Z_DIR_PIN, 0);

    // === Initialize ENABLE pin ===
    gpio_init(EN_PIN);
    gpio_set_dir(EN_PIN, GPIO_OUT);
    gpio_put(EN_PIN, 0); // Active LOW

    // === Initialize per-axis microstepping pins ===
    gpio_init(X_M0_PIN); gpio_set_dir(X_M0_PIN, GPIO_OUT);
    gpio_init(X_M1_PIN); gpio_set_dir(X_M1_PIN, GPIO_OUT);
    gpio_init(X_M2_PIN); gpio_set_dir(X_M2_PIN, GPIO_OUT);

    gpio_init(Y_M0_PIN); gpio_set_dir(Y_M0_PIN, GPIO_OUT);
    gpio_init(Y_M1_PIN); gpio_set_dir(Y_M1_PIN, GPIO_OUT);
    gpio_init(Y_M2_PIN); gpio_set_dir(Y_M2_PIN, GPIO_OUT);

    gpio_init(Z_M0_PIN); gpio_set_dir(Z_M0_PIN, GPIO_OUT);
    gpio_init(Z_M1_PIN); gpio_set_dir(Z_M1_PIN, GPIO_OUT);
    gpio_init(Z_M2_PIN); gpio_set_dir(Z_M2_PIN, GPIO_OUT);

    // === Set microstepping mode for each axis ===
    motor_set_microstep(AXIS_X, MICROSTEP_MODE);
    motor_set_microstep(AXIS_Y, MICROSTEP_MODE);
    motor_set_microstep(AXIS_Z, MICROSTEP_MODE);

    // === Set default step delay ===
    if (STEP_DEFAULT_SPEED < MIN_STEPPER_SPEED)
        step_delay_us = SPEED_TO_DELAY_US(MIN_STEPPER_SPEED);
    else if (STEP_DEFAULT_SPEED > MAX_STEPPER_SPEED)
        step_delay_us = SPEED_TO_DELAY_US(MAX_STEPPER_SPEED);
    else
        step_delay_us = SPEED_TO_DELAY_US(STEP_DEFAULT_SPEED);

    // === Spindle PWM setup ===
    gpio_set_function(SPINDLE_PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(SPINDLE_PWM_PIN);
    uint chan = pwm_gpio_to_channel(SPINDLE_PWM_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 125.0f);  // 1 MHz base
    pwm_set_wrap(slice_num, 999);            // ~1 kHz PWM
    pwm_init(slice_num, &config, true);
    pwm_set_chan_level(slice_num, chan, 0);  // Start off
    spindle_speed_percent = 0;
}

/** Initialize the motor control module */
// void motor_init() {
//     // Initialize stepper motor structs (set pins and max steps)
//     motors[AXIS_X].step_pin = X_STEP_PIN;
//     motors[AXIS_X].dir_pin  = X_DIR_PIN;
//     motors[AXIS_X].position = 0;
//     motors[AXIS_X].max_steps = 10000;

//     motors[AXIS_Y].step_pin = Y_STEP_PIN;
//     motors[AXIS_Y].dir_pin  = Y_DIR_PIN;
//     motors[AXIS_Y].position = 0;
//     motors[AXIS_Y].max_steps = 10000;

//     motors[AXIS_Z].step_pin = Z_STEP_PIN;
//     motors[AXIS_Z].dir_pin  = Z_DIR_PIN;
//     motors[AXIS_Z].position = 0;
//     motors[AXIS_Z].max_steps = 10000;

//     // Initialize GPIOs
//     gpio_init(X_STEP_PIN); gpio_set_dir(X_STEP_PIN, GPIO_OUT); gpio_put(X_STEP_PIN, 0);
//     gpio_init(X_DIR_PIN);  gpio_set_dir(X_DIR_PIN, GPIO_OUT);  gpio_put(X_DIR_PIN, 0);
//     gpio_init(Y_STEP_PIN); gpio_set_dir(Y_STEP_PIN, GPIO_OUT); gpio_put(Y_STEP_PIN, 0);
//     gpio_init(Y_DIR_PIN);  gpio_set_dir(Y_DIR_PIN, GPIO_OUT);  gpio_put(Y_DIR_PIN, 0);
//     gpio_init(Z_STEP_PIN); gpio_set_dir(Z_STEP_PIN, GPIO_OUT); gpio_put(Z_STEP_PIN, 0);
//     gpio_init(Z_DIR_PIN);  gpio_set_dir(Z_DIR_PIN, GPIO_OUT);  gpio_put(Z_DIR_PIN, 0);

//     // Initialize ENABLE pin
//     gpio_init(EN_PIN);
//     gpio_set_dir(EN_PIN, GPIO_OUT);
//     gpio_put(EN_PIN, 0); // Enable stepper drivers (active low)

//     // Initialize microstepping control pins
//     gpio_init(M0_PIN); gpio_set_dir(M0_PIN, GPIO_OUT);
//     gpio_init(M1_PIN); gpio_set_dir(M1_PIN, GPIO_OUT);
//     gpio_init(M2_PIN); gpio_set_dir(M2_PIN, GPIO_OUT);
//     motor_set_microstep(MICROSTEP_MODE);

//     // Set default step delay according to configured speed
//     if (STEP_DEFAULT_SPEED < MIN_STEPPER_SPEED)
//         step_delay_us = SPEED_TO_DELAY_US(MIN_STEPPER_SPEED);
//     else if (STEP_DEFAULT_SPEED > MAX_STEPPER_SPEED)
//         step_delay_us = SPEED_TO_DELAY_US(MAX_STEPPER_SPEED);
//     else
//         step_delay_us = SPEED_TO_DELAY_US(STEP_DEFAULT_SPEED);

//     // Initialize spindle PWM output
//     gpio_set_function(SPINDLE_PWM_PIN, GPIO_FUNC_PWM);
//     uint slice_num = pwm_gpio_to_slice_num(SPINDLE_PWM_PIN);
//     uint chan = pwm_gpio_to_channel(SPINDLE_PWM_PIN);
//     pwm_config config = pwm_get_default_config();
//     pwm_config_set_clkdiv(&config, 125.0f);  // 125 MHz / 125 = 1 MHz base clock
//     pwm_set_wrap(slice_num, 999);            // 1 MHz / 1000 ≈ 1 kHz PWM frequency
//     pwm_init(slice_num, &config, true);
//     pwm_set_chan_level(slice_num, chan, 0);  // Set duty cycle to 0 (off)
//     spindle_speed_percent = 0;
// }

/** Enable or disable stepper motor drivers */
void motor_enable(bool enable) {
    gpio_put(EN_PIN, enable ? 0 : 1);  // ENABLE is active LOW
}

/** Set the microstepping mode for a specific axis
 *  @param axis Axis index (AXIS_X, AXIS_Y, AXIS_Z)
 *  @param microstep Desired microstepping mode (1, 2, 4, 8, 16, 32)
 */
void motor_set_microstep(AxisIndex axis, int microstep) {
    uint m0_pin, m1_pin, m2_pin;
    motors[axis].microstep_mode = microstep;

    // Assign the correct M0, M1, M2 pins for each axis
    switch (axis) {
        case AXIS_X:
            m0_pin = X_M0_PIN;
            m1_pin = X_M1_PIN;
            m2_pin = X_M2_PIN;
            break;
        case AXIS_Y:
            m0_pin = Y_M0_PIN;
            m1_pin = Y_M1_PIN;
            m2_pin = Y_M2_PIN;
            break;
        case AXIS_Z:
            m0_pin = Z_M0_PIN;
            m1_pin = Z_M1_PIN;
            m2_pin = Z_M2_PIN;
            break;
        default:
            return; // Invalid axis
    }

    // Set pin levels based on desired microstepping mode
    switch (microstep) {
        case 1:  // Full step
            gpio_put(m0_pin, 0);
            gpio_put(m1_pin, 0);
            gpio_put(m2_pin, 0);
            break;
        case 2:  // Half step
            gpio_put(m0_pin, 1);
            gpio_put(m1_pin, 0);
            gpio_put(m2_pin, 0);
            break;
        case 4:  // 1/4 step
            gpio_put(m0_pin, 0);
            gpio_put(m1_pin, 1);
            gpio_put(m2_pin, 0);
            break;
        case 8:  // 1/8 step
            gpio_put(m0_pin, 1);
            gpio_put(m1_pin, 1);
            gpio_put(m2_pin, 0);
            break;
        case 16: // 1/16 step
            gpio_put(m0_pin, 0);
            gpio_put(m1_pin, 0);
            gpio_put(m2_pin, 1);
            break;
        case 32: // 1/32 step
            gpio_put(m0_pin, 1);
            gpio_put(m1_pin, 0);
            gpio_put(m2_pin, 1);
            break;
        default:
            // Invalid microstep setting — do nothing
            break;
    }

    // could store current microstepping mode per axis here if needed
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
    int32_t delta = (dir ? steps : -steps);
    // Reverse Y direction only in coordinate system
    // if (axis == AXIS_Y) {
    //     delta = -delta;
    // }
    int32_t target_position = motor->position + delta;

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
            sleep_us(step_delay_us - STEP_PULSE_US); //
        else
            sleep_us(1); // Fallback delay
    }

    motor->position = target_position;
}

/** Set spindle speed (PWM duty cycle in percentage) */
/** Set spindle speed (PWM duty cycle in percentage) with smooth ramp-up */
void spindle_set_speed(uint8_t percent) {
    if (percent > 100) percent = 100;

    uint slice_num = pwm_gpio_to_slice_num(SPINDLE_PWM_PIN);
    uint chan = pwm_gpio_to_channel(SPINDLE_PWM_PIN);
    uint16_t target_level = percent * 10;  // With wrap=999
    if (target_level > 1000) target_level = 1000;

    if (percent == 0) {
        pwm_set_chan_level(slice_num, chan, 0);
        pwm_set_enabled(slice_num, false);
    } else {
        pwm_set_enabled(slice_num, true);

        // Estimate previous level from the last known % value
        uint16_t current_level = spindle_speed_percent * 10;
        if (current_level > 1000) current_level = 1000;

        // Ramp up/down smoothly to target level
        const int step = 20;
        const int delay_ms = 10;

        if (current_level < target_level) {
            while (current_level < target_level) {
                current_level += step;
                if (current_level > target_level) current_level = target_level;
                pwm_set_chan_level(slice_num, chan, current_level);
                sleep_ms(delay_ms);
            }
        } else if (current_level > target_level) {
            while (current_level > target_level) {
                current_level -= step;
                if (current_level < target_level) current_level = target_level;
                pwm_set_chan_level(slice_num, chan, current_level);
                sleep_ms(delay_ms);
            }
        }
    }

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

/** Interpret char c = char(ch) G-Code Commands and calls relevant functions like move_to(int x, int y) 
 * or spindle_set_speed(uint8_t percent) */
bool gcode_process_line(const char *line) {
    int x = -9999, y = -9999, speed = -1;
    char g_cmd[4];

    // Parse G0 or G1
    if (sscanf(line, "G%3s", g_cmd) == 1) {
        if (strcmp(g_cmd, "0") == 0 || strcmp(g_cmd, "1") == 0) {
            // Scan for optional X and Y values
            const char *p = line;
            while (*p) {
                if (*p == 'X' || *p == 'x') x = atoi(p + 1);
                if (*p == 'Y' || *p == 'y') y = atoi(p + 1);
                p++;
            }

            if (x == -9999) x = motor_get_status(AXIS_X).position;
            if (y == -9999) y = motor_get_status(AXIS_Y).position;
            move_to(x, y);
            return true;
        }
    }

    // M3 Sxx (spindle ON)
    if (strstr(line, "M3") != NULL || strstr(line, "m3") != NULL) { // case insensitive check for Spindle on G-Code instruction
        if (sscanf(line, "%*s S%d", &speed) == 1 || sscanf(line, "%*s s%d", &speed) == 1) {
            if (speed < 0) speed = 0;
            if (speed > 100) speed = 100;
            spindle_set_speed(speed);
            return true;
        }
    }

    // M5 (spindle OFF)
    if (strncmp(line, "M5", 2) == 0) {
        spindle_set_speed(0);
        return true;
    }

    // Sxx (set spindle speed)
    if (sscanf(line, "S%d", &speed) == 1) {
        spindle_set_speed(speed);
        return true;
    }

    // M350 X<microstep> Y<microstep> Z<microstep> - Set microstepping mode
if (strncmp(line, "M350", 4) == 0) {
    int x = -1, y = -1, z = -1;

    // Look for microstepping values after axis letters
    sscanf(line, "M350 X%d Y%d Z%d", &x, &y, &z);

    if (x > 0) {
        motor_set_microstep(AXIS_X, x);
        printf("Set X microstep mode to 1/%d step\n", x);
    }
    if (y > 0) {
        motor_set_microstep(AXIS_Y, y);
        printf("Set Y microstep mode to 1/%d step\n", y);
    }
    if (z > 0) {
        printf("Set Y microstep mode to 1/%d step\n", z);
    }

    return true;
    }

    // G92 - user sets (0,0,0) position
if (strncmp(line, "G92", 3) == 0) {
        motors[AXIS_X].position = 0;
        motors[AXIS_Y].position = 0;
        motors[AXIS_Z].position = 0;
        printf("G92 applied: Current position set as (0,0,0)\n");
        return true;
    }

    return false;
}

/** Process G-code commands (G0/G1) */
void move_to(int x, int y) {
    StepperMotor mx = motor_get_status(AXIS_X);
    StepperMotor my = motor_get_status(AXIS_Y);

    int dx = x - mx.position;
    int dy = y - my.position;

    if (dx != 0) {
        motor_move_steps(AXIS_X, dx);
    }
    if (dy != 0) {
        motor_move_steps(AXIS_Y, dy);
    }

    printf("Moved to X=%d Y=%d (ΔX=%d, ΔY=%d)\r\n", x, y, dx, dy);
}
