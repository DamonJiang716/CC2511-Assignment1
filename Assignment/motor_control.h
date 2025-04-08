/** motor_control.h - 步进电机和主轴控制模块头文件 */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "pico/stdlib.h"   // Pico standard library (GPIO, delays, etc.)
#include "hardware/pwm.h"  // Hardware PWM library (for spindle control)

#define MICROSTEP_MODE        16     // Microstepping mode (valid: 1, 2, 4, 8, 16, 32)
#define MIN_STEPPER_SPEED    100     // Minimum stepper speed (steps/second)
#define MAX_STEPPER_SPEED   1000     // Maximum stepper speed (steps/second)
#define STEP_PULSE_US         10     // Duration of STEP high pulse (in microseconds)
#define STEP_DEFAULT_SPEED   500     // Default stepper speed (steps/second)
#define MANUAL_STEP_SIZE      10     // Manual jog step size (used by arrow keys)
/** Stepper motor status structure */
typedef struct {
    uint step_pin;      // STEP pulse control pin number
    uint dir_pin;       // DIR direction control pin number
    int32_t position;   // Current axis position (in steps)
    int32_t max_steps;  // Maximum number of steps for this axis (based on travel length)
} StepperMotor;

/** Axis index enumeration for array access */
typedef enum { AXIS_X = 0, AXIS_Y, AXIS_Z } AxisIndex;

/** Initialize motor control module: configure GPIOs, PWM, etc. */
void motor_init();

/** Enable or disable all stepper motor drivers
 *  @param enable If true, enable drivers (ENABLE pin LOW); if false, disable (ENABLE pin HIGH)
 */
void motor_enable(bool enable);

/** Set microstepping mode for stepper drivers
 *  @param microstep Microstep value (allowed: 1, 2, 4, 8, 16, 32 for full, half, 1/4...1/32 step)
 */
void motor_set_microstep(int microstep);

/** Move a specific axis by a number of steps
 *  @param axis Axis index (AXIS_X, AXIS_Y, AXIS_Z)
 *  @param steps Number of steps to move; positive = forward, negative = backward
 */
void motor_move_steps(AxisIndex axis, int32_t steps);

/** Set spindle motor speed (PWM duty cycle)
 *  @param percent Speed percentage (range: 0 to 100)
 */
void spindle_set_speed(uint8_t percent);

/** Get current spindle speed percentage (either stored or calculated from PWM)
 *  @return Current spindle speed as a percentage (0 to 100%)
 */
uint8_t spindle_get_speed();

StepperMotor motor_get_status(AxisIndex axis);
uint32_t motor_get_step_delay();

/** Parses G0/G1 commands (moves to ) ...*/
bool gcode_process_line(const char *line);

/** Absolute repositioning function */
void move_to(int x, int y);

#endif // MOTOR_CONTROL_H

