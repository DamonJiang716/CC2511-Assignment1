/**************************************************************
 * main.c
 * rev 1.0 04-Apr-2025 ���׶�
 * Assignment
 * ***********************************************************/

 #include <stdlib.h>
 #include <stdio.h>
 #include <string.h>
 #include <ctype.h>
 #include "pico/stdlib.h"
 #include "motor_control.h"
 
 int main() {
     // Initialize standard I/O to enable USB serial
     stdio_init_all();
     while (!stdio_usb_connected()) {
         sleep_ms(10);
     }
     sleep_ms(100);  // Further delay necessary?
     motor_init();   // Initialize motor control module (GPIO, PWM, etc.)
 
     // Print welcome and help message
     printf("\n=== CNC Control System Started ===\r\n");
     printf("Stepper speed range: min %d steps/sec, max %d steps/sec\r\n", MIN_STEPPER_SPEED, MAX_STEPPER_SPEED);
     printf("Spindle speed range: 0%% to 100%%\r\n");
     printf("Current microstepping mode: 1/%d step\r\n", MICROSTEP_MODE);
     printf("Supported commands:\r\n");
     printf("  spindle <0-100>      Set spindle speed (%%)\r\n");
     printf("  x+N / x-N            Move X axis by ±N steps (same for y, z)\r\n");
     printf("  enable on/off        Enable or disable stepper drivers\r\n");
     printf("  G0 / G1 X<n> Y<n>    Move to absolute position (X, Y)\r\n");
    //  printf("  M3 S<1-100>          START spindle at speed <n> (0-100)\r\n"); same as spindle <1-100> command
     printf("  info                 Show current position and speed info\r\n");
     printf("  help                 Show command list\r\n");
     printf("  M350 X<n> Y<n> Z<n>  Set microstepping mode from; 1, 1/2, 1/4, 1/8, 1/16 & 1/32 step options where step: 1/n\r\n");
     printf("  G92                  Set current position as (0,0,0)\r\n");
     printf("Use arrow keys ←→ to control X-axis, ↑↓ to jog Y-axis by %d steps\r\n", MANUAL_STEP_SIZE);
     printf("Right click to paste your clipboard G-Code file into the terminal and ENSURE YOU PRESS ENTER\r\n");
     printf("--------------------------------------------\r\n");
 
     // Main loop: read and execute serial commands
     char cmd_buf[64];           // Command input buffer
     uint8_t buf_index = 0;      // Current buffer length
     int ch;

     ////////////////////////////
    //  motor_enable(true);
    //  sleep_ms(500);  // Let EN settle

    //  motor_move_steps(AXIS_X, 100);
    //  sleep_ms(1000);
    //  motor_move_steps(AXIS_X, -100);
     ////////////////////////////
 
     while (true) {
         ch = getchar_timeout_us(100);    // Non-blocking read, returns -1 on timeout
         if (ch == PICO_ERROR_TIMEOUT) {
             // Optional: do other tasks during idle time
             continue;
         }
 
         char c = (char) ch;
 
         // Optional echo: print visible characters back to terminal
         if (c >= 32 && c <= 126) {
             putchar(c);
         }
 
         // Handle backspace/delete
         if (c == 0x7F || c == 0x08) {
             if (buf_index > 0) {
                 buf_index--;
                 cmd_buf[buf_index] = '\0';
                 // Move cursor back and erase character
                 printf("\b \b");
             }
             continue;
         }
 
         // Handle ANSI escape sequences (e.g., arrow keys)
         if (c == 0x1B) {  // ESC character
             // Expect two more bytes: '[' and a direction letter
             int c1 = getchar_timeout_us(1000);
             int c2 = getchar_timeout_us(1000);
             if (c1 == '[' && c2 != PICO_ERROR_TIMEOUT) {
                 switch (c2) {
                     case 'A':  // ↑ Up arrow - move Y axis forward
                         motor_move_steps(AXIS_Y, MANUAL_STEP_SIZE);
                         break;
                     case 'B':  // ↓ Down arrow - move Y axis backward
                         motor_move_steps(AXIS_Y, -MANUAL_STEP_SIZE);
                         break;
                     case 'C':  // → Right arrow - move X axis forward
                         motor_move_steps(AXIS_X, MANUAL_STEP_SIZE);
                         break;
                     case 'D':  // ← Left arrow - move X axis backward
                         motor_move_steps(AXIS_X, -MANUAL_STEP_SIZE);
                         break;
                     default:
                         break;
                 }
             }
             // Note: arrow key actions are immediate and don't affect command buffer
             continue;
         }
 
         // Handle Enter key as end of command
         if (c == '\r' || c == '\n') {
             if (buf_index == 0) {
                 // Empty command, show prompt again
                 printf("\r\n> ");
                 continue;
             }
             cmd_buf[buf_index] = '\0';  // Null-terminate the string
             printf("\r\n");
 
             // Convert command to lowercase for case-insensitive comparison
             char cmd_lower[64];
             for (int i = 0; i <= buf_index; ++i) {
                 cmd_lower[i] = tolower(cmd_buf[i]);
             }
 
             /** Command Handling */
             if (strncmp(cmd_lower, "help", 4) == 0) {
                 // Show help message
                 printf("Command List:\r\n");
                 printf("  spindle <0-100>       Set spindle speed (%%)\r\n");
                 printf("  x+N / x-N             Move X axis (y/z similar)\r\n");
                 printf("  M5                    Turn spindle OFF\r\n");
                 printf("  enable on/off         Enable or disable stepper drivers\r\n");
                 printf("  info                  Show current status\r\n");
                 printf("  help                  Show this help\r\n");
                 printf("  G0 / G1 X<n> Y<n>     Move to absolute X/Y position (e.g., G1 X100 Y100)\r\n");
                 printf("  M350 X<n> Y<n> Z<n>   Set microstepping mode (n = 1, 2, 4, 8, 16, 32)\r\n");
                 printf("  G92                   Set current position to zero (any axis)\r\n");
                 printf("Right click to paste your clipboard G-Code file into the terminal and ENSURE YOU PRESS ENTER\r\n");
             } /** "engrave" command will take a text file pasted into the serial port window and 
                   commence serial port file reading to the RP2040 */
            //  else if (strncmp(cmd_lower, "engrave", 7) == 0) {
            //     printf("Starting test engraving...\n");
            //     gcode_process_line("M3 S80");
            //     gcode_process_line("G1 X20 Y10");  // JUST DEMO G CODE, DELETE WHEN REDUNDANT 
            //     gcode_process_line("G1 X10 Y10");  
            //     gcode_process_line("M5");
            //  }
              else if (strncmp(cmd_lower, "info", 4) == 0) {
                printf("\r\n--- Machine Status ---\r\n");
                 // Show motor positions and spindle speed
                 StepperMotor mx = motor_get_status(AXIS_X);
                 StepperMotor my = motor_get_status(AXIS_Y);
                 StepperMotor mz = motor_get_status(AXIS_Z);
                 printf("Current position: X=%d, Y=%d, Z=%d (steps)\r\n", mx.position, my.position, mz.position);
                 printf("Spindle speed: %d%%\r\n", spindle_get_speed());
                 printf("Stepper speed range: %d~%d steps/sec (current delay = %d µs)\r\n",
       MIN_STEPPER_SPEED, MAX_STEPPER_SPEED, motor_get_step_delay());
                 printf("Microstepping: X=1/%d, Y=1/%d, Z=1/16 (fixed)\r\n",
                 mx.microstep_mode, my.microstep_mode);
 

             } else if (strncmp(cmd_lower, "enable", 6) == 0) {
                 // Enable or disable drivers
                 if (strstr(cmd_lower, "on") != NULL || strstr(cmd_lower, "1") != NULL) {
                     motor_enable(true);
                     printf("Stepper drivers ENABLED (ENABLE=LOW)\r\n");
                 } else if (strstr(cmd_lower, "off") != NULL || strstr(cmd_lower, "0") != NULL) {
                     motor_enable(false);
                     printf("Stepper drivers DISABLED (ENABLE=HIGH)\r\n");
                 } else {
                     printf("Usage: enable on/off or 1/0\r\n");
                 }
             } else if (strncmp(cmd_lower, "spindle", 7) == 0) {
                 // Set spindle speed
                 int speed_val;
                 if (sscanf(cmd_lower, "spindle %d", &speed_val) == 1) {
                     if (speed_val < 0) speed_val = 0;
                     if (speed_val > 100) speed_val = 100;
                     spindle_set_speed((uint8_t)speed_val);
                     printf("Spindle speed set to %d%%\r\n", speed_val);
                 } else {
                     printf("Usage: spindle <speed%% (0-100)>\r\n");
                 }
             } else {
                 // Try to parse shorthand 'x+100" type axis movement commands (x, y, z)
                 char axis = cmd_lower[0];
                 if ((axis == 'x' || axis == 'y' || axis == 'z') &&
                     (cmd_lower[1] == '+' || cmd_lower[1] == '-' || isdigit((uint)cmd_lower[1]))) {
 
                    // Relative axis movement
                     char *num_ptr;
                     int move_steps;
                     if (cmd_lower[1] == '+' || cmd_lower[1] == '-') {
                         num_ptr = &cmd_lower[2];
                         move_steps = atoi(num_ptr);
                         if (cmd_lower[1] == '-') move_steps = -move_steps;
                     } else {
                         num_ptr = &cmd_lower[1];
                         move_steps = atoi(num_ptr);
                     }
 
                     AxisIndex ax = (axis == 'x' ? AXIS_X : (axis == 'y' ? AXIS_Y : AXIS_Z));
                     motor_move_steps(ax, move_steps);
                     StepperMotor m = motor_get_status(ax);
printf("%c axis moved %d steps, current position = %d\r\n",
       toupper(axis), move_steps, m.position);

                 } else {
                    // If not axis shorthand, try full G-code file parsing
                    if (!gcode_process_line(cmd_buf)) {  // needs to be able to be evaluated as a boolean
                        printf("Unknown command: %s\r\n", cmd_buf);
                    }
                }
            }
 
             // Reset buffer for next command
             buf_index = 0;
             cmd_buf[0] = '\0';
             printf("> ");
         } else {
             // Append character to command buffer
             if (buf_index < sizeof(cmd_buf) - 1) {
                 cmd_buf[buf_index++] = c;
             } else {
                 // Buffer overflow, reset
                 buf_index = 0;
                 cmd_buf[0] = '\0';
                 printf("\r\nCommand too long. Reset. Please try again.\r\n> ");
             }
         }
     } // while(true)
 
     return 0;
 }
 