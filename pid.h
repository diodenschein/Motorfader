#ifndef _PID_H_
#define _OS_PID_H_ 1

// Initialize the PID algorithm module.
void pid_init(void);

// Initialize the PID related register values.
void pid_registers_defaults(void);

// Take the 10-bit position as input and output a signed PWM to be
// applied to the  motors.
int16_t pid_position_to_pwm(int16_t position);

#endif // _PID_H_
