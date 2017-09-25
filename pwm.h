#ifndef _PWM_H_
#define _PWM_H_ 1

#include "registers.h"

void pwm_registers_defaults(void);
void pwm_init(void);
void pwm_update(uint16_t position, int16_t pwm);
void pwm_stop(void);

inline static void pwm_enable(void)
{
    uint8_t flags_lo = registers_read_byte(REG_FLAGS_LO);

    // Enable PWM to the  motor.
    registers_write_byte(REG_FLAGS_LO, flags_lo | (1<<FLAGS_LO_PWM_ENABLED));
}


inline static void pwm_disable(void)
{
    uint8_t flags_lo = registers_read_byte(REG_FLAGS_LO);

    // Disable PWM to the  motor.
    registers_write_byte(REG_FLAGS_LO, flags_lo & ~(1<<FLAGS_LO_PWM_ENABLED));

    // Stop now!
    pwm_stop();
}
void motor_up(void);
void motor_down(void);
#endif // _PWM_H_