#ifndef _BUNGARD_H
#define _BUNGARD_H

void bel_write_byte(uint8_t data);
void bel_init();
void bel_set_steppers_enable(bool enable);
void bel_steppers_step(uint8_t data);
void bel_set_steppers_limit_override_enable(bool enable);
void bel_set_spindle_enable(bool enable);
void bel_set_spindle_speed(uint8_t speed);
void bel_set_spindle_brake(bool enable);
bool bel_get_spindle_enable();


#endif

