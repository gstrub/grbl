#include "grbl.h"

/** Communication avec la bungard
 *  La communication se fait via un port parallèle standard (data+strobe+status)
 *  D7 définit si l'on commande les PàP ou si l'on modifie l'un des registres de configuration
 *  D6 choisit le registre de configuration "latch1" ou "latch2"
 */

/* Latch1 = IC3 (40174)
 *  D0-D3 : spindle speed (30k - 60k RPM)
 *  D4    : spindle enable. Does a startup cycle, also controls the vacuum relay.
 *  D5    : ?
 */
#define BEL_LATCH1_WRITE_MASK 0x80
#define BEL_SPINDLE_SPEED_MASK  0xF
#define BEL_SPINDLE_ENABLE_MASK 0x10
static uint8_t latch1 = 0; // Cache value of latch1

/* Latch2 = IC4 (40174)
 *  D0    : ?
 *  D1    : spindle brake
 *  D2-D3 : microstepping settings
 *  D4    : steppers enable
 *  D5    : ?
 */
#define BEL_LATCH2_WRITE_MASK 0xC0
#define BEL_SPINDLE_BRAKE_MASK  0x2
#define BEL_STEPPERS_MICROSTEP_MASK  0xC
#define BEL_STEPPERS_ENABLE_MASK 0x10
static uint8_t latch2 = 0; // Cache value of latch2

#define BEL_LATCH1 0
#define BEL_LATCH2 1

static uint8_t stepper_limits_override_flag = 0;

#define nop() __asm__("nop\n\t")
void bel_write_byte(uint8_t data)
{
  // as these bitwise operations are quite slow, we do not need additional delays...
  PORTB = (PORTB & 0xFC) | data >> 6;
  PORTD = (PORTD & 0x3) | data << 2;
  // at least 0.5us per lpt spec
  nop();nop();nop();nop();
  // at least 0.5us per lpt spec 
  PORTB |= 0x20;
  nop();nop();nop();nop();
  PORTB &= ~0x20;
}

void bel_write_latch(uint8_t latch, uint8_t data)
{
  // as these bitwise operations are quite slow, we do not need additional delays...
  PORTB = (PORTB & 0xFC) | latch | 0x2;
  PORTD = (PORTD & 0x3) | data << 2;
  // at least 0.5us per lpt spec
  nop();nop();nop();nop();
  // at least 0.5us per lpt spec
  PORTB |= 0x20;
  nop();nop();nop();nop();
  PORTB &= ~0x20;
}

void bel_init()
{
  DDRD |= 0xFC;
  DDRB |= 0x23;
}

void bel_set_steppers_enable(bool enable)
{
  latch2 &= ~BEL_STEPPERS_MICROSTEP_MASK;
 // latch2 |= 0x2b;
  if (enable)
    latch2 |= BEL_STEPPERS_ENABLE_MASK;
  else
    latch2 &= ~BEL_STEPPERS_ENABLE_MASK;
  
  bel_write_latch(BEL_LATCH2, latch2);
}

void bel_steppers_step(uint8_t data)
{
  bel_write_byte(data & 0x3F | stepper_limits_override_flag);
}

void bel_set_steppers_limit_override_enable(bool enable)
{
  if (enable)
    stepper_limits_override_flag = 0x40;
  else
    stepper_limits_override_flag = 0;
}


void bel_set_spindle_enable(bool enable)
{
  uint8_t _latch = latch1;
 if (enable)
    _latch |= BEL_SPINDLE_ENABLE_MASK;
  else
    _latch &= ~BEL_SPINDLE_ENABLE_MASK;
  if (_latch != latch1)
  {
    latch1 = _latch;
    bel_write_latch(BEL_LATCH1, latch1);
  }
}

void bel_set_spindle_speed(uint8_t speed)
{
  uint8_t _latch = latch1 & ~BEL_SPINDLE_SPEED_MASK;
  _latch |= (speed & BEL_SPINDLE_SPEED_MASK) | BEL_SPINDLE_ENABLE_MASK;
  if (_latch != latch1)
  {
    latch1 = _latch;
    bel_write_latch(BEL_LATCH1, latch1);
  }
}

void bel_set_spindle_brake(bool enable)
{
 /* if (enable)
    latch2 |= BEL_SPINDLE_BRAKE_MASK;
  else
    latch2 &= ~BEL_SPINDLE_BRAKE_MASK;
  bel_write_byte(BEL_LATCH2_WRITE_MASK | latch2 & 0x3F);*/
}


bool bel_get_spindle_enable()
{
  return latch1 & BEL_SPINDLE_ENABLE_MASK;
}

