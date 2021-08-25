#include <stdint.h>

#include "../../core/macros.h"

// ------------------------
// Defines
// ------------------------

#define _HAL_TIMER(T) _CAT(LPC_TIM, T)
#define _HAL_TIMER_IRQ(T) TIMER##T##_IRQn
#define __HAL_TIMER_ISR(T) extern "C" void TIMER##T##_IRQHandler()
#define _HAL_TIMER_ISR(T)  __HAL_TIMER_ISR(T)

typedef uint32_t hal_timer_t;
#define HAL_TIMER_TYPE_MAX 0xFFFFFFFF

#define HAL_TIMER_RATE         ((F_CPU) / 4)  // frequency of timers peripherals

#ifndef STEP_TIMER_NUM
  #define STEP_TIMER_NUM        0  // Timer Index for Stepper
#endif
#ifndef PULSE_TIMER_NUM
  #define PULSE_TIMER_NUM       STEP_TIMER_NUM
#endif
#ifndef TEMP_TIMER_NUM
  #define TEMP_TIMER_NUM        1  // Timer Index for Temperature
#endif
#ifndef PWM_TIMER_NUM
  #define PWM_TIMER_NUM         3  // Timer Index for PWM
#endif

#define TEMP_TIMER_RATE        1000000
#define TEMP_TIMER_FREQUENCY   1000 // temperature interrupt frequency

#define STEPPER_TIMER_RATE     HAL_TIMER_RATE   // frequency of stepper timer (HAL_TIMER_RATE / STEPPER_TIMER_PRESCALE)
#define STEPPER_TIMER_TICKS_PER_US ((STEPPER_TIMER_RATE) / 1000000) // stepper timer ticks per µs
#define STEPPER_TIMER_PRESCALE (CYCLES_PER_MICROSECOND / STEPPER_TIMER_TICKS_PER_US)

#define PULSE_TIMER_RATE       STEPPER_TIMER_RATE   // frequency of pulse timer
#define PULSE_TIMER_PRESCALE   STEPPER_TIMER_PRESCALE
#define PULSE_TIMER_TICKS_PER_US STEPPER_TIMER_TICKS_PER_US

#define ENABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_enable_interrupt(STEP_TIMER_NUM)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_disable_interrupt(STEP_TIMER_NUM)
#define STEPPER_ISR_ENABLED() HAL_timer_interrupt_enabled(STEP_TIMER_NUM)

#define ENABLE_TEMPERATURE_INTERRUPT() HAL_timer_enable_interrupt(TEMP_TIMER_NUM)
#define DISABLE_TEMPERATURE_INTERRUPT() HAL_timer_disable_interrupt(TEMP_TIMER_NUM)

#ifndef HAL_STEP_TIMER_ISR
  #define HAL_STEP_TIMER_ISR() _HAL_TIMER_ISR(STEP_TIMER_NUM)
#endif
#ifndef HAL_TEMP_TIMER_ISR
  #define HAL_TEMP_TIMER_ISR() _HAL_TIMER_ISR(TEMP_TIMER_NUM)
#endif

// Timer references by index
#define STEP_TIMER_PTR _HAL_TIMER(STEP_TIMER_NUM)
#define TEMP_TIMER_PTR _HAL_TIMER(TEMP_TIMER_NUM)

// ------------------------
// Public functions
// ------------------------
void HAL_timer_init();
void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency);

FORCE_INLINE static void HAL_timer_set_compare(const uint8_t timer_num, const hal_timer_t compare) {
}

FORCE_INLINE static hal_timer_t HAL_timer_get_compare(const uint8_t timer_num) {
  return 0;
}

FORCE_INLINE static hal_timer_t HAL_timer_get_count(const uint8_t timer_num) {
  return 0;
}

FORCE_INLINE static void HAL_timer_enable_interrupt(const uint8_t timer_num) {
}

FORCE_INLINE static void HAL_timer_disable_interrupt(const uint8_t timer_num) {
}

FORCE_INLINE static bool HAL_timer_interrupt_enabled(const uint8_t timer_num) {
  return false;
}

FORCE_INLINE static void HAL_timer_isr_prologue(const uint8_t timer_num) {

}

#define HAL_timer_isr_epilogue(TIMER_NUM)
