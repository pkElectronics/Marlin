#include <stdint.h>

#include "../../core/macros.h"

// ------------------------
// Defines
// ------------------------

//#define _HAL_TIMER(T) _CAT(LPC_TIM, T)
//#define _HAL_TIMER_IRQ(T) TIMER##T##_IRQn
//#define __HAL_TIMER_ISR(T) extern "C" alarm_callback_t HAL_timer_alarm_pool_##T##_callback()
#define __HAL_TIMER_ISR(T) extern void HAL_timer_##T##_callback()
#define _HAL_TIMER_ISR(T)  __HAL_TIMER_ISR(T)


typedef uint64_t hal_timer_t;
#define HAL_TIMER_TYPE_MAX 0xFFFFFFFF

#define HAL_TIMER_RATE         (1000000)  // fixed value as we use a microsecond timesource
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
#define STEPPER_TIMER_TICKS_PER_US (1) // fixed value as we use a microsecond timesource
#define STEPPER_TIMER_PRESCALE (1)

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
//#define STEP_TIMER_PTR _HAL_TIMER(STEP_TIMER_NUM)
//#define TEMP_TIMER_PTR _HAL_TIMER(TEMP_TIMER_NUM)

extern alarm_pool_t* HAL_timer_pool_0;
extern alarm_pool_t* HAL_timer_pool_1;
extern alarm_pool_t* HAL_timer_pool_2;
extern alarm_pool_t* HAL_timer_pool_3;

extern void HAL_timer_0_callback();
extern void HAL_timer_1_callback();
extern void HAL_timer_2_callback();
extern void HAL_timer_3_callback();

extern int64_t HAL_timer_alarm_pool_0_callback(long int, void*);
extern int64_t HAL_timer_alarm_pool_1_callback(long int, void*);
extern int64_t HAL_timer_alarm_pool_2_callback(long int, void*);
extern int64_t HAL_timer_alarm_pool_3_callback(long int, void*);

// ------------------------
// Public functions
// ------------------------
void HAL_timer_init();
void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency);
void HAL_timer_stop(const uint8_t timer_num);

FORCE_INLINE static void HAL_timer_set_compare(const uint8_t timer_num, const hal_timer_t compare) {
  HAL_timer_stop(timer_num);

  switch (timer_num) {
    case 0:
      alarm_pool_add_alarm_in_us(HAL_timer_pool_0 ,compare , HAL_timer_alarm_pool_0_callback ,0 ,false );
      break;
    
    case 1:
      alarm_pool_add_alarm_in_us(HAL_timer_pool_1 ,compare , HAL_timer_alarm_pool_1_callback ,0 ,false );
      break;

    case 2:
      alarm_pool_add_alarm_in_us(HAL_timer_pool_2 ,compare , HAL_timer_alarm_pool_2_callback ,0 ,false );
      break;

    case 3:
      alarm_pool_add_alarm_in_us(HAL_timer_pool_3 ,compare , HAL_timer_alarm_pool_3_callback ,0 ,false );
      break;
  }
}

FORCE_INLINE static hal_timer_t HAL_timer_get_compare(const uint8_t timer_num) {
  return 0;
}

FORCE_INLINE static hal_timer_t HAL_timer_get_count(const uint8_t timer_num) {
  return time_us_64();
}

FORCE_INLINE static void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  irq_set_enabled(timer_num,true); //lucky coincidence that timer_num and rp2040 irq num matches
}

FORCE_INLINE static void HAL_timer_disable_interrupt(const uint8_t timer_num) {
    irq_set_enabled(timer_num,false); //lucky coincidence that timer_num and rp2040 irq num matches
}

FORCE_INLINE static bool HAL_timer_interrupt_enabled(const uint8_t timer_num) {
  return irq_is_enabled(timer_num); //lucky coincidence that timer_num and rp2040 irq num matches
}

FORCE_INLINE static void HAL_timer_isr_prologue(const uint8_t timer_num) {
  irq_clear(timer_num);
}

#define HAL_timer_isr_epilogue(TIMER_NUM)

//private function
