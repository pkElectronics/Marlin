/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
 * Copyright (c) 2017 Victor Perez
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#include "../platforms.h"

#ifdef __PLAT_RP2040__

#include "HAL.h"
//#include "usb_serial.h" 

#include "../../inc/MarlinConfig.h"
#include "../shared/Delay.h"

extern "C" {
  #include "pico/bootrom.h"
  #include "hardware/watchdog.h"
}



#ifdef USBCON
  DefaultSerial1 MSerial0(false, SerialUSB);
#endif

#if ENABLED(SRAM_EEPROM_EMULATION)
  #if STM32F7xx
    #include <stm32f7xx_ll_pwr.h>
  #elif STM32F4xx
    #include <stm32f4xx_ll_pwr.h>
  #else
    #error "SRAM_EEPROM_EMULATION is currently only supported for STM32F4xx and STM32F7xx"
  #endif
#endif

#if HAS_SD_HOST_DRIVE
  #include "msc_sd.h"
  #include "usbd_cdc_if.h"
#endif

// ------------------------
// Public Variables
// ------------------------

volatile uint16_t HAL_adc_result;

// ------------------------
// Public functions
// ------------------------

TERN_(POSTMORTEM_DEBUGGING, extern void install_min_serial());

// HAL initialization task
void HAL_init() {
  // Ensure F_CPU is a constant expression.
  // If the compiler breaks here, it means that delay code that should compute at compile time will not work.
  // So better safe than sorry here.
  constexpr int cpuFreq = F_CPU;
  UNUSED(cpuFreq);

  #if ENABLED(SDSUPPORT) && DISABLED(SDIO_SUPPORT) && (defined(SDSS) && SDSS != -1)
    OUT_WRITE(SDSS, HIGH); // Try to set SDSS inactive before any other SPI users start up
  #endif

  #if PIN_EXISTS(LED)
    OUT_WRITE(LED_PIN, LOW);
  #endif

  #if ENABLED(SRAM_EEPROM_EMULATION)
    // __HAL_RCC_PWR_CLK_ENABLE();
    // HAL_PWR_EnableBkUpAccess();           // Enable access to backup SRAM
    // __HAL_RCC_BKPSRAM_CLK_ENABLE();
    // LL_PWR_EnableBkUpRegulator();         // Enable backup regulator
    // while (!LL_PWR_IsActiveFlag_BRR());   // Wait until backup regulator is initialized
  #endif

  HAL_timer_init();

  #if ENABLED(EMERGENCY_PARSER) && USBD_USE_CDC
    USB_Hook_init();
  #endif

  TERN_(POSTMORTEM_DEBUGGING, install_min_serial());    // Install the min serial handler

  TERN_(HAS_SD_HOST_DRIVE, MSC_SD_init());              // Enable USB SD card access

  #if PIN_EXISTS(USB_CONNECT)
    OUT_WRITE(USB_CONNECT_PIN, !USB_CONNECT_INVERTING); // USB clear connection
    delay(1000);                                        // Give OS time to notice
    WRITE(USB_CONNECT_PIN, USB_CONNECT_INVERTING);
  #endif
}

// HAL idle task
void HAL_idletask() {
  #if HAS_SHARED_MEDIA
    // Stm32duino currently doesn't have a "loop/idle" method
    // CDC_resume_receive();
    // CDC_continue_transmit();
  #endif
}

void HAL_clear_reset_source() { } // Nothing to do

uint8_t HAL_get_reset_source() { 
  byte result = 0;
 if(watchdog_caused_reboot() ) result |= RST_WATCHDOG;

  return result;
}

void HAL_reboot() { reset_usb_boot(0, 0); }

void _delay_ms(const int delay_ms) { delay(delay_ms); }

extern "C" {
  extern unsigned int _ebss; // end of bss section
}

// ------------------------
// ADC
// ------------------------

volatile uint8_t HAL_adc_conversion_state = 0;

void HAL_adc_isr() {
  adc_run(false); //disable as we only want one result
  irq_clear(ADC_IRQ_FIFO); //clear the irq

  if (adc_fifo_get_level() >= 1  ){
    HAL_adc_result = adc_fifo_get(); //pop the result
    adc_fifo_drain();
    HAL_adc_conversion_state = 1; //signal the end of the conversion
  }
}

void HAL_adc_init() { 
  analogReadResolution(HAL_ADC_RESOLUTION); 
  adc_init();
  adc_fifo_setup(true,false,1,false,false);
  irq_set_exclusive_handler(ADC_IRQ_FIFO, HAL_adc_isr);
  irq_set_enabled(ADC_IRQ_FIFO, true);
  adc_irq_set_enabled(true);

}

void HAL_adc_select_pin(const uint8_t adc_pin) {
  if (adc_pin >= A0 && adc_pin <= A3) {
    adc_gpio_init(adc_pin);
  }
  else if (adc_pin == HAL_ADC_MCU_TEMP_DUMMY_PIN) {
    adc_set_temp_sensor_enabled(true);
  }
}

void HAL_adc_start_conversion(const uint8_t adc_pin) { 
  HAL_adc_conversion_state = 0;

  if (adc_pin != HAL_ADC_MCU_TEMP_DUMMY_PIN){
    adc_select_input(adc_pin-A0); //ADC Channel is Offset from the GPIO Channel
  }
  else {
    adc_select_input(4);
  }
  
 // adc_fifo_drain();
  adc_run(true);
}
uint16_t HAL_adc_get_result() { return HAL_adc_result; }

uint8_t HAL_adc_conversion_done() {
  return HAL_adc_conversion_state;
}

// ------------------------


// Reset the system to initiate a firmware flash
void flashFirmware(const int16_t) { HAL_reboot(); }

// Maple Compatibility
volatile uint32_t systick_uptime_millis = 0;
systickCallback_t systick_user_callback;
void systick_attach_callback(systickCallback_t cb) { systick_user_callback = cb; }
void HAL_SYSTICK_Callback() {
  systick_uptime_millis++;
  if (systick_user_callback) systick_user_callback();
}

#endif // __PLAT_RP2040__
