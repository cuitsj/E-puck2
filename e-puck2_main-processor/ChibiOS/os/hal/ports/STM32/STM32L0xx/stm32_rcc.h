/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    STM32L0xx/stm32_rcc.h
 * @brief   RCC helper driver header.
 * @note    This file requires definitions from the ST header file
 *          @p stm32l0xx.h.
 *
 * @addtogroup STM32L1xx_RCC
 * @{
 */

#ifndef _STM32_RCC_
#define _STM32_RCC_

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Generic RCC operations
 * @{
 */
/**
 * @brief   Enables the clock of one or more peripheral on the APB1 bus.
 *
 * @param[in] mask      APB1 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAPB1(mask, lp) {                                           \
  RCC->APB1ENR |= (mask);                                                   \
  if (lp)                                                                   \
    RCC->APB1SMENR |= (mask);                                               \
}

/**
 * @brief   Disables the clock of one or more peripheral on the APB1 bus.
 *
 * @param[in] mask      APB1 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccDisableAPB1(mask, lp) {                                          \
  RCC->APB1ENR &= ~(mask);                                                  \
  if (lp)                                                                   \
    RCC->APB1SMENR &= ~(mask);                                              \
}

/**
 * @brief   Resets one or more peripheral on the APB1 bus.
 *
 * @param[in] mask      APB1 peripherals mask
 *
 * @api
 */
#define rccResetAPB1(mask) {                                                \
  RCC->APB1RSTR |= (mask);                                                  \
  RCC->APB1RSTR = 0;                                                        \
}

/**
 * @brief   Enables the clock of one or more peripheral on the APB2 bus.
 *
 * @param[in] mask      APB2 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAPB2(mask, lp) {                                           \
  RCC->APB2ENR |= (mask);                                                   \
  if (lp)                                                                   \
    RCC->APB2SMENR |= (mask);                                               \
}

/**
 * @brief   Disables the clock of one or more peripheral on the APB2 bus.
 *
 * @param[in] mask      APB2 peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccDisableAPB2(mask, lp) {                                          \
  RCC->APB2ENR &= ~(mask);                                                  \
  if (lp)                                                                   \
    RCC->APB2SMENR &= ~(mask);                                              \
}

/**
 * @brief   Resets one or more peripheral on the APB2 bus.
 *
 * @param[in] mask      APB2 peripherals mask
 *
 * @api
 */
#define rccResetAPB2(mask) {                                                \
  RCC->APB2RSTR |= (mask);                                                  \
  RCC->APB2RSTR = 0;                                                        \
}

/**
 * @brief   Enables the clock of one or more peripheral on the AHB bus.
 *
 * @param[in] mask      AHB peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableAHB(mask, lp) {                                            \
  RCC->AHBENR |= (mask);                                                    \
  if (lp)                                                                   \
    RCC->AHBSMENR |= (mask);                                                \
}

/**
 * @brief   Disables the clock of one or more peripheral on the AHB bus.
 *
 * @param[in] mask      AHB peripherals mask
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccDisableAHB(mask, lp) {                                           \
  RCC->AHBENR &= ~(mask);                                                   \
  if (lp)                                                                   \
    RCC->AHBSMENR &= ~(mask);                                               \
}

/**
 * @brief   Resets one or more peripheral on the AHB bus.
 *
 * @param[in] mask      AHB peripherals mask
 *
 * @api
 */
#define rccResetAHB(mask) {                                                 \
  RCC->AHBRSTR |= (mask);                                                   \
  RCC->AHBRSTR = 0;                                                         \
}
/** @} */

/**
 * @name    ADC peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Enables the ADC1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableADC1(lp) rccEnableAPB2(RCC_APB2ENR_ADC1EN, lp)

/**
 * @brief   Disables the ADC1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccDisableADC1(lp) rccDisableAPB2(RCC_APB2ENR_ADC1EN, lp)

/**
 * @brief   Resets the ADC1 peripheral.
 *
 * @api
 */
#define rccResetADC1() rccResetAPB2(RCC_APB2RSTR_ADC1RST)
/** @} */

/**
 * @name    DAC peripheral specific RCC operations
 * @{
 */
/**
 * @brief   Enables the DAC1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableDAC1(lp) rccEnableAPB1(RCC_APB1ENR_DACEN, lp)

/**
 * @brief   Disables the DAC1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccDisableDAC1(lp) rccDisableAPB1(RCC_APB1ENR_DACEN, lp)

/**
 * @brief   Resets the DAC1 peripheral.
 *
 * @api
 */
#define rccResetDAC1() rccResetAPB1(RCC_APB1RSTR_DACRST)
/** @} */

/**
 * @name    DMA peripheral specific RCC operations
 * @{
 */
/**
 * @brief   Enables the DMA1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableDMA1(lp) rccEnableAHB(RCC_AHBENR_DMA1EN, lp)

/**
 * @brief   Disables the DMA1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccDisableDMA1(lp) rccDisableAHB(RCC_AHBENR_DMA1EN, lp)

/**
 * @brief   Resets the DMA1 peripheral.
 *
 * @api
 */
#define rccResetDMA1() rccResetAHB(RCC_AHBRSTR_DMA1RST)
/** @} */

/**
 * @name    PWR interface specific RCC operations
 * @{
 */
/**
 * @brief   Enables the PWR interface clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnablePWRInterface(lp) rccEnableAPB1(RCC_APB1ENR_PWREN, lp)

/**
 * @brief   Disables PWR interface clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccDisablePWRInterface(lp) rccDisableAPB1(RCC_APB1ENR_PWREN, lp)

/**
 * @brief   Resets the PWR interface.
 *
 * @api
 */
#define rccResetPWRInterface() rccResetAPB1(RCC_APB1RSTR_PWRRST)
/** @} */

/**
 * @name    I2C peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Enables the I2C1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableI2C1(lp) rccEnableAPB1(RCC_APB1ENR_I2C1EN, lp)

/**
 * @brief   Disables the I2C1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccDisableI2C1(lp) rccDisableAPB1(RCC_APB1ENR_I2C1EN, lp)

/**
 * @brief   Resets the I2C1 peripheral.
 *
 * @api
 */
#define rccResetI2C1() rccResetAPB1(RCC_APB1RSTR_I2C1RST)

/**
 * @brief   Enables the I2C2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableI2C2(lp) rccEnableAPB1(RCC_APB1ENR_I2C2EN, lp)

/**
 * @brief   Disables the I2C2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccDisableI2C2(lp) rccDisableAPB1(RCC_APB1ENR_I2C2EN, lp)

/**
 * @brief   Resets the I2C2 peripheral.
 *
 * @api
 */
#define rccResetI2C2() rccResetAPB1(RCC_APB1RSTR_I2C2RST)
/** @} */

/**
 * @name    SPI peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Enables the SPI1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableSPI1(lp) rccEnableAPB2(RCC_APB2ENR_SPI1EN, lp)

/**
 * @brief   Disables the SPI1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccDisableSPI1(lp) rccDisableAPB2(RCC_APB2ENR_SPI1EN, lp)

/**
 * @brief   Resets the SPI1 peripheral.
 *
 * @api
 */
#define rccResetSPI1() rccResetAPB2(RCC_APB2RSTR_SPI1RST)

/**
 * @brief   Enables the SPI2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableSPI2(lp) rccEnableAPB1(RCC_APB1ENR_SPI2EN, lp)

/**
 * @brief   Disables the SPI2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccDisableSPI2(lp) rccDisableAPB1(RCC_APB1ENR_SPI2EN, lp)

/**
 * @brief   Resets the SPI2 peripheral.
 *
 * @api
 */
#define rccResetSPI2() rccResetAPB1(RCC_APB1RSTR_SPI2RST)
/** @} */

/**
 * @name    TIM peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Enables the TIM2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM2(lp) rccEnableAPB1(RCC_APB1ENR_TIM2EN, lp)

/**
 * @brief   Disables the TIM2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccDisableTIM2(lp) rccDisableAPB1(RCC_APB1ENR_TIM2EN, lp)

/**
 * @brief   Resets the TIM2 peripheral.
 *
 * @api
 */
#define rccResetTIM2() rccResetAPB1(RCC_APB1RSTR_TIM2RST)
/**
 * @brief   Enables the TIM6 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM6(lp) rccEnableAPB1(RCC_APB1ENR_TIM6EN, lp)

/**
 * @brief   Disables the TIM6 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccDisableTIM6(lp) rccDisableAPB1(RCC_APB1ENR_TIM6EN, lp)

/**
 * @brief   Resets the TIM6 peripheral.
 *
 * @api
 */
#define rccResetTIM6() rccResetAPB1(RCC_APB1RSTR_TIM6RST)

/**
 * @brief   Enables the TIM21 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM21(lp) rccEnableAPB2(RCC_APB2ENR_TIM21EN, lp)

/**
 * @brief   Disables the TIM21 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccDisableTIM21(lp) rccDisableAPB2(RCC_APB2ENR_TIM21EN, lp)

/**
 * @brief   Resets the TIM21 peripheral.
 *
 * @api
 */
#define rccResetTIM21() rccResetAPB2(RCC_APB2RSTR_TIM21RST)

/**
 * @brief   Enables the TIM22 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableTIM22(lp) rccEnableAPB2(RCC_APB2ENR_TIM22EN, lp)

/**
 * @brief   Disables the TIM22 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccDisableTIM22(lp) rccDisableAPB2(RCC_APB2ENR_TIM22EN, lp)

/**
 * @brief   Resets the TIM22 peripheral.
 *
 * @api
 */
#define rccResetTIM22() rccResetAPB2(RCC_APB2RSTR_TIM22RST)
/** @} */

/**
 * @name    USART/UART peripherals specific RCC operations
 * @{
 */
/**
 * @brief   Enables the USART1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUSART1(lp) rccEnableAPB2(RCC_APB2ENR_USART1EN, lp)

/**
 * @brief   Disables the USART1 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccDisableUSART1(lp) rccDisableAPB2(RCC_APB2ENR_USART1EN, lp)

/**
 * @brief   Resets the USART1 peripheral.
 *
 * @api
 */
#define rccResetUSART1() rccResetAPB2(RCC_APB2RSTR_USART1RST)

/**
 * @brief   Enables the USART2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUSART2(lp) rccEnableAPB1(RCC_APB1ENR_USART2EN, lp)

/**
 * @brief   Disables the USART2 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccDisableUSART2(lp) rccDisableAPB1(RCC_APB1ENR_USART2EN, lp)

/**
 * @brief   Resets the USART2 peripheral.
 *
 * @api
 */
#define rccResetUSART2() rccResetAPB1(RCC_APB1RSTR_USART2RST)

/**
 * @brief   Enables the USART3 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUSART3(lp) rccEnableAPB1(RCC_APB1ENR_USART3EN, lp)

/**
 * @brief   Disables the USART3 peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccDisableUSART3(lp) rccDisableAPB1(RCC_APB1ENR_USART3EN, lp)

/**
 * @brief   Resets the USART3 peripheral.
 *
 * @api
 */
#define rccResetUSART3() rccResetAPB1(RCC_APB1RSTR_USART3RST)
/** @} */

/**
 * @name    USB peripheral specific RCC operations
 * @{
 */
/**
 * @brief   Enables the USB peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccEnableUSB(lp) rccEnableAPB1(RCC_APB1ENR_USBEN, lp)

/**
 * @brief   Disables the USB peripheral clock.
 *
 * @param[in] lp        low power enable flag
 *
 * @api
 */
#define rccDisableUSB(lp) rccDisableAPB1(RCC_APB1ENR_USBEN, lp)

/**
 * @brief   Resets the USB peripheral.
 *
 * @api
 */
#define rccResetUSB() rccResetAPB1(RCC_APB1RSTR_USBRST)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
}
#endif

#endif /* _STM32_RCC_ */

/** @} */
