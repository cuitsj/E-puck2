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
 * @file    STM32F3xx/ext_lld_isr.c
 * @brief   STM32F3xx EXT subsystem low level driver ISR code.
 *
 * @addtogroup EXT
 * @{
 */

#include "hal.h"

#if HAL_USE_EXT || defined(__DOXYGEN__)

#include "ext_lld_isr.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if !defined(STM32_DISABLE_EXTI0_HANDLER)
/**
 * @brief   EXTI[0] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector58) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR & EXTI->IMR & (1 << 0);
  EXTI->PR = pr;
  if (pr & (1 << 0))
    EXTD1.config->channels[0].cb(&EXTD1, 0);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(STM32_DISABLE_EXTI1_HANDLER)
/**
 * @brief   EXTI[1] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector5C) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR & EXTI->IMR & (1 << 1);
  EXTI->PR = pr;
  if (pr & (1 << 1))
    EXTD1.config->channels[1].cb(&EXTD1, 1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(STM32_DISABLE_EXTI2_HANDLER)
/**
 * @brief   EXTI[2] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector60) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR & EXTI->IMR & (1 << 2);
  EXTI->PR = pr;
  if (pr & (1 << 2))
    EXTD1.config->channels[2].cb(&EXTD1, 2);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(STM32_DISABLE_EXTI3_HANDLER)
/**
 * @brief   EXTI[3] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector64) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR & EXTI->IMR & (1 << 3);
  EXTI->PR = pr;
  if (pr & (1 << 3))
    EXTD1.config->channels[3].cb(&EXTD1, 3);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(STM32_DISABLE_EXTI4_HANDLER)
/**
 * @brief   EXTI[4] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector68) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR & EXTI->IMR & (1 << 4);
  EXTI->PR = pr;
  if (pr & (1 << 4))
    EXTD1.config->channels[4].cb(&EXTD1, 4);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(STM32_DISABLE_EXTI5_9_HANDLER)
/**
 * @brief   EXTI[5]...EXTI[9] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector9C) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR & EXTI->IMR & ((1 << 5) | (1 << 6) | (1 << 7) | (1 << 8) |
                               (1 << 9));
  EXTI->PR = pr;
  if (pr & (1 << 5))
    EXTD1.config->channels[5].cb(&EXTD1, 5);
  if (pr & (1 << 6))
    EXTD1.config->channels[6].cb(&EXTD1, 6);
  if (pr & (1 << 7))
    EXTD1.config->channels[7].cb(&EXTD1, 7);
  if (pr & (1 << 8))
    EXTD1.config->channels[8].cb(&EXTD1, 8);
  if (pr & (1 << 9))
    EXTD1.config->channels[9].cb(&EXTD1, 9);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(STM32_DISABLE_EXTI10_15_HANDLER)
/**
 * @brief   EXTI[10]...EXTI[15] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(VectorE0) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR & EXTI->IMR & ((1 << 10) | (1 << 11) | (1 << 12) | (1 << 13) |
                               (1 << 14) | (1 << 15));
  EXTI->PR = pr;
  if (pr & (1 << 10))
    EXTD1.config->channels[10].cb(&EXTD1, 10);
  if (pr & (1 << 11))
    EXTD1.config->channels[11].cb(&EXTD1, 11);
  if (pr & (1 << 12))
    EXTD1.config->channels[12].cb(&EXTD1, 12);
  if (pr & (1 << 13))
    EXTD1.config->channels[13].cb(&EXTD1, 13);
  if (pr & (1 << 14))
    EXTD1.config->channels[14].cb(&EXTD1, 14);
  if (pr & (1 << 15))
    EXTD1.config->channels[15].cb(&EXTD1, 15);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(STM32_DISABLE_EXTI16_HANDLER)
/**
 * @brief   EXTI[16] interrupt handler (PVD).
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector44) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR & EXTI->IMR & (1 << 16);
  EXTI->PR = pr;
  if (pr & (1 << 16))
    EXTD1.config->channels[16].cb(&EXTD1, 16);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(STM32_DISABLE_EXTI17_HANDLER)
/**
 * @brief   EXTI[17] interrupt handler (RTC Alarm).
 *
 * @isr
 */
OSAL_IRQ_HANDLER(VectorE4) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR & EXTI->IMR & (1 << 17);
  EXTI->PR = pr;
  if (pr & (1 << 17))
    EXTD1.config->channels[17].cb(&EXTD1, 17);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(STM32_DISABLE_EXTI18_HANDLER) && STM32_HAS_USB
/**
 * @brief   EXTI[18] interrupt handler (USB Wakeup).
 *
 * @isr
 */
OSAL_IRQ_HANDLER(VectorE8) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR & EXTI->IMR & (1 << 18);
  EXTI->PR = pr;
  if (pr & (1 << 18))
    EXTD1.config->channels[18].cb(&EXTD1, 18);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(STM32_DISABLE_EXTI19_HANDLER)
/**
 * @brief   EXTI[19] interrupt handler (Tamper TimeStamp).
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector48) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR & EXTI->IMR & (1 << 19);
  EXTI->PR = pr;
  if (pr & (1 << 19))
    EXTD1.config->channels[19].cb(&EXTD1, 19);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(STM32_DISABLE_EXTI20_HANDLER)
/**
 * @brief   EXTI[20] interrupt handler (RTC Wakeup).
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector4C) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR & EXTI->IMR & (1 << 20);
  EXTI->PR = pr;
  if (pr & (1 << 20))
    EXTD1.config->channels[20].cb(&EXTD1, 20);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(STM32_DISABLE_EXTI21_22_29_HANDLER)
/**
 * @brief   EXTI[21],EXTI[22],EXTI[29] interrupt handler (COMP1, COMP2, COMP3).
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector140) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR & EXTI->IMR & ((1 << 21) | (1 << 22) | (1 << 29));
  EXTI->PR = pr;
  if (pr & (1 << 21))
    EXTD1.config->channels[21].cb(&EXTD1, 21);
  if (pr & (1 << 22))
    EXTD1.config->channels[22].cb(&EXTD1, 22);
  if (pr & (1 << 29))
    EXTD1.config->channels[29].cb(&EXTD1, 29);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(STM32_DISABLE_EXTI30_32_HANDLER)
/**
 * @brief   EXTI[30]...EXTI[32] interrupt handler (COMP4, COMP5, COMP6).
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector144) {
  uint32_t pr;

  OSAL_IRQ_PROLOGUE();

  pr = EXTI->PR & EXTI->IMR & ((1 << 30) | (1 << 31));
  EXTI->PR = pr;
  if (pr & (1 << 30))
    EXTD1.config->channels[30].cb(&EXTD1, 30);
  if (pr & (1 << 31))
    EXTD1.config->channels[31].cb(&EXTD1, 31);

  pr = EXTI->PR2 & EXTI->IMR2 & (1 << 0);
  EXTI->PR2 = pr;
  if (pr & (1 << 0))
    EXTD1.config->channels[32].cb(&EXTD1, 32);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(STM32_DISABLE_EXTI33_HANDLER)
/**
 * @brief   EXTI[33] interrupt handler (COMP7).
 *
 * @isr
 */
OSAL_IRQ_HANDLER(RTC_WKUP_IRQHandler) {
  uint32_t pr2;

  OSAL_IRQ_PROLOGUE();

  pr2 = EXTI->PR2 & EXTI->IMR & (1 << 1);
  EXTI->PR2 = pr2;
  if (pr2 & (1 << 1))
    EXTD1.config->channels[33].cb(&EXTD1, 33);

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Enables EXTI IRQ sources.
 *
 * @notapi
 */
void ext_lld_exti_irq_enable(void) {

  nvicEnableVector(EXTI0_IRQn, STM32_EXT_EXTI0_IRQ_PRIORITY);
  nvicEnableVector(EXTI1_IRQn, STM32_EXT_EXTI1_IRQ_PRIORITY);
  nvicEnableVector(EXTI2_TSC_IRQn, STM32_EXT_EXTI2_IRQ_PRIORITY);
  nvicEnableVector(EXTI3_IRQn, STM32_EXT_EXTI3_IRQ_PRIORITY);
  nvicEnableVector(EXTI4_IRQn, STM32_EXT_EXTI4_IRQ_PRIORITY);
  nvicEnableVector(EXTI9_5_IRQn, STM32_EXT_EXTI5_9_IRQ_PRIORITY);
  nvicEnableVector(EXTI15_10_IRQn, STM32_EXT_EXTI10_15_IRQ_PRIORITY);
  nvicEnableVector(PVD_IRQn, STM32_EXT_EXTI16_IRQ_PRIORITY);
  nvicEnableVector(RTC_Alarm_IRQn, STM32_EXT_EXTI17_IRQ_PRIORITY);
#if STM32_HAS_USB
  nvicEnableVector(USBWakeUp_IRQn, STM32_EXT_EXTI18_IRQ_PRIORITY);
#endif
  nvicEnableVector(TAMP_STAMP_IRQn, STM32_EXT_EXTI19_IRQ_PRIORITY);
  nvicEnableVector(RTC_WKUP_IRQn, STM32_EXT_EXTI20_IRQ_PRIORITY);
  nvicEnableVector(COMP1_2_3_IRQn, STM32_EXT_EXTI21_22_29_IRQ_PRIORITY);
  nvicEnableVector(COMP4_5_6_IRQn, STM32_EXT_EXTI30_32_IRQ_PRIORITY);
#if STM32_EXTI_NUM_LINES >= 34
  nvicEnableVector(COMP7_IRQn, STM32_EXT_EXTI33_IRQ_PRIORITY);
#endif
}

/**
 * @brief   Disables EXTI IRQ sources.
 *
 * @notapi
 */
void ext_lld_exti_irq_disable(void) {

  nvicDisableVector(EXTI0_IRQn);
  nvicDisableVector(EXTI1_IRQn);
  nvicDisableVector(EXTI2_TSC_IRQn);
  nvicDisableVector(EXTI3_IRQn);
  nvicDisableVector(EXTI4_IRQn);
  nvicDisableVector(EXTI9_5_IRQn);
  nvicDisableVector(EXTI15_10_IRQn);
  nvicDisableVector(PVD_IRQn);
  nvicDisableVector(RTC_Alarm_IRQn);
#if STM32_HAS_USB
  nvicDisableVector(USBWakeUp_IRQn);
#endif
  nvicDisableVector(TAMP_STAMP_IRQn);
  nvicDisableVector(RTC_WKUP_IRQn);
  nvicDisableVector(COMP1_2_3_IRQn);
  nvicDisableVector(COMP4_5_6_IRQn);
#if STM32_EXTI_NUM_LINES >= 34
  nvicDisableVector(COMP7_IRQn);
#endif
}

#endif /* HAL_USE_EXT */

/** @} */
