#include <init.h>


#ifdef CONFIG_NETWORKING
#include <stm32_ll_gpio.h>
/*
При включении CONFIG_CLOCK_CONTROL, драйвер Zephyr (v2.7.99) инициализирует все
нужные регистры Master Clock Output 1 (MCO1), кроме записи в регистр
GPIOA->MODER8 (регистр для настройки выхода PA8) режима Alternate function mode.
Поэтому запись производится вручную ниже. MCO1 - PA8 - Ethernet PHY clock в
SOVA427; MCO2 - PC9 - не используется в SOVA427;
*/
static int mco_pinPA8_init(const struct device *port) {
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
  return 0;
}
SYS_INIT(mco_pinPA8_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
#endif