

#pragma once
#ifdef CONFIG_ARCH_CHIP_STM32F405RG
    #include "../../../nuttx/src/tc/stm/stm32f4/include/tc_arch/micro_hal.h"
#elif CONFIG_ARCH_CHIP_STM32H743ZI
    #include "../../../nuttx/src/tc/stm/stm32h7/include/tc_arch/micro_hal.h"
#else
    #include <tc_arch/micro_hal.h>
#endif
