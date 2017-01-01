#ifndef STM32F10X_H
#define STM32F10X_H

#define __enable_irq()
#define DMA1_Channel7_IRQn 16
#define NVIC_SetPriority(x, y)
#define NVIC_ClearPendingIRQ(x)
#define NVIC_EnableIRQ(x)

#include <cstdint>


using ADC_TypeDef = uint32_t*;


#endif // STM32F10X_H
