#include "PseudoRandomNumberGenerator.h"

#ifndef PC_VISUALIZATION
using RndDataType = uint16_t;
pseudo_rng::PseudoRandomNumberGenerator<RndDataType> RndGen(255, 3, 97);
#endif

namespace pseudo_rng {

#ifndef PC_VISUALIZATION
void ADC1_2_IRQHandler() {
    uint16_t data = RndGen.m_ADC->DR;
    constexpr const uint8_t typeBitSize = sizeof(RndDataType) * 8;

    do {
    if (RndGen.m_SampleBit > typeBitSize-2) {
        RndGen.m_SampleBit = 0;
        ++RndGen.m_SampleIter;
        if (RndGen.m_SampleIter > 2) {
            // Finished for three items, exit
            RndGen.m_ADCSamplingStatus = 1;
            break;
        }
    }

    // use few adc sample bits
    RndGen.m_Samples[RndGen.m_SampleIter] |= static_cast<RndDataType>((data >> 9) & 0x03) << RndGen.m_SampleBit;
    RndGen.m_SampleBit += 2;

    // fire next sampling
    RndGen.m_ADC->CR2 |= ADC_CR2_ADON;
    } while (0) ;

    RndGen.m_ADCSamplingStatus = 1;
    NVIC_ClearPendingIRQ(ADC1_2_IRQn);
}
#endif

} // namespace
