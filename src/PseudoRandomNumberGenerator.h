#ifndef PSEUDORANDOMNUMBERGENERATOR_H_
#define PSEUDORANDOMNUMBERGENERATOR_H_
#include <csignal>
#include <algorithm>    // std::fill_n
#include "stm32f10x.h"

namespace pseudo_rng {

extern "C" void ADC1_2_IRQHandler();

/**
 * The "Type" template parameter must be simple integer type.
 *
 * @Note: based on: http://www.electro-tech-online.com/threads/ultra-fast-pseudorandom-number-generator-for-8-bit.124249/
 */
template <class Type>
class PseudoRandomNumberGenerator {
public:
    PseudoRandomNumberGenerator(const Type s1, const Type s2, const Type s3);
    PseudoRandomNumberGenerator() = delete;
    ~PseudoRandomNumberGenerator() = default;

    void seed(const Type s1, const Type s2, const Type s3);
    Type randomize();
    void fireADC();
    void seedWithADCSamples();

    friend void ADC1_2_IRQHandler();

    void initADC();

private:
    Type m_A;
    Type m_B;
    Type m_C;
    Type m_Iter;

    ADC_TypeDef* m_ADC;
    volatile Type m_Samples[3];
    volatile uint8_t m_SampleIter;
    volatile uint8_t m_SampleBit;

    volatile std::sig_atomic_t m_ADCSamplingStatus;
};


template <class Type>
inline PseudoRandomNumberGenerator<Type>::PseudoRandomNumberGenerator(const Type s1, const Type s2, const Type s3)
    : m_SampleIter(0)
    , m_SampleBit(0)
    , m_ADCSamplingStatus(0)
{
#ifndef PC_VISUALIZATION
    m_ADC = ADC1;
#endif

    seed(s1,s2,s3);
}

template <class Type>
inline void PseudoRandomNumberGenerator<Type>::seed(const Type s1, const Type s2, const Type s3) {
    //XOR new entropy into key state
    m_A ^=s1;
    m_B ^=s2;
    m_C ^=s3;

    m_Iter++;
    m_A = (m_A^m_C^m_Iter);
    m_B = (m_B+m_A);
    m_C = ((m_C+(m_B>>1))^m_A);
}


template <class Type>
inline Type PseudoRandomNumberGenerator<Type>::randomize() {
    m_Iter++;                       //m_Iter is incremented every round and is not affected by any other variable
    m_A = (m_A^m_C^m_Iter);         //note the mix of addition and XOR
    m_B = (m_B+m_A);                //And the use of very few instructions
    m_C = ((m_C+(m_B>>1))^m_A);     //the right shift is to ensure that high-order bits from m_B can affect
    return m_C;                     //low order bits of other variables
}

template <class Type>
inline void PseudoRandomNumberGenerator<Type>::initADC() {
#ifndef PC_VISUALIZATION

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    __enable_irq();
    NVIC_SetPriority(ADC1_2_IRQn, 7);
    NVIC_ClearPendingIRQ(ADC1_2_IRQn);
    NVIC_EnableIRQ(ADC1_2_IRQn);


    // set adcpre to 6 (48MHz/6 = 8MHz)
    RCC->CFGR |= RCC_CFGR_ADCPRE_1;
    RCC->CFGR &= ~(RCC_CFGR_ADCPRE_0);

    m_ADC->CR1 = 0;
    m_ADC->CR1 |= ADC_CR1_SCAN | ADC_CR1_EOCIE;

    m_ADC->CR2 = 0;
//    m_ADC->CR2 |= ADC_CR2_CAL;

    asm("nop");
    asm("nop");
    asm("nop");
    //    ADC1->CR2 |= ADC_CR2_ADON;

    m_ADC->SMPR2 |= ADC_SMPR2_SMP1_2 | ADC_SMPR2_SMP1_0;

    m_ADC->SQR3 |= ADC_SQR3_SQ1_0; // channel 1

#endif
}

template <class Type>
inline void PseudoRandomNumberGenerator<Type>::fireADC() {
#ifndef PC_VISUALIZATION

    initADC();
    m_ADCSamplingStatus = 0;
    std::fill_n(m_Samples, sizeof(m_Samples)/sizeof(m_Samples[0]), 0);
    m_SampleIter = 0;
    m_SampleBit = 0;
    m_ADC->CR2 |= ADC_CR2_ADON;

#endif
}

template <class Type>
inline void PseudoRandomNumberGenerator<Type>::seedWithADCSamples() {
    while (0 == m_ADCSamplingStatus) {;}

    Type s0 = m_Samples[0];
    Type s1;
    Type s2;
    if (s0 == m_Samples[1]) {
        seed(s0, m_B, m_C);
        s1 = randomize();
    } else {
        s1 = m_Samples[1];
    }

    if (s1 == m_Samples[2]) {
        seed(s0, s1, m_C);
        s2 = randomize();
    } else {
        s2 = m_Samples[2];
    }

    seed(s0, s1, s2);
}

} // namespace


#endif /* PSEUDORANDOMNUMBERGENERATOR_H_ */
