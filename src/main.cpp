//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "Timer.h"
#include "BlinkLed.h"
#include "WS2813Leds.h"
#include "RingBuffer.hpp"

#include "AnimationContext.h"
#include "PseudoRandomDotsAnimation.hpp"
#include "PseudoRandomNumberGenerator.h"
#include "FallingLightAnimation.hpp"
#include "MovingPixelsAnimation.hpp"
#include "FallingSnowAnimation.hpp"
#include "WalkingDotsAnimation.hpp"


extern WS2813Leds<LEDS_COUNT> ledStrip1;
extern LedsTransmission ws2813TransLayer;
// todo fix uint16_t
extern pseudo_rng::PseudoRandomNumberGenerator<uint16_t> RndGen;

// ----------------------------------------------------------------------------
//
// Standalone STM32F1 led blink sample (trace via NONE).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// Then demonstrates how to blink a led with 1 Hz, using a
// continuous loop and SysTick delays.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the NONE output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=8000000.
//
// Note: The default clock settings take the user defined HSE_VALUE and try
// to reach the maximum possible system clock. For the default 8 MHz input
// the result is guaranteed, but for other values it might not be possible,
// so please adjust the PLL settings in system/src/cmsis/system_stm32f10x.c
//

// Definitions visible only within this translation unit.
namespace
{
  // ----- Timing definitions -------------------------------------------------

  // Keep the LED on for 2/3 of a second.
  constexpr Timer::ticks_t BLINK_ON_TICKS = Timer::FREQUENCY_HZ * 1 / 2;
  constexpr Timer::ticks_t BLINK_OFF_TICKS = (Timer::FREQUENCY_HZ
      - BLINK_ON_TICKS) * 3;
}

#define BITBAND_SRAM(addr, bit) ((uint32_t*)(SRAM_BB_BASE + ((((uint32_t)addr) - SRAM_BASE) << 5) + ((bit) << 2)))
volatile static uint8_t spiLedBuff[12]; //1B data -> 4B SPI need to send, 12 bytes for one LED

static uint32_t* pSpiG7 = BITBAND_SRAM(&spiLedBuff[0], 6);
static uint32_t* pSpiG6 = BITBAND_SRAM(&spiLedBuff[0], 2);
static uint32_t* pSpiG5 = BITBAND_SRAM(&spiLedBuff[1], 6);
static uint32_t* pSpiG4 = BITBAND_SRAM(&spiLedBuff[1], 2);
static uint32_t* pSpiG3 = BITBAND_SRAM(&spiLedBuff[2], 6);
static uint32_t* pSpiG2 = BITBAND_SRAM(&spiLedBuff[2], 2);
static uint32_t* pSpiG1 = BITBAND_SRAM(&spiLedBuff[3], 6);
static uint32_t* pSpiG0 = BITBAND_SRAM(&spiLedBuff[3], 2);

static uint32_t* pSpiR7 = BITBAND_SRAM(&spiLedBuff[4], 6);
static uint32_t* pSpiR6 = BITBAND_SRAM(&spiLedBuff[4], 2);
static uint32_t* pSpiR5 = BITBAND_SRAM(&spiLedBuff[5], 6);
static uint32_t* pSpiR4 = BITBAND_SRAM(&spiLedBuff[5], 2);
static uint32_t* pSpiR3 = BITBAND_SRAM(&spiLedBuff[6], 6);
static uint32_t* pSpiR2 = BITBAND_SRAM(&spiLedBuff[6], 2);
static uint32_t* pSpiR1 = BITBAND_SRAM(&spiLedBuff[7], 6);
static uint32_t* pSpiR0 = BITBAND_SRAM(&spiLedBuff[7], 2);

static uint32_t* pSpiB7 = BITBAND_SRAM(&spiLedBuff[8], 6);
static uint32_t* pSpiB6 = BITBAND_SRAM(&spiLedBuff[8], 2);
static uint32_t* pSpiB5 = BITBAND_SRAM(&spiLedBuff[9], 6);
static uint32_t* pSpiB4 = BITBAND_SRAM(&spiLedBuff[9], 2);
static uint32_t* pSpiB3 = BITBAND_SRAM(&spiLedBuff[10], 6);
static uint32_t* pSpiB2 = BITBAND_SRAM(&spiLedBuff[10], 2);
static uint32_t* pSpiB1 = BITBAND_SRAM(&spiLedBuff[11], 6);
static uint32_t* pSpiB0 = BITBAND_SRAM(&spiLedBuff[11], 2);

#ifdef __cplusplus
extern "C" {
#endif
//    void ADC1_2_IRQHandler();
    void USART1_IRQHandler();
#ifdef __cplusplus
}
#endif


// initializes buffer with frame
void initSpiLedBuff() {
    for (size_t i = 0; i < sizeof(spiLedBuff)/sizeof(spiLedBuff[0]); ++i) {
        spiLedBuff[i] = 0b10001000;
    }
}

void convertRGB(const uint8_t red, const uint8_t green, const uint8_t blue) {
    *pSpiG7 = green >> 7;
    *pSpiG6 = green >> 6;
    *pSpiG5 = green >> 5;
    *pSpiG4 = green >> 4;
    *pSpiG3 = green >> 3;
    *pSpiG2 = green >> 2;
    *pSpiG1 = green >> 1;
    *pSpiG0 = green;

    *pSpiR7 = red >> 7;
    *pSpiR6 = red >> 6;
    *pSpiR5 = red >> 5;
    *pSpiR4 = red >> 4;
    *pSpiR3 = red >> 3;
    *pSpiR2 = red >> 2;
    *pSpiR1 = red >> 1;
    *pSpiR0 = red;

    *pSpiB7 = blue >> 7;
    *pSpiB6 = blue >> 6;
    *pSpiB5 = blue >> 5;
    *pSpiB4 = blue >> 4;
    *pSpiB3 = blue >> 3;
    *pSpiB2 = blue >> 2;
    *pSpiB1 = blue >> 1;
    *pSpiB0 = blue;
}


void sendSpiBuffer() {
    for (size_t i = 0; i < sizeof(spiLedBuff)/sizeof(spiLedBuff[0]); ++i) {
        while((SPI1->SR & SPI_SR_TXE)==0) { ; }
            SPI1->DR = spiLedBuff[i];
    }
}

void initDMA() {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel2->CCR = 0;
    DMA1_Channel2->CNDTR = 1;
    DMA1_Channel2->CPAR = (uint32_t)&GPIOB->IDR;
    DMA1_Channel2->CMAR = (uint32_t)&TIM2->CNT;
    DMA1_Channel2->CCR = DMA_CCR1_MSIZE_0 | DMA_CCR1_CIRC | DMA_CCR1_EN;// DMA_CCR_MSIZE_0 | DMA_CCR_CIRC |DMA_CCR_EN;

}

void initTIM() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->CR1 = 0;
    TIM2->SMCR = 0x0056;//0x0056;
    TIM2->PSC = 17;//17;
    TIM2->ARR = 3; //3;
//    TIM2->CNT = 99;
    TIM2->CCR3 = 2;
    TIM2->CCR4 = 2;
    TIM2->CCMR1 = 0x0001;
    TIM2->CCMR2 = 0x6000;
    TIM2->CCER = 0x1001;
    TIM2->DIER = 0x0800;
    TIM2->CR1 = TIM_CR1_OPM | TIM_CR1_DIR | TIM_CR1_CEN;
}

void initSPI() {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

//    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;

    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_CPHA;
    SPI1->CR2 |= SPI_CR2_SSOE;
    SPI1->CR1 |= SPI_CR1_SPE;
//    SPI1->CR1 |= SPI_CR1_SPE;
}
void initGPIOS() {
    // For SPI1:
    // PA5 - SCK
    // PA6 - MISO
    // PA7 - MOSI
    // TIM2 channels: 1: PA0, 2: PA1, 3 PA2, 4 PA3
    // INPUT - PB0

//    RCC->AHBENR |= RCC_AHBENR_GPIOFEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;

    // use alternative functions
//    GPIOA->MODER = 0x2828A8A4;     // PA5,6,7 AF
    GPIOA->CRL &= ~(  GPIO_CRL_MODE5 | GPIO_CRL_CNF5
                    | GPIO_CRL_MODE6 | GPIO_CRL_CNF6
                    | GPIO_CRL_MODE7 | GPIO_CRL_CNF7
                    | GPIO_CRL_MODE0 | GPIO_CRL_CNF0
                    | GPIO_CRL_MODE1 | GPIO_CRL_CNF1 // PA1 analog
                    | GPIO_CRL_MODE2 | GPIO_CRL_CNF2
                    | GPIO_CRL_MODE3 | GPIO_CRL_CNF3
                    );
    GPIOA->CRH &= ~( GPIO_CRH_MODE9 | GPIO_CRH_CNF9 /* P9 -> usart1 tx */
                    );

    GPIOA->CRL |= GPIO_CRL_MODE5 | GPIO_CRL_CNF5_1 /* PA5 AF, 50 MHz, push-pull */
               | GPIO_CRL_MODE7 | GPIO_CRL_CNF7_1 // PA7 - MOSI
               | GPIO_CRL_CNF6_0 // MISO should be as input, so zero value are ok, but set input type as floating
               // Not sure that this will work if I make connection bridge connection (use reistors for safety)
               | GPIO_CRL_CNF0_0 // PA0 - floating input
               //| GPIO_CRL_CNF1_0 // PA1 - analog
               | GPIO_CRL_CNF2_0 // PA2 - floating input
               | GPIO_CRL_MODE3 //| GPIO_CRL_CNF3_1 // PA3 - output drive the LEDS
               ;

    GPIOA->CRH |= GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1; /* PA9 -> usart1 tx AF, 50 MHz, push-pull */

//    GPIOA->CRL |= GPIO_CRL_MODE5 | GPIO_CRL_CNF5_1 /* PA5 AF, 50 MHz, push-pull */
//               | GPIO_CRL_MODE7 | GPIO_CRL_CNF7_1 // PA7 - MOSI
//               | GPIO_CRL_MODE6 | GPIO_CRL_CNF6_1 // MISO should be as input, so zero value are ok, but set input type as floating
//               | GPIO_CRL_MODE4 | GPIO_CRL_CNF4_1
//               // Not sure that this will work if I make connection bridge connection (use reistors for safety)
//               | GPIO_CRL_CNF0_0 // PA0 - floating input
//               | GPIO_CRL_CNF1_0 // PA1 - floating input
//               | GPIO_CRL_CNF2_0 // PA2 - floating input
//               | GPIO_CRL_MODE3 | GPIO_CRL_CNF3_1 // PA3 - output drive the LEDS
//               ;


//    GPIOA->ODR = 0;

    // AF remaps
    //AFIO->MAPR = 0; // SPI1, TIM2 requires 0 (no remap) at its bits

    // PB0 as input, other as output
    GPIOB->CRL = 0x22242224;
    GPIOB->CRH = 0x22222222;
    GPIOB->ODR = 0;


//    GPIOA->PUPDR = 0x24140141;
//    GPIOA->ODR = 0x00000000;
//    GPIOA->AFR[0] = 0x01001100;
//    GPIOA->AFR[1] = 0x00000440;


//    GPIOB->AFR[0] = 0x00000010;
//    GPIOB->MODER = 0x00000008;
//
//    GPIOF->ODR = 0x00000000;
//    GPIOF->MODER = 0x00000004;
}

void initUSART1() {
    __enable_irq();
    NVIC_SetPriority(USART1_IRQn, 7);
    NVIC_ClearPendingIRQ(USART1_IRQn);
    NVIC_EnableIRQ(USART1_IRQn);

    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // baudrate 19200, apb2 clk = 48MHz
    USART1->BRR = 0;
    USART1->BRR |= (USART_BRR_DIV_Mantissa & ((uint16_t)156 << 4)) |
                    (USART_BRR_DIV_Fraction & ((uint16_t)4));

    USART1->CR2 = 0;

    USART1->CR3 = 0;

    USART1->CR1 = 0;
    USART1->CR1 |= USART_CR1_TE | USART_CR1_UE | USART_CR1_TXEIE;
}

//void initADC() {
//    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
//    asm("nop");
//    // set adcpre to 6 (48MHz/6 = 8MHz)
//    RCC->CFGR |= RCC_CFGR_ADCPRE_1;
//    RCC->CFGR &= ~(RCC_CFGR_ADCPRE_0);
//
//    ADC1->CR1 = 0;
//    ADC1->CR1 |= ADC_CR1_SCAN | ADC_CR1_EOCIE;
//
//    ADC1->CR2 = 0;
//    ADC1->CR2 |= ADC_CR2_CAL;
//
//    asm("nop");
//    asm("nop");
//    asm("nop");
////    ADC1->CR2 |= ADC_CR2_ADON;
//
//    ADC1->SMPR2 |= ADC_SMPR2_SMP1_2 | ADC_SMPR2_SMP1_0;
//
//    ADC1->SQR3 |= ADC_SQR3_SQ1_0; // channel 1
//
//    __enable_irq();
//    NVIC_SetPriority(ADC1_2_IRQn, 7);
//    NVIC_ClearPendingIRQ(ADC1_2_IRQn);
//    NVIC_EnableIRQ(ADC1_2_IRQn);
//
//}

RingBuffer<uint8_t, 32> uartRingBuffer;
volatile uint8_t adc_bit = 0;
volatile uint8_t rnddata = 0;


void USART1_IRQHandler() {

    uint8_t data;
    if (true == uartRingBuffer.get(data)) {
        USART1->DR = data;
    } else {
        // disable TX
        USART1->CR1 &= ~USART_CR1_TXEIE;
    }

    NVIC_ClearPendingIRQ(USART1_IRQn);
}

void spiTx(uint8_t data) {
//    if ((SPI1->SR & SPI_SR_RXNE) == true)
//        uint16_t data = SPI1->DR;
//
//    if ((SPI1->SR & SPI_SR_MODF) == true)
//        SPI1->SR |= SPI_SR_MODF;
//    if ((SPI1->SR & SPI_SR_OVR) == true)
//        SPI1->SR |= SPI_SR_OVR;

    while((SPI1->SR & SPI_SR_TXE)==0) { }
    SPI1->DR = data;
}




void movingLed() {
    Timer timer;
    RGBColor movingColor{60, 0, 200};
    const RGBColor backgroundColor{0,0,0};

    constexpr const auto delay = 6; // ms

    for (int jj = 0; jj < 5; ++jj) {
        if (jj > 0) {
            movingColor.r = RndGen.randomize() % 240;
            movingColor.g = RndGen.randomize() % 180;
            movingColor.b = RndGen.randomize() % 80;
        }
        //move forward
        for (std::size_t i = 0; i < LEDS_COUNT; ++i) {

            // clear turned on LED before
            if (i > 0) {
                ledStrip1.fill(i-1, i, backgroundColor);
            }
            // set next led
            ledStrip1.fill(i, i+1, movingColor);

            while(false == ledStrip1.send()) {;}
            timer.sleep(delay);
        }

        timer.sleep(500);
        // moving back
        for (int32_t i = LEDS_COUNT-2; i >= 1; --i ) {
            // clear turned on LED before

            // set next led
            ledStrip1.fill(i, i+1, movingColor);

            ledStrip1.fill(i+1, i+2, backgroundColor);


            while(false == ledStrip1.send()) {;}
            timer.sleep(delay);
        }
    }

    // moving back
}


// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

//#define BITBAND(addr,bitnum) ((addr & 0xF0000000)+((addr & 0xFFFFF)<<5)+(bitnum<<2))
//#define MEM_ADDR(addr) *((volatile unsigned long*) (addr))



int main(int argc, char* argv[])
{

  volatile uint32_t destData = 0u;

//  MEM_ADDR(BITBAND(reinterpret_cast<unsigned long>(&destData), 0)) = 3;
  uint8_t data = 0b00001010;

  uint32_t* bit  = BITBAND_SRAM(&destData, 0);
  uint32_t* bit1 = BITBAND_SRAM(&destData, 1);
  uint32_t* bit2 = BITBAND_SRAM(&destData, 2);
  uint32_t* bit3 = BITBAND_SRAM(&destData, 3);
  *bit = data >> 0;
  *bit1 = data >> 1;
  *bit2 = data >> 2;
  *bit3 = data >> 3;

  constexpr const uint8_t ws2bits = 0b10001100; // sends bit 0, and bit 1

  Timer timer;
  timer.start();

  BlinkLed blinkLed;

  // Perform all necessary initialisations for the LED.
  blinkLed.powerUp();

  uint32_t seconds = 0;

  initGPIOS();
  initUSART1();
//  initDMA();
//  initTIM();
//  initSPI();


  initSpiLedBuff();
  convertRGB(0x01, 0x02, 0x04);
  // Infinite loop

  ledStrip1.registerTransmissionLayer(&ws2813TransLayer);

  animations::AnimationContext animationContext(RndGen);
  animations::PseudoRandomDotsAnimation dotsAnim(timer, ledStrip1);
  animations::FallingLightAnimation fallingLight(timer, ledStrip1);
  animations::MovingPixelsAnimation movingPixels(timer, ledStrip1);
  animations::FallingSnowAnimation fallingSnow(timer, ledStrip1);
  animations::WalkingDotsAnimation walkingDots(timer, ledStrip1);
  animationContext.registerAnimation(&movingPixels);
  animationContext.registerAnimation(&walkingDots);
  animationContext.registerAnimation(&fallingSnow);
  animationContext.registerAnimation(&dotsAnim);
  animationContext.registerAnimation(&fallingLight);




  volatile uint16_t t = 0;
  RndGen.initADC();
  for (volatile int i = 0; i < 2; ++i) {
      t = RndGen.randomize();
//      RndGen.fireADC();
//      RndGen.seedWithADCSamples();
  }
  t += 3;


  while (1)
    {
      animationContext.runNext();
      timer.sleep(200);


      blinkLed.turnOn();
      timer.sleep(50);//seconds== 0 ? Timer::FREQUENCY_HZ : BLINK_ON_TICKS);
//      GPIOA->BSRR |= GPIO_BSRR_BR5;

      blinkLed.turnOff();
      timer.sleep(100); //BLINK_OFF_TICKS);

      ++seconds;

//      sendSpiBuffer();

//      movingLed();

//      timer.sleep(500);

//      ledStrip1.fill(0, 3, {128,0,0});
//      ledStrip1.fill(3, 6, {0,128,0});
//      ledStrip1.fill(6, 9, {0,0,128});
//      while (false == ledStrip1.send());

//      timer.sleep(500);
//      timer.sleep(10);

//      ledStrip1.setMarkerColor(RGBColor {128,0,0});
//      ledStrip1.fill(0,2);
//      ledStrip1.fill(3,5);
//      ledStrip1.fill(0,3, {0,0,0});
//
//      while (false == ledStrip1.send()) {;}

    }
  // Infinite loop, never return.
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
