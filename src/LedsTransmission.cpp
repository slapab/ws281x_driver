#include <utility>              // std::swap
#include "stm32f10x.h"
#include "LedsTransmission.h"
#include "WS2813Leds.h"

#ifndef PC_VISUALIZATION
extern WS2813Leds<LEDS_COUNT> ledStrip1;
LedsTransmission ws2813TransLayer(ledStrip1);
#endif

#define BITBAND_SRAM(addr, bit) ((volatile uint32_t*)(SRAM_BB_BASE + ((((uint32_t)addr) - SRAM_BASE) << 5) + ((bit) << 2)))


void LedsTransmission::initSPI() {
#ifndef PC_VISUALIZATION

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR1 = 0;
    SPI1->CR2 = 0;
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_CPHA;
    SPI1->CR2 |= SPI_CR2_SSOE | SPI_CR2_TXDMAEN;
    SPI1->CR1 |= SPI_CR1_SPE;

#endif
}

void LedsTransmission::initDMA() {
#ifndef PC_VISUALIZATION
    // todo fix hardcoded irq number

    // initiate the DMA
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel3->CCR = 0;

    // peripheral to memory, 8bit -> 8bit
    DMA1_Channel3->CCR |= DMA_CCR3_TCIE | DMA_CCR3_MINC | DMA_CCR3_DIR | DMA_CCR3_PL_1;
    DMA1_Channel3->CNDTR = BUFFER_SIZE;

    DMA1_Channel3->CPAR = reinterpret_cast<std::uintptr_t>(&SPI1->DR);
//    DMA1_Channel6->CMAR = reinterpret_cast<std::uintptr_t>(m_pCurrBuffer);

    NVIC_SetPriority(DMA1_Channel3_IRQn, 6);
    NVIC_ClearPendingIRQ(DMA1_Channel3_IRQn);
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);

#endif
}


bool LedsTransmission::beginTransmision(const std::sig_atomic_t transferSize) {
    bool retVal = false;

#ifndef PC_VISUALIZATION
    // detect if transaction is in progress
    if ((TransmissionStatus::FINISHED == m_TransmissionStatus) &&
        (0 != (SPI1->SR & SPI_SR_TXE)) && (0 == (SPI1->SR & SPI_SR_BSY))) {

        convertFirstTwoLeds();

        // disable channel before updating its registers
        DMA1_Channel3->CCR &= ~DMA_CCR3_EN;

        m_TransmissionStatus = TransmissionStatus::IN_PROGRESS;

        // set up the transfer size
        m_TransferSize = transferSize - 1;

        // set data length
        DMA1_Channel3->CNDTR = LedsTransmission::BUFFER_SIZE;
        // write the source address
        DMA1_Channel3->CMAR = reinterpret_cast<std::uintptr_t>(&m_ConvBuffers[m_currBuffIdx][0]);

        // enable the DMA transaction
        DMA1_Channel3->CCR |= DMA_CCR3_EN;


        retVal = true;
    }
#else
    (void)transferSize;
#endif

    return retVal;
}


#ifndef PC_VISUALIZATION
void DMA1_Channel3_IRQHandler() {
    // Clear interrupt flags
    GPIOA->BSRR |= GPIO_BSRR_BS3;

    NVIC_ClearPendingIRQ(DMA1_Channel3_IRQn);
    DMA1->IFCR |= DMA_IFCR_CGIF3;
    // disable channel before updating its registers
    DMA1_Channel3->CCR &= ~DMA_CCR3_EN;

    if (ws2813TransLayer.m_TransferSize > 0) {
        --ws2813TransLayer.m_TransferSize;

        // assumptions is taken: current buffer is just sent, and the next has already
        // converted data, so swap buffers (next with current) and start transmission
        std::swap(ws2813TransLayer.m_currBuffIdx, ws2813TransLayer.m_nextBuffIdx);

        // fire the DMA
        DMA1_Channel3->CNDTR = LedsTransmission::BUFFER_SIZE;
        DMA1_Channel3->CMAR = reinterpret_cast<std::uintptr_t>(&ws2813TransLayer.m_ConvBuffers[ws2813TransLayer.m_currBuffIdx][0]);
        DMA1_Channel3->CCR |= DMA_CCR3_EN;

        RGBColor color = ws2813TransLayer.m_LedsStrip.getNextToTransmit();
        auto* bitNextBuffPtrs = ws2813TransLayer.m_bitBandPtrs[ws2813TransLayer.m_nextBuffIdx];
        // convert into next buffer
        *bitNextBuffPtrs[0]  = color.g >> 7;
        *bitNextBuffPtrs[1]  = color.g >> 6;
        *bitNextBuffPtrs[2]  = color.g >> 5;
        *bitNextBuffPtrs[3]  = color.g >> 4;
        *bitNextBuffPtrs[4]  = color.g >> 3;
        *bitNextBuffPtrs[5]  = color.g >> 2;
        *bitNextBuffPtrs[6]  = color.g >> 1;
        *bitNextBuffPtrs[7]  = color.g;

        *bitNextBuffPtrs[8]  = color.r >> 7;
        *bitNextBuffPtrs[9]  = color.r >> 6;
        *bitNextBuffPtrs[10] = color.r >> 5;
        *bitNextBuffPtrs[11] = color.r >> 4;
        *bitNextBuffPtrs[12] = color.r >> 3;
        *bitNextBuffPtrs[13] = color.r >> 2;
        *bitNextBuffPtrs[14] = color.r >> 1;
        *bitNextBuffPtrs[15] = color.r;

        *bitNextBuffPtrs[16] = color.b >> 7;
        *bitNextBuffPtrs[17] = color.b >> 6;
        *bitNextBuffPtrs[18] = color.b >> 5;
        *bitNextBuffPtrs[19] = color.b >> 4;
        *bitNextBuffPtrs[20] = color.b >> 3;
        *bitNextBuffPtrs[21] = color.b >> 2;
        *bitNextBuffPtrs[22] = color.b >> 1;
        *bitNextBuffPtrs[23] = color.b;
    }
//    else if (0 != ws2813TransLayer.m_TransmittingFirstLed) {
//        ws2813TransLayer.m_TransmittingFirstLed = 0;
//    }
    else {
        ws2813TransLayer.m_TransmissionStatus = LedsTransmission::TransmissionStatus::FINISHED;
        GPIOA->BSRR |= GPIO_BSRR_BR3;
            asm("nop");
            asm("nop");
            asm("nop");
            asm("nop");
            asm("nop");
        GPIOA->BSRR |= GPIO_BSRR_BS3;
    }

    GPIOA->BSRR |= GPIO_BSRR_BR3;
}
#endif

void LedsTransmission::initBindBandPointers() {
#ifndef PC_VISUALIZATION

    for (uint8_t i = 0; i < BUFFERS_NO; ++i) {
        for (uint8_t buffByte = 0, bitBandBit = 0; buffByte < BUFFER_SIZE; ++buffByte, bitBandBit += 2) {
            m_bitBandPtrs[i][bitBandBit] = BITBAND_SRAM(&m_ConvBuffers[i][buffByte], 6);
            m_bitBandPtrs[i][bitBandBit+1] = BITBAND_SRAM(&m_ConvBuffers[i][buffByte], 2);

            // assign template value into each cell of buffer
            m_ConvBuffers[i][buffByte] = 0b10001000;
        }
    }

#endif
}

void LedsTransmission::convertFirstTwoLeds() {
#ifndef PC_VISUALIZATION

    const uint8_t indexes[] = {m_currBuffIdx, m_nextBuffIdx};
    // convert two LEDs, first convert into current buffer, second into next buffer
    for (auto i = 0; i < 2; ++i) {
        if (true == m_LedsStrip.hasNextToTransmit()) {
            RGBColor color = m_LedsStrip.getNextToTransmit();
            auto* bitNextBuffPtrs = ws2813TransLayer.m_bitBandPtrs[indexes[i]];
            // convert into next buffer
            *bitNextBuffPtrs[0]  = color.g >> 7;
            *bitNextBuffPtrs[1]  = color.g >> 6;
            *bitNextBuffPtrs[2]  = color.g >> 5;
            *bitNextBuffPtrs[3]  = color.g >> 4;
            *bitNextBuffPtrs[4]  = color.g >> 3;
            *bitNextBuffPtrs[5]  = color.g >> 2;
            *bitNextBuffPtrs[6]  = color.g >> 1;
            *bitNextBuffPtrs[7]  = color.g;

            *bitNextBuffPtrs[8]  = color.r >> 7;
            *bitNextBuffPtrs[9]  = color.r >> 6;
            *bitNextBuffPtrs[10] = color.r >> 5;
            *bitNextBuffPtrs[11] = color.r >> 4;
            *bitNextBuffPtrs[12] = color.r >> 3;
            *bitNextBuffPtrs[13] = color.r >> 2;
            *bitNextBuffPtrs[14] = color.r >> 1;
            *bitNextBuffPtrs[15] = color.r;

            *bitNextBuffPtrs[16] = color.b >> 7;
            *bitNextBuffPtrs[17] = color.b >> 6;
            *bitNextBuffPtrs[18] = color.b >> 5;
            *bitNextBuffPtrs[19] = color.b >> 4;
            *bitNextBuffPtrs[20] = color.b >> 3;
            *bitNextBuffPtrs[21] = color.b >> 2;
            *bitNextBuffPtrs[22] = color.b >> 1;
            *bitNextBuffPtrs[23] = color.b;
        }
    }

#endif
}
