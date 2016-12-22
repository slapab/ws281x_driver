#include <utility>          //std::swap
#include <cstdint>
#include "WS2813Leds.h"
#include "stm32f10x.h"


WS2813Leds<LEDS_COUNT> ledStrip1;

//
//RGBColor::RGBColor()
//    : LEDColor(0,0,0) { ; }
//
//RGBColor::RGBColor(uint8_t r, uint8_t g, uint8_t b)
//    : LEDColor(r,g,b) { ; }
//
//
//WS2813Leds::WS2813Leds()
//    : m_pTransmittingBuff(&m_RgbBuffers[1][0])
//    , m_pFillingBuff(&m_RgbBuffers[0][0])
//    , m_pModifiedStart(&m_RgbBuffers[0][0])
//    , m_pModifiedEnd(&m_RgbBuffers[0][0])
//    , m_pTranssmitionEnd(nullptr)
//    , m_pTransmissionNextLEDColor(nullptr)
//    , m_FillingBuffReady(FillingBuffState::Ready)
//    , m_transmissionLayer(nullptr)
//{
//
//    //todo fix hardoced irq number
//    __enable_irq();
//    NVIC_SetPriority(DMA1_Channel7_IRQn, 7);
//    NVIC_ClearPendingIRQ(DMA1_Channel7_IRQn);
//    NVIC_EnableIRQ(DMA1_Channel7_IRQn);
//
//}
//
//void WS2813Leds::setMarkerColor(RGBColor&& color) {
//    m_MarkerColor = color;
//}
//
//bool WS2813Leds::hasNextToTransmit() {
//    // m_pTranssmissionEnd points to element one after the last to send
//    return m_pTransmissionNextLEDColor < m_pTranssmitionEnd;
//}
//
//RGBColor WS2813Leds::getNextToTransmit() {
//    RGBColor retColor;
//    if (true == hasNextToTransmit()) {
//        retColor.r = *m_pTransmissionNextLEDColor;
//        retColor.g = *(m_pTransmissionNextLEDColor+1);
//        retColor.b = *(m_pTransmissionNextLEDColor+2);
//        m_pTransmissionNextLEDColor += 3;
//    }
//
//    return retColor;
//}
//
//bool WS2813Leds::swapBuffers() {
//    bool retval = false;
//
//    // If transmission is still in progress then do nothing.
//    if (nullptr != m_transmissionLayer && false == m_transmissionLayer->stillTransmitting()) {
//    do {
//        // The call to this class methods that operates on Filling buffer will
//        // block until this flag will be set to Ready by the DMA that
//        // synchronizes Transmitting and Filling buffers. After that the API
//        // can make operations on the Filling buffer.
//        m_FillingBuffReady = FillingBuffState::NotReady;
//
//        // calculate the distance of modified start in current filling buffer
//        const std::ptrdiff_t modifiedStartPos = m_pModifiedStart - m_pFillingBuff;
//        const std::ptrdiff_t modifiedEndPos = m_pModifiedEnd - m_pFillingBuff;
//
//        // Fire DMA that copies memory from start_modified to transmission_end
//        const std::int32_t modifiedSize = (modifiedEndPos - modifiedStartPos) * sizeof(m_RgbBuffers[0][0]);
//        if (modifiedSize <= 0) {
//            break;
//        }
//
//        // Note that the m_pTransmittingBuff will be the filling buffer after
//        // procedure finish.
//        startDMATransaction(modifiedSize, m_pModifiedStart, m_pTransmittingBuff + modifiedStartPos);
//
//        // set transmission_end pointer that points at right place in the
//        // transmitting buffer
//        m_pTranssmitionEnd = m_pModifiedEnd;
//        // set next led pointer to points to the first data in transmitting buffer
//        m_pTransmissionNextLEDColor = m_pFillingBuff;
//
//
//        // Make the filling buffer the transmitting one, and the transmitting
//        // make the filling buffer
//        std::swap(m_pFillingBuff, m_pTransmittingBuff);
//
//        // restore modified position in current(new) filling buffer
//        if (modifiedStartPos > 0) {
//            m_pModifiedStart = m_pFillingBuff + modifiedStartPos;
//        } else {
//            m_pModifiedStart = m_pFillingBuff;
//        }
//
//        // reset the end position in new filling buffer
//        m_pModifiedEnd = m_pFillingBuff;
//
//        // begin transmission -> it will handle the first and later conversions
//        while (false == m_transmissionLayer->beginTransmision((m_pTranssmitionEnd - m_pTransmissionNextLEDColor)/3)) { ; }
//
//        retval = true;
//    } while(0);
//    }
//
//    return retval;
//}
//
//
//void WS2813Leds::startDMATransaction(const uint16_t transferSize, const uint8_t* startAddr, const uint8_t* endAddr) {
//
//    if (0 == transferSize) {
//        return;
//    }
//
//    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
//    DMA1_Channel7->CCR = 0;
//
//    // memory to memory, start address will be in peripheral, 8bit -> 8bit
//    DMA1_Channel7->CCR |= DMA_CCR7_MEM2MEM | DMA_CCR7_TCIE | DMA_CCR7_MINC | DMA_CCR7_PINC;
//    DMA1_Channel7->CNDTR = transferSize;
//
//    DMA1_Channel7->CPAR = reinterpret_cast<std::uintptr_t>(startAddr);
//    DMA1_Channel7->CMAR = reinterpret_cast<std::uintptr_t>(endAddr);
//
//    // start the DMA transaction
//    DMA1_Channel7->CCR |= DMA_CCR7_EN;
//}



void DMA1_Channel7_IRQHandler() {
    // indicate that synchronize process of just transmitting buffer with new
    // filling buffer has ended -> this will unblock access to filling buffer
    ledStrip1.m_FillingBuffReady = WS2813Leds<155>::FillingBuffState::Ready;

    // Clear interrupt flags
    NVIC_ClearPendingIRQ(DMA1_Channel7_IRQn);
    DMA1->IFCR |= DMA_IFCR_CGIF7;
}




//void WS2813Leds::fill(uint16_t start, uint16_t end) {
//    fill(start, end, m_MarkerColor);
//}
//
//
//void WS2813Leds::fill(uint16_t start, uint16_t end, const RGBColor& color) {
//    start *= 3;
//    end *= 3;
//    int32_t placeDistance = end - start;
//    if (placeDistance <= 0) {
//        return;
//    }
//
//    //todo fix led size
//    if (end >= 150*3)
//    {
//        return;
//    }
//
//    while(FillingBuffState::Ready != m_FillingBuffReady) { ; }
//
//    // update modified_start pointer
//    std::ptrdiff_t ptrDist = m_pModifiedStart - m_pFillingBuff;
//    if (start < ptrDist && ptrDist > 0) {
//        m_pModifiedStart = m_pFillingBuff + start;
//    } else {
//        m_pModifiedStart = m_pFillingBuff;
//    }
//
//    // update transmission_end pointer
//    ptrDist = m_pModifiedEnd - m_pFillingBuff;
//    if (end > ptrDist && ptrDist >= 0) {
//        m_pModifiedEnd = m_pFillingBuff + end;
//    } else {
//        m_pModifiedEnd = m_pModifiedStart + placeDistance;
//    }
//
//    for (uint16_t i = start; i < end; i += 3) {
//        m_pFillingBuff[i] = color.r;
//        m_pFillingBuff[i+1] = color.g;
//        m_pFillingBuff[i+2] = color.b;
//    }
//}
//
//
//bool WS2813Leds::send() {
//    return swapBuffers();
//}

//void WS2813Leds::convertFirstSPIBuffer() {
//    // points to the next RGB data
//    m_pTransmissionNextLEDColor += 6; //todo
//    //todo convert data into spi transmission current buffer
//    //todo convert data into spi transmission next buffer
//}
