#ifndef WS2813LEDS_H_
#define WS2813LEDS_H_

#include <csignal>
#include <utility>
#include "stm32f10x.h"
#include "LedsTransmission.h"


#ifdef __STDC_NO_ATOMICS__
#error "No support for std::atomic"
#endif


//Interrupt handler declaration in C/C++
#ifdef __cplusplus
extern "C" {
#endif
     void DMA1_Channel7_IRQHandler();
     void DMA1_Channel3_IRQHandler();
#ifdef __cplusplus
}
#endif


constexpr const std::size_t LEDS_COUNT = 300;


class LEDColor {
public:
    LEDColor(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {;}
    virtual ~LEDColor() {};

    uint8_t r;
    uint8_t g;
    uint8_t b;
};

class RGBColor : public LEDColor {
public:
    RGBColor();
    RGBColor(uint8_t r, uint8_t g, uint8_t b);
    RGBColor& inc(const uint8_t v, const RGBColor& max);
    RGBColor& operator+=(const uint8_t d);
    RGBColor& operator-=(const uint8_t d);
    RGBColor& operator++();
    RGBColor& operator--();
    RGBColor operator- (const uint8_t d) const;

    virtual ~RGBColor() {}
};


template <std::size_t LEDS_CNT>
class WS2813Leds {
public:
    WS2813Leds();
    virtual ~WS2813Leds() {;}

    void setMarkerColor(RGBColor&& color);
    void fill(uint16_t start, uint16_t end);
    void fill(uint16_t start, uint16_t end, const RGBColor& color);
    bool moveForward(const uint16_t orgStart, const uint16_t newStart, const uint16_t len);
    bool moveBackward(const uint16_t orgStart, const uint16_t newStart, const uint16_t len);
    void clear();
    bool send();

    virtual RGBColor getNextToTransmit();

    void registerTransmissionLayer(LedsTransmission* transsmission) {
        if (nullptr != transsmission) {
            m_transmissionLayer = transsmission;
        }
    }

    constexpr std::size_t getLedsCount() const {
        return LEDS_CNT;
    }

    friend void DMA1_Channel7_IRQHandler();
    friend void DMA1_Channel3_IRQHandler();
    friend class LedsTransmission;

protected:
    enum FillingBuffState : std::sig_atomic_t {
          NotReady = 0
        , Ready = 1
    };

    bool swapBuffers();
    virtual void startDMATransaction(const uint16_t transferSize, const uint8_t* startAddr, const uint8_t* endAddr);
    bool hasNextToTransmit();
//    void convertFirstSPIBuffer();

protected:

    uint8_t m_RgbBuffers[2][LEDS_COUNT*3];
    uint8_t* m_pTransmittingBuff;
    uint8_t* m_pFillingBuff;

    /// used to store from where data were being modified
    uint8_t* m_pModifiedStart;
    uint8_t* m_pModifiedEnd;
    /// points to space one after the last which will be transmitted
    uint8_t* m_pTranssmitionEnd;

    /// using during converting data - modification of this pointer occurs mostly
    /// in the interrupt routine
    uint8_t* volatile m_pTransmissionNextLEDColor;


    RGBColor m_MarkerColor;

    volatile std::sig_atomic_t m_FillingBuffReady;

    LedsTransmission* m_transmissionLayer;
};


// IMPLEMENTATION ///


inline RGBColor::RGBColor()
    : LEDColor(0,0,0) { ; }

inline RGBColor::RGBColor(uint8_t r, uint8_t g, uint8_t b)
    : LEDColor(r,g,b) { ; }


inline RGBColor& RGBColor::operator+=(const uint8_t d) {
    uint8_t diff = 255 - r;
    if (diff >= d) {r += d;} else {r += diff;}
    diff = 255 - g;
    if (diff >= d) {g += d;} else {g += diff;}
    diff = 255 - b;
    if (diff >= d) {b += d;} else {b += diff;}
    return *this;
}

inline RGBColor& RGBColor::operator-=(const uint8_t d) {
    if (r >= d) {r -= d;} else {r = 0;}
    if (g >= d) {g -= d;} else {g = 0;}
    if (b >= d) {b -= d;} else {b = 0;}
    return *this;
}

inline RGBColor& RGBColor::operator++() {
    if (r < 255) ++r;
    if (g < 255) ++g;
    if (b < 255) ++b;
    return *this;
}

inline RGBColor& RGBColor::operator--() {
    if (r > 0) --r;
    if (g > 0) --g;
    if (b > 0) --b;
    return *this;
}

inline  RGBColor& RGBColor::inc(const uint8_t v, const RGBColor& max) {
    uint8_t diff = max.r - r;
    if (diff >= v) {r += v;} else {r += diff;}
    diff = max.g - g;
    if (diff >= v) {g += v;} else {g += diff;}
    diff = max.b - b;
    if (diff >= v) {b += v;} else {b += diff;}

    return *this;
}

inline RGBColor RGBColor::operator- (const uint8_t d) const {
    RGBColor t = *this;
    t -= d;
    return t;
}

template <std::size_t LEDS_CNT>
inline  WS2813Leds<LEDS_CNT>::WS2813Leds()
    : m_pTransmittingBuff(&m_RgbBuffers[1][0])
    , m_pFillingBuff(&m_RgbBuffers[0][0])
    , m_pModifiedStart(&m_RgbBuffers[0][0])
    , m_pModifiedEnd(&m_RgbBuffers[0][0])
    , m_pTranssmitionEnd(nullptr)
    , m_pTransmissionNextLEDColor(nullptr)
    , m_FillingBuffReady(FillingBuffState::Ready)
    , m_transmissionLayer(nullptr)
{

    //todo fix hardoced irq number
    __enable_irq();
    NVIC_SetPriority(DMA1_Channel7_IRQn, 7);
    NVIC_ClearPendingIRQ(DMA1_Channel7_IRQn);
    NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

template <std::size_t LEDS_CNT>
inline void WS2813Leds<LEDS_CNT>::setMarkerColor(RGBColor&& color) {
    m_MarkerColor = color;
}

template <std::size_t LEDS_CNT>
inline bool WS2813Leds<LEDS_CNT>::hasNextToTransmit() {
    // m_pTranssmissionEnd points to element one after the last to send
    return m_pTransmissionNextLEDColor < m_pTranssmitionEnd;
}

template <std::size_t LEDS_CNT>
inline RGBColor WS2813Leds<LEDS_CNT>::getNextToTransmit() {
    RGBColor retColor;
    if (true == hasNextToTransmit()) {
        retColor.r = *m_pTransmissionNextLEDColor;
        retColor.g = *(m_pTransmissionNextLEDColor+1);
        retColor.b = *(m_pTransmissionNextLEDColor+2);
        m_pTransmissionNextLEDColor += 3;
    }

    return retColor;
}

template <std::size_t LEDS_CNT>
inline bool WS2813Leds<LEDS_CNT>::swapBuffers() {
    bool retval = false;

    // If transmission is still in progress then do nothing.
    if (nullptr != m_transmissionLayer && false == m_transmissionLayer->stillTransmitting()) {
    do {
        // The call to this class methods that operates on Filling buffer will
        // block until this flag will be set to Ready by the DMA that
        // synchronizes Transmitting and Filling buffers. After that the API
        // can make operations on the Filling buffer.
        m_FillingBuffReady = FillingBuffState::NotReady;

        // calculate the distance of modified start in current filling buffer
        const std::ptrdiff_t modifiedStartPos = m_pModifiedStart - m_pFillingBuff;
        const std::ptrdiff_t modifiedEndPos = m_pModifiedEnd - m_pFillingBuff;

        // Fire DMA that copies memory from start_modified to transmission_end
        const std::int32_t modifiedSize = (modifiedEndPos - modifiedStartPos) * sizeof(m_RgbBuffers[0][0]);
        if (modifiedSize <= 0) {
            break;
        }

        // Note that the m_pTransmittingBuff will be the filling buffer after
        // procedure finish.
        startDMATransaction(modifiedSize, m_pModifiedStart, m_pTransmittingBuff + modifiedStartPos);

        // set transmission_end pointer that points at right place in the
        // transmitting buffer
        m_pTranssmitionEnd = m_pModifiedEnd;
        // set next led pointer to points to the first data in transmitting buffer
        m_pTransmissionNextLEDColor = m_pFillingBuff;


        // Make the filling buffer the transmitting one, and the transmitting
        // make the filling buffer
        std::swap(m_pFillingBuff, m_pTransmittingBuff);

        // restore modified position in current(new) filling buffer
        if (modifiedStartPos > 0) {
            m_pModifiedStart = m_pFillingBuff + modifiedStartPos;
        } else {
            m_pModifiedStart = m_pFillingBuff;
        }

        // reset the end position in new filling buffer
        m_pModifiedEnd = m_pFillingBuff;

        // begin transmission -> it will handle the first and later conversions
        while (false == m_transmissionLayer->beginTransmision((m_pTranssmitionEnd - m_pTransmissionNextLEDColor)/3)) { ; }

        retval = true;
    } while(0);
    }

    return retval;
}

template <std::size_t LEDS_CNT>
inline void WS2813Leds<LEDS_CNT>::startDMATransaction(const uint16_t transferSize, const uint8_t* startAddr, const uint8_t* endAddr) {

    if (0 == transferSize) {
        return;
    }

#ifndef PC_VISUALIZATION
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel7->CCR = 0;

    // memory to memory, start address will be in peripheral, 8bit -> 8bit
    DMA1_Channel7->CCR |= DMA_CCR7_MEM2MEM | DMA_CCR7_TCIE | DMA_CCR7_MINC | DMA_CCR7_PINC;
    DMA1_Channel7->CNDTR = transferSize;

    DMA1_Channel7->CPAR = reinterpret_cast<std::uintptr_t>(startAddr);
    DMA1_Channel7->CMAR = reinterpret_cast<std::uintptr_t>(endAddr);

    // start the DMA transaction
    DMA1_Channel7->CCR |= DMA_CCR7_EN;
#endif
}




template <std::size_t LEDS_CNT>
inline void WS2813Leds<LEDS_CNT>::fill(uint16_t start, uint16_t end) {
    fill(start, end, m_MarkerColor);
}


template <std::size_t LEDS_CNT>
inline void WS2813Leds<LEDS_CNT>::fill(uint16_t start, uint16_t end, const RGBColor& color) {
    start *= 3;
    end *= 3;
    int32_t placeDistance = end - start;
    if (placeDistance <= 0) {
        return;
    }

    if (end > LEDS_COUNT*3)
    {
        return;
    }

    while(FillingBuffState::Ready != m_FillingBuffReady) { ; }

    // update modified_start pointer
    std::ptrdiff_t ptrDist = m_pModifiedStart - m_pFillingBuff;
    if (start < ptrDist && ptrDist > 0) {
        m_pModifiedStart = m_pFillingBuff + start;
    } else {
        m_pModifiedStart = m_pFillingBuff;
    }

    // update transmission_end pointer
    ptrDist = m_pModifiedEnd - m_pFillingBuff;
    if (end > ptrDist && ptrDist >= 0) {
        m_pModifiedEnd = m_pFillingBuff + end;
    }

    for (uint16_t i = start; i < end; i += 3) {
        m_pFillingBuff[i] = color.r;
        m_pFillingBuff[i+1] = color.g;
        m_pFillingBuff[i+2] = color.b;
    }
}


template <std::size_t LEDS_CNT>
inline bool WS2813Leds<LEDS_CNT>::send() {
    return swapBuffers();
}

template <std::size_t LEDS_CNT>
inline void WS2813Leds<LEDS_CNT>::clear() {
    fill(0, LEDS_CNT, {0,0,0});
}

template <std::size_t LEDS_CNT>
inline bool WS2813Leds<LEDS_CNT>::moveForward(const uint16_t orgStart, const uint16_t newStart, const uint16_t len) {
    bool retval = false;
    const uint16_t _orgStart = orgStart*3;
    const uint16_t _newStart = newStart*3;
    const uint16_t _len = len*3;
    do {
        const int32_t orgEnd = _orgStart + _len; // should substrate 1??

        // check for proper distance
        if (orgStart >= newStart) {
            break;
        }
        // check for buffer boundaries
        if ((0 == len) || (orgEnd >= LEDS_CNT*3) || (newStart + len > LEDS_CNT)) {
            break;
        }

        // calculate the distance between org start and new start
        const uint16_t distance = _newStart - _orgStart;

        // start copy from the last index
        auto iter = orgEnd;
        while (iter >= static_cast<int32_t>(_orgStart)) {
            m_pFillingBuff[iter+distance] = m_pFillingBuff[iter];
            --iter;
        }

        // update modified_start pointer
        std::ptrdiff_t ptrDist = m_pModifiedStart - m_pFillingBuff;
        if ((iter+1+distance) < ptrDist && ptrDist > 0) {
            m_pModifiedStart = m_pFillingBuff + iter + 1 + distance;
        } else {
            m_pModifiedStart = m_pFillingBuff;
        }

        // update modified_end pointer
        ptrDist = m_pModifiedEnd - m_pFillingBuff;
        if (((orgEnd + distance) > ptrDist) && (ptrDist >= 0)) {
            m_pModifiedEnd = m_pFillingBuff + orgEnd + distance;
        }

        retval = true;

    } while(0);

    return retval;
}


template <std::size_t LEDS_CNT>
bool WS2813Leds<LEDS_CNT>::moveBackward(const uint16_t orgStart, const uint16_t newStart, const uint16_t len) {
    bool retval = false;
    const uint16_t _orgStart = orgStart * 3;
    const uint16_t _newStart = newStart * 3;
    const uint16_t _len = len * 3;
    const int32_t orgEnd = _orgStart - _len;
    do {
        if (orgStart <= newStart) {
            break;
        }

        if ((0 == len) || (orgEnd <= 0) || (static_cast<int32_t>(_newStart - _len) < 0)) {
            break;
        }

        // calculate the distance between org start and new start
        const uint16_t distance = _orgStart - _newStart;

        // start copy from the first index
        auto iter = orgEnd;
        while (iter < static_cast<int32_t>(_orgStart+3)) {
            m_pFillingBuff[iter-distance] = m_pFillingBuff[iter];
            ++iter;
        }

        // update modified_start pointer
        std::ptrdiff_t ptrDist = m_pModifiedStart - m_pFillingBuff;
        if ((orgEnd-distance) < ptrDist && ptrDist > 0) {
            m_pModifiedStart = m_pFillingBuff + orgEnd - distance;
        } else {
            m_pModifiedStart = m_pFillingBuff;
        }

        // update modified_end pointer
        ptrDist = m_pModifiedEnd - m_pFillingBuff;
        if (((iter-1-distance) > ptrDist) && (ptrDist >= 0)) {
            m_pModifiedEnd = m_pFillingBuff + iter-1-distance;
        }

        retval = true;
    } while(0);

    return retval;
}



#endif /* WS2813LEDS_H_ */
