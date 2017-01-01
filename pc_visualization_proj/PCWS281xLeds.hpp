#ifndef PCWS281XLEDS_H
#define PCWS281XLEDS_H

#include <cstring>
#include "../src/WS2813Leds.h"
#include "PCLedsTransmission.h"

template <std::size_t LEDS_CNT>
class PCWS281xLeds : public WS2813Leds<LEDS_CNT>
{
public:
    PCWS281xLeds() : WS2813Leds<LEDS_CNT>() {;}
    virtual ~PCWS281xLeds() {;}

    RGBColor getNextToTransmit() override {
        return WS2813Leds<LEDS_CNT>::getNextToTransmit();
    }

    friend class PCLedsTransmission;
protected:
    virtual void startDMATransaction(const uint16_t transferSize, const uint8_t* startAddr, const uint8_t* endAddr) override {
        if (0 == transferSize) {
            return;
        }
        // PC implementation! synchronize buffers
        std::memcpy(const_cast<uint8_t*>(endAddr), startAddr, transferSize);
        WS2813Leds<LEDS_CNT>::m_FillingBuffReady = WS2813Leds<LEDS_CNT>::FillingBuffState::Ready;

    }
};

#endif // PCWS281XLEDS_H
