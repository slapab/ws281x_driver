#ifndef LEDS_TRANSMISSION_H
#define LEDS_TRANSMISSION_H

#include <cstdint>
#include <csignal>

//Interrupt handler declaration in C/C++
#ifdef __cplusplus
extern "C" {
#endif
     void DMA1_Channel3_IRQHandler();
#ifdef __cplusplus
}
#endif

// forward declaration
template <std::size_t LEDS_CNT>
class WS2813Leds;

class LedsTransmission {
public:
    friend void DMA1_Channel3_IRQHandler();

    template <std::size_t LED_CNTS>
    explicit LedsTransmission(WS2813Leds<LED_CNTS>& ledsStrip)
        : m_LedsStrip(ledsStrip)
        , m_currBuffIdx(0)  // points to the first buffer
        , m_nextBuffIdx(1)  // points to the second buffer
        , m_TransmissionStatus(TransmissionStatus::FINISHED)
        , m_TransferSize(0)
    {
        initBindBandPointers();
        initSPI();
        initDMA();

        // this not working with globals
    //    m_LedsStrip.registerTransmissionLayer(this);
    }
    ///  Begins transmission, it takes into account that data are already
    /// converted and stored into buffer
    bool beginTransmision(const std::sig_atomic_t transferSize);
    bool stillTransmitting() { return TransmissionStatus::IN_PROGRESS == m_TransmissionStatus; }

private:
    void initBindBandPointers();
    void initSPI();
    void initDMA();
    void convertFirstTwoLeds();

private:
    enum TransmissionStatus : std::sig_atomic_t {
        IN_PROGRESS = 0,
        FINISHED = 1
    };


    WS2813Leds<300u>& m_LedsStrip;

    /// Buffers which will store converted data from RGB to SPI valid bytes
    /// @Note Type of buffers are 16-bit because SPI supports sending 16 bits
    /// wide data.
    constexpr static const uint8_t BUFFERS_NO = 2;
    constexpr static const uint8_t BUFFER_SIZE = 12;
    volatile uint8_t m_ConvBuffers[BUFFERS_NO][BUFFER_SIZE];
    /// stores pointers to bitband memory that corresponds to the m_ConvBuffers
    volatile uint32_t* m_bitBandPtrs[BUFFERS_NO][BUFFER_SIZE*2]; // 1 byte stores only 2 bits

    /// Points to current buffer from which data are transmitting
    uint8_t m_currBuffIdx;
    /// Points to buffer that contains fresh data, ready to send
    uint8_t m_nextBuffIdx;

    volatile std::sig_atomic_t m_TransmissionStatus;
    volatile std::sig_atomic_t m_TransferSize;

};


#endif // LEDS_TRANSMISSION_H
