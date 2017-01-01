#ifndef PCLedsTransmission_H
#define PCLedsTransmission_H

#include "../src/LedsTransmission.h"
#include <QObject>
#include <cstdint>
#include <csignal>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>


// forward declaration
template <std::size_t LEDS_CNT>
class PCWS281xLeds;
class RenderArea;

class PCLedsTransmission : public QObject, public LedsTransmission {
    Q_OBJECT
public:
    PCLedsTransmission() = delete;

    template <std::size_t LED_CNTS>
    explicit PCLedsTransmission(PCWS281xLeds<LED_CNTS>& ledsStrip)
        : QObject(nullptr)
        , LedsTransmission(ledsStrip)
        , m_pRenderArea(nullptr)
    {
        // this not working with globals
        //    m_LedsStrip.registerTransmissionLayer(this);

        qRegisterMetaType<uint8_t>("uint8_t");
        qRegisterMetaType<std::size_t>("size_t");

        m_Continue.store(true);
        // start transmission thread -> it should block until beginTransmission will be called
        m_TransThread = std::thread(&PCLedsTransmission::transmissionThreadFunc, this);
    }

    virtual ~PCLedsTransmission();

    ///  Begins transmission, it takes into account that data are already
    /// converted and stored into buffer
    bool beginTransmision(const std::sig_atomic_t transferSize) override;

    void setRenderArea(RenderArea* pRenderArea);

signals:
    void postLEDData(const uint8_t r, const uint8_t g, const uint8_t b, const std::size_t pos);
    void endOfTransmissoin();

private:
    void transmissionThreadFunc();

private:
    // Will used to pass data to the UI thread
    RenderArea* m_pRenderArea;

    std::thread m_TransThread;
    std::mutex  m_Mutex;
    std::condition_variable m_CV;

    // the thread should check for true, if false should exit
    std::atomic<bool> m_Continue;
};


#endif // PCLedsTransmission_H
