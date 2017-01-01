#include <thread>
#include <chrono>
#include <QtDebug>
#include <cstdint>

#include "PCLedsTransmission.h"
#include "PCWS281xLeds.hpp"
#include "render_area.h"

PCLedsTransmission::~PCLedsTransmission() {
    std::unique_lock<std::mutex> lk(m_Mutex);
    m_Continue.store(false);
    lk.unlock();
    m_CV.notify_one();
    m_TransThread.join();
}


// will imitate sending data to WS281x
void PCLedsTransmission::transmissionThreadFunc() {
    std::size_t size = 0;
    RGBColor c;

    std::unique_lock<std::mutex> lk(m_Mutex);
    while(true == m_Continue.load()) {
        // Wait until beginTransmission() is being called

        // wait about 500ms and check if there is waiting transfer and m_Continue is true, or
        // if m_Continue is false then exit from waiting ane exit the thread

        while (LedsTransmission::TransmissionStatus::IN_PROGRESS != LedsTransmission::m_TransmissionStatus) {
            m_CV.wait(lk);

//                , std::chrono::milliseconds(500), [this]() {
//            return ((TransmissionStatus::IN_PROGRESS == m_TransmissionStatus) || (false == m_Continue.load()));
//        });

            // exit from the thread
            if (false == m_Continue.load()) {
                return;
            }
        }

        // nullptr!!!!
        if (nullptr == m_pRenderArea) {
            qDebug() << "nullptr == m_pRenderArea!";
            continue;
        }

        // begin transmission, and transmit
        size = static_cast<std::size_t>(m_TransferSize);
        for (std::size_t i = 0; i < size; ++i) {
            // get data from transmission buffer from ledsStrip class
            c = m_LedsStrip.getNextToTransmit();
            // write data to the render

            emit postLEDData(c.r, c.g, c.b, i*3);
        }
        // imitate transmission delay - about 1.333 us for each bit of color
        std::this_thread::sleep_for(std::chrono::nanoseconds(size*1333*24));
        // simulate transmission end on the line
        std::this_thread::sleep_for(std::chrono::microseconds(50));

        // inform the RenderArea that transmission is over
        emit endOfTransmissoin();

        m_TransmissionStatus = TransmissionStatus::FINISHED;

        // note, the mutex will be unlocked automatically
    }
}



bool PCLedsTransmission::beginTransmision(const std::sig_atomic_t transferSize) {
    bool retVal = false;

    // detect if transaction is in progress
    std::unique_lock<std::mutex> lk(m_Mutex);//, std::defer_lock_t());
    if ((TransmissionStatus::FINISHED == m_TransmissionStatus)) { // && (true == lk.try_lock())) {

        m_TransmissionStatus = TransmissionStatus::IN_PROGRESS;
        m_TransferSize = transferSize;

        lk.unlock();
        m_CV.notify_one();

        retVal = true;
    }

    return retVal;
}

void PCLedsTransmission::setRenderArea(RenderArea *pRenderArea)
{
    if (nullptr != pRenderArea) {
        m_pRenderArea = pRenderArea;
        auto c1 = QObject::connect(this, &PCLedsTransmission::endOfTransmissoin, m_pRenderArea, &RenderArea::redraw, Qt::ConnectionType::QueuedConnection);
        auto c2 = QObject::connect(this, &PCLedsTransmission::postLEDData, m_pRenderArea, &RenderArea::writeLED, Qt::ConnectionType::QueuedConnection);

        if (!c1)
            qDebug() << "Signal-Slot connection doesn't work";

        if (!c2)
            qDebug() << "Signal-Slot connection doesn't work!";
    }
}
