#include <chrono>
#include <QtDebug>
#include "render_area.h"
#include "animation_thread.h"
#include "PCWS281xLeds.hpp"
#include "Timer.h"
#include "PCLedsTransmission.h"
#include "../src/AnimationContext.h"
#include "../src/PseudoRandomNumberGenerator.h"
#include "../src/FallingLightAnimation.hpp"
#include "../src/PseudoRandomDotsAnimation.hpp"
#include "../src/MovingPixelsAnimation.hpp"
#include "../src/FallingSnowAnimation.hpp"
#include "../src/WalkingDotsAnimation.hpp"
#include "../src/MovingColorfullLightsAnimation.hpp"
#include "../src/MovingColorfullLights2DirAnimation.hpp"


void AnimationManagement::threadImpl() {
    using namespace std::chrono_literals;
    std::hash<std::thread::id> threadIDHasher;
    const std::size_t thrID = threadIDHasher(std::this_thread::get_id());
    qDebug() << "Starting AnimationManagement thread, id = " << thrID;


    // init leds strip, transmission layer
    PCWS281xLeds<LEDS_COUNT> ledStrip1;
    PCLedsTransmission ws281xTransLayer(ledStrip1);
    ws281xTransLayer.setRenderArea(&m_RenderArea);
    // register transmission layer into ws2813Leds object
    ledStrip1.registerTransmissionLayer(&ws281xTransLayer);

    // create pseudo random number generator
    pseudo_rng::PseudoRandomNumberGenerator<uint16_t> rndGen(8011, 61291, 337);

    // create animation context
    animations::AnimationContext animationsContex(rndGen);

    Timer t;
    // create animations
    animations::FallingLightAnimation fallingLightAnimation(t, ledStrip1);
    animations::PseudoRandomDotsAnimation pseudoRandomDots(t, ledStrip1);
    animations::MovingPixelsAnimation movingPixels(t, ledStrip1);
    animations::FallingSnowAnimation fallingSnow(t, ledStrip1);
    animations::WalkingDotsAnimation walkingDots(t, ledStrip1);
    animations::MovingColorfullLightsAnimation movingColorfullLights(t, ledStrip1);
    animations::MovingColorfullLights2DirectionsAnimation movingColorfullLightsIn2Directions(t, ledStrip1);
//     register animations
    animationsContex.registerAnimation(&pseudoRandomDots);
//    animationsContex.registerAnimation(&movingPixels);
//    animationsContex.registerAnimation(&fallingLightAnimation);
    animationsContex.registerAnimation(&fallingSnow);
//    animationsContex.registerAnimation(&walkingDots);
//    animationsContex.registerAnimation(&movingColorfullLights);
//    animationsContex.registerAnimation(&movingColorfullLightsIn2Directions);

    // loop until someone cleared flag
    while(true == m_IsRunningFlag.test_and_set(std::memory_order_acquire)) {
        qDebug() << "Running next animation";
        // run next animation
        animationsContex.runNext();

        std::this_thread::sleep_for(10ms);
    }

    qDebug() << "Exiting from AnimationManagement thread, id = " << thrID;
}



AnimationManagement::AnimationManagement(RenderArea& renderArea)
    : m_RenderArea(renderArea)
{
    m_IsRunningFlag.test_and_set(std::memory_order_acquire);
}

AnimationManagement::~AnimationManagement() {
    m_IsRunningFlag.clear(std::memory_order_release);
}

bool AnimationManagement::startThread() {
    bool retval = false;

    m_IsRunningFlag.test_and_set(std::memory_order_acquire);

    try {
        m_ThreadHandle = std::thread(&AnimationManagement::threadImpl, this);
        retval = true;
    } catch (...) {;}

    return retval;
}

void AnimationManagement::stopThread() {
    m_IsRunningFlag.clear(std::memory_order_release);
    m_ThreadHandle.join();
}
