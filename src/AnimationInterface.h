#ifndef ANIMATIONINTERFACE_H_
#define ANIMATIONINTERFACE_H_

#include "Timer.h"
#include "AnimationContext.h"
#include "WS2813Leds.h"

namespace animations {
class AnimationContext;

class AnimationInterface {
public:
    AnimationInterface() = delete;
    AnimationInterface(Timer& t, WS2813Leds<LEDS_COUNT>& ledStrip)
        : m_Timer(t), m_LedStrip(ledStrip) {
        ;
    }
    virtual ~AnimationInterface() = default;

    virtual void run(AnimationContext* context) = 0;

protected:
    Timer& m_Timer;
    WS2813Leds<LEDS_COUNT>& m_LedStrip;
};

} //namespace Animations

#endif /* ANIMATIONINTERFACE_H_ */
