/*
 * PseudoRandomDotsAnimation.hpp
 *
 *  Created on: 24.12.2016
 *      Author: scott
 */

#ifndef PSEUDORANDOMDOTSANIMATION_HPP_
#define PSEUDORANDOMDOTSANIMATION_HPP_

namespace animations {
class PseudoRandomDotsAnimation final : public AnimationInterface {
public:
    virtual ~PseudoRandomDotsAnimation() = default;
    PseudoRandomDotsAnimation() = delete;

    PseudoRandomDotsAnimation(Timer& t, WS2813Leds<LEDS_COUNT>& ledStrip)
        : AnimationInterface(t, ledStrip) {;}

    void run(AnimationContext* context) override;
};


inline void PseudoRandomDotsAnimation::run(AnimationContext* context) {
    m_LedStrip.fill(0,2, {0,250, 0});

    while (false == m_LedStrip.send());
    m_Timer.sleep(500);
}

} //namespace
#endif /* PSEUDORANDOMDOTSANIMATION_HPP_ */
