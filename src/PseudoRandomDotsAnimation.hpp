#ifndef PSEUDORANDOMDOTSANIMATION_HPP_
#define PSEUDORANDOMDOTSANIMATION_HPP_

#include "AnimationInterface.h"

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
    auto rndGen = context->getRndGen();
    RGBColor fg{180, 0, 0};
    RGBColor tmp;


    // generate index
    for (int j = 0; j < 3; ++j ) {
    for (int i = 0; i < 400; ++i) {
        uint16_t idx;
        do {
        idx = rndGen.randomize() & 0x01FF;
        } while(idx >= LEDS_COUNT-1);

        fg.r = rndGen.randomize() % 230;
        fg.g = rndGen.randomize() % 200;
        fg.b = rndGen.randomize() % 120;

        m_LedStrip.fill(idx, idx+1, fg);
        while (false == m_LedStrip.send());

        m_Timer.sleep(10);
        fg.r = 0;
        fg.g = 0;
        fg.b = 0;


        do {
        idx = rndGen.randomize() & 0x01FF;
        } while(idx >= LEDS_COUNT-1);

        m_LedStrip.fill(idx, idx+1, fg);
        while (false == m_LedStrip.send());


        m_Timer.sleep(15);
    }
        //-----------------------------------------------
        // just turn off most leds
//        fg = {0,0,0};
//        for (std::size_t i = 0; i < 400; ++i) {
//            uint16_t idx ;//= rndGen.randomize();

//            do {
//                idx = rndGen.randomize() & 0x01FF;
//            } while(idx >= LEDS_COUNT-1);

//            m_LedStrip.fill(idx, idx+1, fg);
//            while (false == m_LedStrip.send());
//            m_Timer.sleep(15);
//        }
        //-----------------------------------------------

        //-----------------------------------------------
        // flash light - as clear strip
        uint16_t idx;
        do {
            idx = rndGen.randomize() & 0x01FF;
        } while (idx >= LEDS_COUNT-1);

        // bright up
        RGBColor c{5,5,5};
        for (auto i = 0; i <= 180; i += 5) {
            m_LedStrip.fill(0, LEDS_COUNT-1, c);
            while (false == m_LedStrip.send());
            m_Timer.sleep(8);
            if (j < 1) {
                c.r = c.g = c.b = static_cast<uint8_t>(i);
            } else if (j < 2) {
                c.r = c.b = 0;
                c.g = static_cast<uint8_t>(i);
            } else if (j < 3) {
                c.r = c.g = 0;
                c.b = static_cast<uint8_t>(i);
            }
        }

        // bright down
        for (int32_t i = 180; i >= 0; i -= 5) {
            m_LedStrip.fill(0, LEDS_COUNT, c);
            while (false == m_LedStrip.send());
            m_Timer.sleep(8);
            if (j < 1) {
                c.r = c.g = c.b = static_cast<uint8_t>(i);
            } else if (j < 2) {
               c.r = c.b = 0;
               c.g = static_cast<uint8_t>(i);
            } else if (j < 3) {
                c.r = c.g = 0;
                c.b = static_cast<uint8_t>(i);
            }
        }

        // turn off leds
        c = {0,0,0};
        m_LedStrip.fill(0, LEDS_COUNT, c);
        while (false == m_LedStrip.send());
        m_Timer.sleep(500);

        //-----------------------------------------------
    }

}

} //namespace
#endif /* PSEUDORANDOMDOTSANIMATION_HPP_ */
