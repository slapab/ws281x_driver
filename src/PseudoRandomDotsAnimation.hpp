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
    auto rndGen = context->getRndGen();
    RGBColor fg{180, 0, 0};
    RGBColor tmp;


    // generate index
    for (int j = 0; j < 2; ++j ) {
    for (int i = 0; i < 298; ++i) {

        uint16_t idx = rndGen.randomize() % (LEDS_COUNT-1);

        fg.r = rndGen.randomize() ;
        fg.g = rndGen.randomize();
        fg.b = rndGen.randomize() % 120;

        m_LedStrip.fill(idx, idx+1, fg);
        while (false == m_LedStrip.send());


            m_Timer.sleep(20);
            fg.r = 0;
            fg.g = 0;
            fg.b = 0;
            idx = rndGen.randomize() % (LEDS_COUNT-1);
            m_LedStrip.fill(idx, idx+1, fg);
            while (false == m_LedStrip.send());


        m_Timer.sleep(20);

        // srnad
        rndGen.seed(rndGen.randomize(), idx, fg.b);
    }
    }


//    for (int i = 0 ; i < 100; ++i) {
//
//        m_LedStrip.fill(idx, idx+1, fg);
//        while (false == m_LedStrip.send());
//
//        m_Timer.sleep(50);
//    }

}

} //namespace
#endif /* PSEUDORANDOMDOTSANIMATION_HPP_ */
