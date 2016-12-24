#ifndef FALLINGLIGHTANIMATION_HPP_
#define FALLINGLIGHTANIMATION_HPP_

namespace animations {
class FallingLightAnimation final : public AnimationInterface {
public:
    virtual ~FallingLightAnimation() = default;
    FallingLightAnimation() = delete;

    FallingLightAnimation(Timer& t, WS2813Leds<LEDS_COUNT>& ledStrip)
        : AnimationInterface(t, ledStrip) {;}

    void run(AnimationContext* context) override;
};


inline void FallingLightAnimation::run(AnimationContext* context) {
    auto rndGen = context->getRndGen();
    RGBColor fg{0, 0, 0};

    // clear
    m_LedStrip.fill(0, LEDS_COUNT-1, fg);
    while (false == m_LedStrip.send());

    fg.r = fg.b = fg.g = 240;
    RGBColor l5{210, 210, 210};
    RGBColor l4{180, 180, 180};
    RGBColor l3{120, 120, 120};
    RGBColor l2{60, 60, 60};
    RGBColor l1{20, 20, 20};


    uint16_t delay = 25;
    bool dir = true ;

    for (int j = 0; j < 50; ++j) {
        if (j > 0) {
            fg.r = rndGen.randomize();
            fg.g = rndGen.randomize();
            fg.b = rndGen.randomize();

            l5 = {fg.r * 5/6, fg.g * 5/6, fg.b *5/6};
            l4 = {fg.r * 7/9, fg.g * 7/9, fg.b * 7/9};
            l3 = {fg.r * 5/9, fg.g * 5/9, fg.b * 5/9};
            l2 = {fg.r * 3/9, fg.g * 3/9, fg.b * 3/9};
            l1 = {fg.r * 1/4, fg.g * 1/9, fg.b * 1/4};
        }

        if (true == dir ) {
            // From bottom -> up
            for (auto i = 6; i < LEDS_COUNT-6; ++i) {
                if (i > 6) {
                    m_LedStrip.fill(i-6, i-6+1, {0,0,0});
                }
                m_LedStrip.fill(i-6, i-6 +1, l1);
                //l1
                m_LedStrip.fill(i-5, i-5 +1, l2);
                //l2
                m_LedStrip.fill(i-4, i-4 +1, l3);
                m_LedStrip.fill(i-3, i-3 +1, l4);
                m_LedStrip.fill(i-2, i-2 +1, l5);
                m_LedStrip.fill(i-1, i-1 +1, fg);



                while(false == m_LedStrip.send());

                m_Timer.sleep(delay);

            }

            // dimm last
            m_LedStrip.fill(LEDS_COUNT-7, LEDS_COUNT-1, {0,0,0});
            while(false == m_LedStrip.send());
            m_Timer.sleep(2);

        } else {
            // top -> bottom
            for (int32_t i = LEDS_COUNT-5; i >= 0; --i) {



                m_LedStrip.fill(i, i+1, fg);
                m_LedStrip.fill(i+4, i+4+1, l1);
                m_LedStrip.fill(i+3, i+3+1, l2);
                m_LedStrip.fill(i+2, i+2+1, l3);
                m_LedStrip.fill(i+1, i+1+1, l4);

                if (i < LEDS_COUNT - 5) {
                    m_LedStrip.fill(i+5, i+5+1, {0,0,0});
                }

                while(false == m_LedStrip.send());

                m_Timer.sleep(delay);

            }

            // dimming the rest LEDS
            m_LedStrip.fill(0, 5, {0,0,0});
            while(false == m_LedStrip.send());
            m_Timer.sleep(2);
        }

        // change direction
        dir ^= true;
    }



}

} //namespace
#endif /* FALLINGLIGHTANIMATION_HPP_ */
