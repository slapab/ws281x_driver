#ifndef MOVINGPIXELSANIMATION_HPP
#define MOVINGPIXELSANIMATION_HPP

#include "AnimationInterface.h"

namespace animations {

class MovingPixelsAnimation final : public AnimationInterface {
public:
    virtual ~MovingPixelsAnimation() = default;
    MovingPixelsAnimation() = delete;

    MovingPixelsAnimation(Timer& t, WS2813Leds<LEDS_COUNT>& ledStrip)
        : AnimationInterface(t, ledStrip) {;}

    void run(AnimationContext* context) override;
};


inline void MovingPixelsAnimation::run(AnimationContext* context) {
    auto RndGen = context->getRndGen();
    RGBColor movingColor{60, 0, 200};
    RGBColor flashColorMax;
    RGBColor flashColorL1;
    RGBColor flashColorL2;
    RGBColor flashColorL3;
    const RGBColor backgroundColor{0,0,0};
    constexpr const auto ledsStripMiddlePos = LEDS_COUNT/2;

    auto genRandColor = [&]() {
        movingColor.r = RndGen.randomize() % 220;
        movingColor.g = RndGen.randomize() % 180;
        movingColor.b = RndGen.randomize() & 0x00FF;
    };


    Timer::ticks_t delay = 11; // ms

    for (int jj = 0; jj < 10; ++jj) {
        delay = 12;
        genRandColor();
        // ---- withe flash
//        flashColorMax = {200,200,200};
//        flashColorL1 = {40,40,40};
//        flashColorL2 = {90,90,90};
//        flashColorL3 = {150,150,150};
        // ----

        // ---- dots color will be the flash color
        flashColorMax = movingColor;
        flashColorL3.r = static_cast<uint8_t>(((static_cast<int>(flashColorMax.r) * 7)/9));
        flashColorL3.g = static_cast<uint8_t>(((static_cast<int>(flashColorMax.g) * 7)/9));
        flashColorL3.b = static_cast<uint8_t>(((static_cast<int>(flashColorMax.b) * 7)/9));
        flashColorL2.r = static_cast<uint8_t>(((static_cast<int>(flashColorMax.r) * 4)/6));
        flashColorL2.g = static_cast<uint8_t>(((static_cast<int>(flashColorMax.g) * 4)/6));
        flashColorL2.b = static_cast<uint8_t>(((static_cast<int>(flashColorMax.b) * 4)/6));
        flashColorL1.r = static_cast<uint8_t>(((static_cast<int>(flashColorMax.r) * 1)/3));
        flashColorL1.g = static_cast<uint8_t>(((static_cast<int>(flashColorMax.g) * 1)/3));
        flashColorL1.b = static_cast<uint8_t>(((static_cast<int>(flashColorMax.b) * 1)/3));
        // ----
//        RndGen.seed(flashColorL3.r+flashColorL3.r+flashColorL2.g, jj, flashColorL2.g ^ (2*flashColorL3.g));
        for (int16_t i = 1, k = LEDS_COUNT-2; i < LEDS_COUNT; ++i, --k) {

            // when dots meet at the middle
            if (i + 1 == k) {
                genRandColor();
                delay = 6;
            }
            // show flash at the middle after meeting of dots
            else if ( i > k ) {
                flashColorMax -= 2;
                flashColorL1 -= 1;
                flashColorL2 -= 2;
                flashColorL3 -= 2;
                //max
                m_LedStrip.fill(ledsStripMiddlePos-2, ledsStripMiddlePos+3, flashColorMax);
                // l3
                m_LedStrip.fill(ledsStripMiddlePos-5, ledsStripMiddlePos-2, flashColorL3);
                m_LedStrip.fill(ledsStripMiddlePos+3, ledsStripMiddlePos+6, flashColorL3);
                // l2
                m_LedStrip.fill(ledsStripMiddlePos-8, ledsStripMiddlePos-5, flashColorL2);
                m_LedStrip.fill(ledsStripMiddlePos+6, ledsStripMiddlePos+9, flashColorL2);
                // l1
                m_LedStrip.fill(ledsStripMiddlePos-11, ledsStripMiddlePos-8, flashColorL1);
                m_LedStrip.fill(ledsStripMiddlePos+9, ledsStripMiddlePos+12, flashColorL1);
            }

            // moving up:
            // clear turned on LED before
            m_LedStrip.fill(i-1, i, backgroundColor);
            // set next led
            m_LedStrip.fill(i, i+1, movingColor);

            // moving down:
            // set next led
            m_LedStrip.fill(k, k+1, movingColor);
            m_LedStrip.fill(k+1, k+2, backgroundColor);



            while(false == m_LedStrip.send()) {;}
            m_Timer.sleep(delay);

        }

    }
}

} // namespace
#endif // MOVINGPIXELSANIMATION_HPP
