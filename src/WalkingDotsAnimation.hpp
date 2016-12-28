#ifndef WALKINGDOTSANIMATION_HPP
#define WALKINGDOTSANIMATION_HPP

#include "AnimationInterface.h"

namespace animations {

class WalkingDotsAnimation final : public AnimationInterface {
public:
    virtual ~WalkingDotsAnimation() = default;
    WalkingDotsAnimation() = delete;

    WalkingDotsAnimation(Timer& t, WS2813Leds<LEDS_COUNT>& ledStrip)
        : AnimationInterface(t, ledStrip) {;}

    void run(AnimationContext* context) override;

    constexpr static uint8_t min(uint8_t a, uint8_t b, uint8_t c) {
        if (a > b) {
            a = b;
        }
        // a should be min right now
        return (a < c) ? a : c;
    }
};


inline void WalkingDotsAnimation::run(AnimationContext* context) {
    constexpr const Timer::ticks_t delay = 70;

    m_LedStrip.clear();

    const RGBColor defColors[] = {  {0x0c, 0xbc, 0xee}
                                    , {0xeb, 0x6c, 0x61}
                                    , {0x85, 0x30, 0xc1}
                                    , {0x28, 0xa5, 0x19}
                                    , {0xe3, 0x17, 0x2f}
                                    , {0xd4, 0xf2, 0x13}
                                    , {0x79, 0xa3, 0x5c}
                                    , {0x0e, 0x64, 0xfb}
                                    , {56, 216, 123}
                                    };

    RGBColor tc = defColors[0];

    bool dimmingDir = false;
    int keepColorAtMax = 10;
    int keepColorAtMin = 7;
    uint8_t color_idx = 0;

    // Describes the white light moving up an down
    struct {
        int idx = 4;
        bool direction = true;
        const RGBColor cmax{220,220,220};

        void operator++() {
            // update the index
            if (true == direction) { ++idx; } else { --idx; }

            // change direction
            if ((idx > static_cast<int>(LEDS_COUNT-4)) || (idx < 4)) {
                direction ^= true;
                if (true == direction) {
                    idx = 5;
                } else {
                    idx = static_cast<int>(LEDS_COUNT - 5);
                }
            }
        }


        void insertIntoStrip(WS2813Leds<LEDS_COUNT>& ledStrip, const RGBColor& maskColor) {
            // set color created by 'walking dots' but in positions where dots are not 'walking' normally
            ledStrip.fill(static_cast<uint16_t>(idx-2),static_cast<uint16_t>(idx-1), maskColor);
            ledStrip.fill(static_cast<uint16_t>(idx+2),static_cast<uint16_t>(idx+3), maskColor);

            // set white color
            ledStrip.fill(static_cast<uint16_t>(idx-1),static_cast<uint16_t>(idx+2), cmax);

            // update accordingly the index and change direction if needed
            operator++();
        }
    } whiteLight;


    m_LedStrip.clear();
    for (int repeater = 0; repeater < 1000; ++repeater) {
        // fill even leds
        for (uint16_t i = 0 ; i < LEDS_COUNT; i += 4) {
            m_LedStrip.fill(i, i+1, tc); // turn on
            m_LedStrip.fill(i+2, i+3, {0,0,0}); // turn off
        }

        whiteLight.insertIntoStrip(m_LedStrip, tc);

        // send
        while (false == m_LedStrip.send());
        m_Timer.sleep(delay);

        // fill odd leds
        for (uint16_t i = 2 ; i < LEDS_COUNT; i += 4) {
            m_LedStrip.fill(i, i+1, tc); // turn on
            m_LedStrip.fill(i-2, i-1, {0,0,0}); // turn off
        }

        // control dimming
        if (false == dimmingDir) {
            tc-=30;
            if ( tc.r <= 5 && tc.g <= 5 && tc.b <= 5) {
                --keepColorAtMin;
                if (keepColorAtMin <= 0) {
                    keepColorAtMin = 7;
                    dimmingDir ^= true;
                    color_idx = (color_idx+1) % (sizeof(defColors)/sizeof(defColors[0]));
                }
            }
        } else {
            if (defColors[color_idx].r == tc.r && defColors[color_idx].g == tc.g && defColors[color_idx].b == tc.b) {
                --keepColorAtMax;
                if (keepColorAtMax <= 0) {
                    keepColorAtMax = 10;
                    dimmingDir ^= true;
                }

            } else {
                tc.inc(20, defColors[color_idx]);
            }
        }

        whiteLight.insertIntoStrip(m_LedStrip, tc);

        // send
        while (false == m_LedStrip.send());
        m_Timer.sleep(delay-10);

    } // repeater loop

    m_LedStrip.clear();
    while (false == m_LedStrip.send());
    m_Timer.sleep(5);

}

} // namespace
#endif // WALKINGDOTSANIMATION_HPP
