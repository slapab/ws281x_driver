#ifndef MOVINGCOLORFULLLIGHTSANIMATION_HPP_
#define MOVINGCOLORFULLLIGHTSANIMATION_HPP_

#include "AnimationInterface.h"

namespace animations {
class MovingColorfullLightsAnimation final : public AnimationInterface {
public:
    virtual ~MovingColorfullLightsAnimation() = default;
    MovingColorfullLightsAnimation() = delete;

    MovingColorfullLightsAnimation(Timer& t, WS2813Leds<LEDS_COUNT>& ledStrip)
        : AnimationInterface(t, ledStrip) {;}

    void run(AnimationContext* context) override;

protected:
    struct LightStrip {
        constexpr static const uint8_t LightLen = 15;
        constexpr static const uint8_t SegLen = 3;
        constexpr static const uint8_t SegNo = LightLen / SegLen;
        constexpr static const uint8_t ColorsTabLen = 9;
    private:
        uint8_t ledCnt = SegLen;
        uint8_t segCnt = SegNo;
        uint8_t colorIdx = 0;
        const RGBColor Colors[ColorsTabLen] = {{0x33, 0x7e, 0xa3},
                                               {0x9e, 0xf7, 0x02},
                                               {0xf4, 0x39, 0xff},
                                               {220,  220, 220},
                                               {0xf9, 0x81, 0x1e},
                                               {0x24, 0xc7, 0xd7},
                                               {0x23, 0x90, 0x32},
                                               {0xfe, 0xe1, 0x1b},
                                               {0xbd, 0x0a, 0x76}};
    public:
        RGBColor currColor = Colors[0];
    public:
        void operator++() {
            // control light color
            if (0 == segCnt) {
                colorIdx = (colorIdx+1) % ColorsTabLen;
                currColor = Colors[colorIdx];
                segCnt = SegNo;
            }

            // color light brightness on each segment
            if (0 == ledCnt) {
                --segCnt;
                ledCnt = SegLen;
                switch(segCnt) {
                case SegNo-1: currColor -= 40; break;
                case SegNo-2: currColor -= 30; break;
                case SegNo-3: currColor -= 20; break;
                case SegNo-4: currColor -= 20; break;
                default: break;
                };
            }
            --ledCnt;
        }
    };
};


inline void MovingColorfullLightsAnimation::run(AnimationContext* context) {
    (void)context;
//    auto rndGen = context->getRndGen();


    m_LedStrip.clear();


    constexpr const Timer::ticks_t delayMax = 80;
    constexpr const Timer::ticks_t delayMin = 10;
    uint16_t delayMinCnt = 100;
    bool delayDir = true;
    Timer::ticks_t delay = 30;

    LightStrip lightStrip;

    // set at the beginning the last LED
    m_LedStrip.fill(LEDS_COUNT-1,LEDS_COUNT,lightStrip.currColor);
    while (false == m_LedStrip.send());
    m_Timer.sleep(delay);

    for (int repeater = 0; repeater < 3200; ++repeater) {

        // control delay
        if (0 == repeater % 5) {
            if (true == delayDir) {
                delay += 4;
            } else {
                if (delayMin == delay) {
                    --delayMinCnt;
                } else {
                    delay -= 4;
                }
            }

            if (delay >= delayMax || (delay <= delayMin && 0 == delayMinCnt)) {
                delayDir ^= true;
                delayMinCnt = 100;
            }
        }

        // move data in the buffer by one: top -> down
        bool test = m_LedStrip.moveBackward(LEDS_COUNT, LEDS_COUNT-1, LEDS_COUNT-1);

        // set the last LED
        m_LedStrip.fill(LEDS_COUNT-1,LEDS_COUNT,lightStrip.currColor);
        ++lightStrip;

        while (false == m_LedStrip.send());
        m_Timer.sleep(delay);

    } // repeat loop


    m_LedStrip.clear();
    while (false == m_LedStrip.send());
    m_Timer.sleep(2);
}

} //namespace
#endif /* MOVINGCOLORFULLLIGHTSANIMATION_HPP_ */
