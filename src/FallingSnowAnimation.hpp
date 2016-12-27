#ifndef FALLINGSNOWANIMATION_HPP_
#define FALLINGSNOWANIMATION_HPP_

#include "AnimationInterface.h"

//template <class Type>
//pseudo_rng::PseudoRandomNumberGenerator<Type>;

namespace animations {
class FallingSnowAnimation final : public AnimationInterface {
public:
    virtual ~FallingSnowAnimation() = default;
    FallingSnowAnimation() = delete;

    FallingSnowAnimation(Timer& t, WS2813Leds<LEDS_COUNT>& ledStrip)
        : AnimationInterface(t, ledStrip) {;}

    void run(AnimationContext* context) override;

private:
    void runLight(const uint16_t idx, uint16_t& iter, RGBColor* t);
    void showLight(uint16_t startidx, uint16_t iter, RGBColor* Colors);
    void moveLight(uint16_t startidx, uint16_t iter, RGBColor* Colors);
    void hideLight(uint16_t startidx, uint16_t iter, RGBColor* Colors);

    void generateIndexes(AnimationContext* context);
    uint16_t genOneIdx(AnimationContext* context);
    void generateColor(AnimationContext* context, RGBColor* Colors);

private:
    constexpr static const uint8_t m_LightLength = 14;
    // how many lightLength the 'snow' will go
    constexpr static const uint8_t m_FallingFactor = 2;
    constexpr static const uint8_t m_IndexesNo = 4;
    uint16_t m_Indexes[m_IndexesNo];

};


inline void FallingSnowAnimation::run(AnimationContext* context) {
    auto rndGen = context->getRndGen();

    RGBColor slide[7] = {{200, 200, 200},
                         {173,173,173},
                         {140,140,140},
                         {107,107,107},
                         {74,74,74},
                         {31,31,31},
                         {10,10,10}
                        };

    RGBColor slide2[7] = {{200, 200, 200},
                         {173,173,173},
                         {140,140,140},
                         {107,107,107},
                         {74,74,74},
                         {31,31,31},
                         {10,10,10}
                        };
    RGBColor slide3[7] = {{200, 200, 200},
                         {173,173,173},
                         {140,140,140},
                         {107,107,107},
                         {74,74,74},
                         {31,31,31},
                         {10,10,10}
                        };

    RGBColor slide4[7] = {{200, 200, 200},
                         {173,173,173},
                         {140,140,140},
                         {107,107,107},
                         {74,74,74},
                         {31,31,31},
                         {10,10,10}
                        };

    uint16_t iter1, iter2, iter3, iter4;


    m_LedStrip.clear();
    while (false == m_LedStrip.send());
    m_Timer.sleep(25);

    // repeat loop
    for (uint16_t repeat = 0; repeat < 30; ++repeat) {

        rndGen.seed(repeat, rndGen.randomize(), m_Indexes[0]);
        generateIndexes(context);

        iter1 = 0;
        iter2 = 0;
        iter3 = 0;
        iter4 = 0;
        //whole light road
        for (auto i = 0; i < ((m_FallingFactor+1)*m_LightLength) + m_LightLength + (m_LightLength); ++i) {



            runLight(m_Indexes[0], iter1, slide);


            if ( i >= 2*m_LightLength ) {
                runLight(m_Indexes[1], iter2, slide2);
            }

            // run delayed
            if ( i >= m_LightLength) {
                runLight(m_Indexes[2], iter3, slide3);
            }

            runLight(m_Indexes[3], iter4, slide4);

            while (false == m_LedStrip.send());
            m_Timer.sleep(15);

        }

        generateColor(context, slide);
        generateColor(context, slide2);
        generateColor(context, slide3);
//        m_Timer.sleep(200);
//        m_LedStrip.clear();
//        while (false == m_LedStrip.send());
        m_Timer.sleep(5);

    } // repeat loop

}

inline void FallingSnowAnimation::generateColor(AnimationContext* context, RGBColor* Colors) {
    auto& rndGen = context->getRndGen();
    int colorR = rndGen.randomize() & 0x00FF;
    int colorG = rndGen.randomize() & 0x00FF;
    int colorB = rndGen.randomize() & 0x00FF;

    RGBColor& cMax = Colors[0];
    RGBColor& cL6 = Colors[1];
    RGBColor& cL5 = Colors[2];
    RGBColor& cL4 = Colors[3];
    RGBColor& cL3 = Colors[4];
    RGBColor& cL2 = Colors[5];
    RGBColor& cL1 = Colors[6];

    cMax = {static_cast<uint8_t>(colorR), static_cast<uint8_t>(colorG), static_cast<uint8_t>(colorB)};
    cL6 = {static_cast<uint8_t>((colorR*7)/8), static_cast<uint8_t>((colorG*7)/8), static_cast<uint8_t>((colorB*7)/8)};
    cL5 = {static_cast<uint8_t>((colorR*6)/8), static_cast<uint8_t>((colorG*6)/8), static_cast<uint8_t>((colorB*6)/8)};
    cL4 = {static_cast<uint8_t>((colorR*5)/9), static_cast<uint8_t>((colorG*5)/9), static_cast<uint8_t>((colorB*5)/9)};
    cL3 = {static_cast<uint8_t>((colorR*3)/8), static_cast<uint8_t>((colorG*3)/8), static_cast<uint8_t>((colorB*3)/8)};
    cL2 = {static_cast<uint8_t>(colorR / 5), static_cast<uint8_t>(colorG / 5), static_cast<uint8_t>(colorB / 5)};
    cL1 = {static_cast<uint8_t>(colorR % 10), static_cast<uint8_t>(colorG % 10), static_cast<uint8_t>(colorB % 10)};
}

inline uint16_t FallingSnowAnimation::genOneIdx(AnimationContext* context) {
    uint16_t idx;
    constexpr const uint8_t hiddenLEDsNo = 15;
    constexpr const uint16_t minIndexVal = (m_LightLength*m_FallingFactor)+hiddenLEDsNo;
    auto& rndGen = context->getRndGen();
    do {
        idx = rndGen.randomize() & 0x01FF;
    } while (idx < minIndexVal || idx >= LEDS_COUNT-((m_LightLength*m_FallingFactor)));
    return idx;
}

inline void FallingSnowAnimation::generateIndexes(AnimationContext* context) {
    auto genNext = [&](const uint8_t alreadyFoundNo) {
        int16_t idxdiff;
        uint16_t newIdx;
        do {
            newIdx = genOneIdx(context);
            // look for previous
            for (auto i = 0; i < alreadyFoundNo; ++i) {
                idxdiff = static_cast<int16_t>(newIdx-m_Indexes[i]);
                if (idxdiff < 0) {
                    idxdiff = -idxdiff;
                }

                if (idxdiff < m_LightLength*m_FallingFactor) {
                    break;
                }
            }
        } while (idxdiff < m_LightLength*m_FallingFactor);

        return newIdx;
    };

    // generate the first
    m_Indexes[0] = genOneIdx(context);
    // generate next unique
    for (uint8_t i = 1; i < m_IndexesNo; ++i) {
        m_Indexes[i] = genNext(i);
    }
}

inline void FallingSnowAnimation::runLight(const uint16_t idx, uint16_t& iter, RGBColor* colors) {
    if ( iter >= ((m_FallingFactor+1)*m_LightLength)) {
        return;
    }

    // start from one dot to moment when whole length is visible
    if (iter < m_LightLength) {
        showLight(idx, iter, colors);
    }
    // move light to moment when the last one the whole length is visible
    else if (iter < (m_LightLength*m_FallingFactor)) {
        moveLight(idx, iter, colors);
    }
    // hiding light one by one
    else if (iter >= m_LightLength*m_FallingFactor) {
        hideLight(idx, iter, colors);
    }

    ++iter;
}

inline void FallingSnowAnimation::hideLight(uint16_t startidx, uint16_t iter, RGBColor* Colors) {
    RGBColor& c1Max = Colors[0];
    RGBColor& c1L6 = Colors[1];
    RGBColor& c1L5 = Colors[2];
    RGBColor& c1L4 = Colors[3];
    RGBColor& c1L3 = Colors[4];
    RGBColor& c1L2 = Colors[5];
    RGBColor& c1L1 = Colors[6];

    uint16_t intIter = static_cast<uint16_t>(iter - (m_LightLength*m_FallingFactor));
    uint16_t move = startidx + intIter + (m_LightLength*(m_FallingFactor-1));
    // turn off the prev led
    m_LedStrip.fill(move, move + 1, {0,0,0});
    switch (intIter) {
    // keep lighting one from max
    case 0:
        m_LedStrip.fill(move+1, move+1+2, c1L1);
        m_LedStrip.fill(move+1+2, move+1+2+2, c1L2);
        m_LedStrip.fill(move+1+2+2, move+1+2+2+2, c1L3);
        m_LedStrip.fill(move+1+2+2+2, move+1+2+2+2+2, c1L4);
        m_LedStrip.fill(move+1+2+2+2+2, move+1+2+2+2+2+2, c1L5);
        m_LedStrip.fill(move+1+2+2+2+2+2, move+1+2+2+2+2+2+2, c1L6);
        m_LedStrip.fill(move+1+2+2+2+2+2+2, move+1+2+2+2+2+2+2+1, c1Max); break;
    case 1:
        m_LedStrip.fill(move+1, move+1+2, c1L1);
        m_LedStrip.fill(move+1+2, move+1+2+2, c1L2);
        m_LedStrip.fill(move+1+2+2, move+1+2+2+2, c1L3);
        m_LedStrip.fill(move+1+2+2+2, move+1+2+2+2+2, c1L4);
        m_LedStrip.fill(move+1+2+2+2+2, move+1+2+2+2+2+2, c1L5);
        m_LedStrip.fill(move+1+2+2+2+2+2, move+1+2+2+2+2+2+2, c1L6); break;
    // L6 off
    case 2:
        m_LedStrip.fill(move+1, move+1+2, c1L1);
        m_LedStrip.fill(move+1+2, move+1+2+2, c1L2);
        m_LedStrip.fill(move+1+2+2, move+1+2+2+2, c1L3);
        m_LedStrip.fill(move+1+2+2+2, move+1+2+2+2+2, c1L4);
        m_LedStrip.fill(move+1+2+2+2+2, move+1+2+2+2+2+2, c1L5);
        m_LedStrip.fill(move+1+2+2+2+2+2, move+1+2+2+2+2+2+1, c1L6); break;
    case 3:
        m_LedStrip.fill(move+1, move+1+2, c1L1);
        m_LedStrip.fill(move+1+2, move+1+2+2, c1L2);
        m_LedStrip.fill(move+1+2+2, move+1+2+2+2, c1L3);
        m_LedStrip.fill(move+1+2+2+2, move+1+2+2+2+2, c1L4);
        m_LedStrip.fill(move+1+2+2+2+2, move+1+2+2+2+2+2, c1L5); break;
    // L5 off
    case 4:
        m_LedStrip.fill(move+1, move+1+2, c1L1);
        m_LedStrip.fill(move+1+2, move+1+2+2, c1L2);
        m_LedStrip.fill(move+1+2+2, move+1+2+2+2, c1L3);
        m_LedStrip.fill(move+1+2+2+2, move+1+2+2+2+2, c1L4);
        m_LedStrip.fill(move+1+2+2+2+2, move+1+2+2+2+2+1, c1L5); break;
    case 5:
        m_LedStrip.fill(move+1, move+1+2, c1L1);
        m_LedStrip.fill(move+1+2, move+1+2+2, c1L2);
        m_LedStrip.fill(move+1+2+2, move+1+2+2+2, c1L3);
        m_LedStrip.fill(move+1+2+2+2, move+1+2+2+2+2, c1L4); break;
    // L4 off
    case 6:
        m_LedStrip.fill(move+1, move+1+2, c1L1);
        m_LedStrip.fill(move+1+2, move+1+2+2, c1L2);
        m_LedStrip.fill(move+1+2+2, move+1+2+2+2, c1L3);
        m_LedStrip.fill(move+1+2+2+2, move+1+2+2+2+1, c1L4); break;
    case 7:
        m_LedStrip.fill(move+1, move+1+2, c1L1);
        m_LedStrip.fill(move+1+2, move+1+2+2, c1L2);
        m_LedStrip.fill(move+1+2+2, move+1+2+2+2, c1L3); break;
    // L3 off
    case 8:
        m_LedStrip.fill(move+1, move+1+2, c1L1);
        m_LedStrip.fill(move+1+2, move+1+2+2, c1L2);
        m_LedStrip.fill(move+1+2+2, move+1+2+2+1, c1L3); break;
    case 9:
        m_LedStrip.fill(move+1, move+1+2, c1L1);
        m_LedStrip.fill(move+1+2, move+1+2+2, c1L2); break;
    // L2 off
    case 10:
        m_LedStrip.fill(move+1, move+1+2, c1L1);
        m_LedStrip.fill(move+1+2, move+1+2+1, c1L2); break;
    case 11:
        m_LedStrip.fill(move+1, move+1+2, c1L1); break;
    // L1 off
    case 12:
        m_LedStrip.fill(move+1, move+1+1, c1L1); break;
    default: break;
    }
}


inline void FallingSnowAnimation::moveLight(uint16_t startidx, uint16_t iter, RGBColor* Colors) {
    RGBColor& c1Max = Colors[0];
    RGBColor& c1L6 = Colors[1];
    RGBColor& c1L5 = Colors[2];
    RGBColor& c1L4 = Colors[3];
    RGBColor& c1L3 = Colors[4];
    RGBColor& c1L2 = Colors[5];
    RGBColor& c1L1 = Colors[6];

    uint16_t move = static_cast<uint16_t>(startidx + (iter-m_LightLength));
    // clear the latest led
    m_LedStrip.fill(move, move+1, {0,0,0});
    ++move;
    m_LedStrip.fill(move, move+2, c1L1);
    m_LedStrip.fill(move+2, move+2+2, c1L2);
    m_LedStrip.fill(move+2+2, move+2+2+2, c1L3);
    m_LedStrip.fill(move+2+2+2, move+2+2+2+2, c1L4);
    m_LedStrip.fill(move+2+2+2+2, move+2+2+2+2+2, c1L5);
    m_LedStrip.fill(move+2+2+2+2+2, move+2+2+2+2+2+2, c1L6);
    m_LedStrip.fill(move+2+2+2+2+2+2, move+2+2+2+2+2+2+2, c1Max);
}


inline void FallingSnowAnimation::showLight(uint16_t startidx, uint16_t iter, RGBColor* Colors) {
    RGBColor& c1Max = Colors[0];
    RGBColor& c1L6 = Colors[1];
    RGBColor& c1L5 = Colors[2];
    RGBColor& c1L4 = Colors[3];
    RGBColor& c1L3 = Colors[4];
    RGBColor& c1L2 = Colors[5];
    RGBColor& c1L1 = Colors[6];

    switch(iter) {
    // show 1 maximum dot
    case 0 : m_LedStrip.fill(startidx, startidx+1, c1Max); break;
    // show 2 max dot
    case 1 : m_LedStrip.fill(startidx, startidx+2, c1Max); break;
    // show 1st dot of L6 , and maximum
    case 2 : m_LedStrip.fill(startidx, startidx+1, c1L6);
             m_LedStrip.fill(startidx+1, startidx+1+2, c1Max); break;
    // show 2nd dot of L6, and maximum
    case 3 : m_LedStrip.fill(startidx, startidx+2, c1L6);
             m_LedStrip.fill(startidx+2, startidx+2+2, c1Max); break;
    // show 1st dot of L5, and L6, and maximum
    case 4 : m_LedStrip.fill(startidx, startidx+1, c1L5);
             m_LedStrip.fill(startidx+1, startidx+1+2, c1L6);
             m_LedStrip.fill(startidx+1+2, startidx+1+2+2, c1Max); break;
    // show 2 dots of L5 and the rest
    case 5 :
             m_LedStrip.fill(startidx, startidx+2, c1L5);
             m_LedStrip.fill(startidx+2, startidx+2+2, c1L6);
             m_LedStrip.fill(startidx+2+2, startidx+2+2+2, c1Max); break;
    // show 1st dot of L4 and the rest
    case 6 :
             m_LedStrip.fill(startidx, startidx+1, c1L4);
             m_LedStrip.fill(startidx+1, startidx+1+2, c1L5);
             m_LedStrip.fill(startidx+1+2, startidx+1+2+2, c1L6);
             m_LedStrip.fill(startidx+1+2+2, startidx+1+2+2+2, c1Max); break;
    // show 2 dots of L4 and the rest
    case 7 :
             m_LedStrip.fill(startidx, startidx+2, c1L4);
             m_LedStrip.fill(startidx+2, startidx+2+2, c1L5);
             m_LedStrip.fill(startidx+2+2, startidx+2+2+2, c1L6);
             m_LedStrip.fill(startidx+2+2+2, startidx+2+2+2+2, c1Max); break;
    // show 1st dot of L3 and the rest
    case 8 :
            m_LedStrip.fill(startidx, startidx+1, c1L3);
            m_LedStrip.fill(startidx+1, startidx+1+2, c1L4);
            m_LedStrip.fill(startidx+1+2, startidx+1+2+2, c1L5);
            m_LedStrip.fill(startidx+1+2+2, startidx+1+2+2+2, c1L6);
            m_LedStrip.fill(startidx+1+2+2+2, startidx+1+2+2+2+2, c1Max); break;
    // show 2 dots of L3 and the rest
    case 9 :
            m_LedStrip.fill(startidx, startidx+2, c1L3);
            m_LedStrip.fill(startidx+2, startidx+2+2, c1L4);
            m_LedStrip.fill(startidx+2+2, startidx+2+2+2, c1L5);
            m_LedStrip.fill(startidx+2+2+2, startidx+2+2+2+2, c1L6);
            m_LedStrip.fill(startidx+2+2+2+2, startidx+2+2+2+2+2, c1Max); break;
    // show 1st dot of L2 and the rest
    case 10 :
            m_LedStrip.fill(startidx, startidx+1, c1L2);
            m_LedStrip.fill(startidx+1, startidx+1+2, c1L3);
            m_LedStrip.fill(startidx+1+2, startidx+1+2+2, c1L4);
            m_LedStrip.fill(startidx+1+2+2, startidx+1+2+2+2, c1L5);
            m_LedStrip.fill(startidx+1+2+2+2, startidx+1+2+2+2+2, c1L6);
            m_LedStrip.fill(startidx+1+2+2+2+2, startidx+1+2+2+2+2+2, c1Max); break;
    // show 2 dots of L2 and the rest
    case 11 :
            m_LedStrip.fill(startidx, startidx+2, c1L2);
            m_LedStrip.fill(startidx+2, startidx+2+2, c1L3);
            m_LedStrip.fill(startidx+2+2, startidx+2+2+2, c1L4);
            m_LedStrip.fill(startidx+2+2+2, startidx+2+2+2+2, c1L5);
            m_LedStrip.fill(startidx+2+2+2+2, startidx+2+2+2+2+2, c1L6);
            m_LedStrip.fill(startidx+2+2+2+2+2, startidx+2+2+2+2+2+2, c1Max); break;
    // show 1st dot of L1 and the rest
    case 12 :
            m_LedStrip.fill(startidx, startidx+1, c1L1);
            m_LedStrip.fill(startidx+1, startidx+1+2, c1L2);
            m_LedStrip.fill(startidx+1+2, startidx+1+2+2, c1L3);
            m_LedStrip.fill(startidx+1+2+2, startidx+1+2+2+2, c1L4);
            m_LedStrip.fill(startidx+1+2+2+2, startidx+1+2+2+2+2, c1L5);
            m_LedStrip.fill(startidx+1+2+2+2+2, startidx+1+2+2+2+2+2, c1L6);
            m_LedStrip.fill(startidx+1+2+2+2+2+2, startidx+1+2+2+2+2+2+2, c1Max); break;
    // show 2 dots of L1 and the rest
    case 13 :
            m_LedStrip.fill(startidx, startidx+2, c1L1);
            m_LedStrip.fill(startidx+2, startidx+2+2, c1L2);
            m_LedStrip.fill(startidx+2+2, startidx+2+2+2, c1L3);
            m_LedStrip.fill(startidx+2+2+2, startidx+2+2+2+2, c1L4);
            m_LedStrip.fill(startidx+2+2+2+2, startidx+2+2+2+2+2, c1L5);
            m_LedStrip.fill(startidx+2+2+2+2+2, startidx+2+2+2+2+2+2, c1L6);
            m_LedStrip.fill(startidx+2+2+2+2+2+2, startidx+2+2+2+2+2+2+2, c1Max); break;
    default: break;
    }
}
} //namespace
#endif /* FALLINGSNOWANIMATION_HPP_ */
