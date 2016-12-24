#ifndef ANIMATIONCONTEXT_H_
#define ANIMATIONCONTEXT_H_

#include "AnimationInterface.h"
#include "PseudoRandomNumberGenerator.h"

namespace animations {

class AnimationContext {
public:
    AnimationContext(pseudo_rng::PseudoRandomNumberGenerator<uint16_t>& rnd);
    ~AnimationContext() = default;

    int registerAnimation(AnimationInterface*);
    bool runNext();
    bool runSpecific(const int descr);
    pseudo_rng::PseudoRandomNumberGenerator<uint16_t>& getRndGen() {
        return m_RndGen;
    }

private:
    constexpr static const auto ANIMATIONS_NO = 20;
    AnimationInterface* m_Animations[ANIMATIONS_NO];
    int m_CurrentAnimation;
    pseudo_rng::PseudoRandomNumberGenerator<uint16_t>& m_RndGen;
};


}

#endif /* ANIMATIONCONTEXT_H_ */
