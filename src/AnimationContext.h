#ifndef ANIMATIONCONTEXT_H_
#define ANIMATIONCONTEXT_H_

#include "AnimationInterface.h"

namespace animations {

class AnimationContext {
public:
    AnimationContext();
    ~AnimationContext() = default;

    int registerAnimation(AnimationInterface*);
    bool runNext();
    bool runSpecific(const int descr);

private:
    constexpr static const auto ANIMATIONS_NO = 20;
    AnimationInterface* m_Animations[ANIMATIONS_NO];
    int m_CurrentAnimation;
};


}

#endif /* ANIMATIONCONTEXT_H_ */
