#include <algorithm>            // std::fill_n
#include "AnimationContext.h"
#include "AnimationInterface.h"

namespace animations {

AnimationContext::AnimationContext()
    : m_CurrentAnimation(-1) {
    std::fill_n(m_Animations, ANIMATIONS_NO, nullptr);
}

int AnimationContext::registerAnimation(AnimationInterface* newAnimation) {
    int newDescr = -1;

    if (nullptr != newAnimation) {
        for (auto i = 0; i < ANIMATIONS_NO; ++i) {
            if (nullptr == m_Animations[i]) {
                m_Animations[i] = newAnimation;
                // success, return 'descriptor' of animation
                newDescr = i;
                break;
            }
        }
    }

    return newDescr;
}



bool AnimationContext::runNext() {
    bool retval = false;

    ++m_CurrentAnimation;
    if (m_CurrentAnimation >= ANIMATIONS_NO) {
        m_CurrentAnimation = 0;
    }

    if (nullptr == m_Animations[m_CurrentAnimation]) {
        for (auto i = 0; i < ANIMATIONS_NO; ++i) {
            if (nullptr != m_Animations[i]) {
                m_CurrentAnimation = i;
                retval = true;
                break;
            }
        }
    } else {
        retval = true;
    }

    if (true == retval) {
        m_Animations[m_CurrentAnimation]->run(this);
    }

    return retval;
}


bool AnimationContext::runSpecific(const int descr) {
    bool retval = false;

    if ((descr < ANIMATIONS_NO) && (nullptr != m_Animations[descr])) {
        m_Animations[descr]->run(this);
        retval = true;
    }

    return retval;
}


} // namespace
