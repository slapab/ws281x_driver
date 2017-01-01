#ifndef ANIMATION_THREAD_H
#define ANIMATION_THREAD_H

#include <atomic>
#include <thread>

class RenderArea;

class AnimationManagement {
public:
    ~AnimationManagement();
    AnimationManagement() = delete;
    AnimationManagement(RenderArea& renderArea);

    bool startThread();
    void stopThread();

protected:
    void threadImpl();

private:

    RenderArea&      m_RenderArea;
    std::thread      m_ThreadHandle;
    std::atomic_flag m_IsRunningFlag;
};

#endif // ANIMATION_THREAD_H
