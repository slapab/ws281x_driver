#-------------------------------------------------
#
# Project created by QtCreator 2016-12-25T17:42:08
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = ws281x_visualization
TEMPLATE = app

CONFIG += c++14
DEFINES+=PC_VISUALIZATION=1

SOURCES += main.cpp\
        mainwindow.cpp \
    render_area.cpp \
    animation_thread.cpp \
    Timer.cpp \
    ../src/AnimationContext.cpp \
    PCLedsTransmission.cpp \
    ../src/LedsTransmission.cpp \
    ../src/PseudoRandomNumberGenerator.cpp

HEADERS  += mainwindow.h \
    render_area.h \
    animation_thread.h \
    Timer.h \
    stm32f10x.h \
    PCWS281xLeds.hpp \
    ../src/WS2813Leds.h \
    ../src/AnimationInterface.h \
    ../src/AnimationContext.h \
    ../src/FallingLightAnimation.hpp \
    ../src/FallingSnowAnimation.hpp \
    PCLedsTransmission.h \
    ../src/LedsTransmission.h \
    ../src/PseudoRandomNumberGenerator.h \
    ../src/MovingColorfullLights2DirAnimation.hpp \
    ../src/MovingColorfullLightsAnimation.hpp \
    ../src/MovingPixelsAnimation.hpp \
    ../src/PseudoRandomDotsAnimation.hpp \
    ../src/WalkingDotsAnimation.hpp

FORMS    += mainwindow.ui
