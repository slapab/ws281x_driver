#include <thread>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "render_area.h"

extern std::atomic<bool> runAnimationThread;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    m_renderArea = new RenderArea();
    m_renderArea->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Expanding);
    ui->m_HorLayout->setSizeConstraint(QLayout::SetNoConstraint);
    ui->m_HorLayout->stretch(2);
    ui->m_HorLayout->addWidget(m_renderArea);

    // spawn animation thread
    if (nullptr != m_renderArea) {
        m_AnimationThread = std::make_unique<AnimationManagement>(*m_renderArea);
        if (nullptr != m_AnimationThread) {
            m_AnimationThread->startThread();
        }
    } else {
        m_AnimationThread.reset(nullptr);
    }

}

MainWindow::~MainWindow()
{
    if (nullptr != m_AnimationThread) {
        m_AnimationThread->stopThread();
    }

    delete ui;
}
