#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include <thread>
#include <memory>               //std::unique_ptr
#include "animation_thread.h"

namespace Ui {
class MainWindow;
}

class RenderArea;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    RenderArea *m_renderArea;

    std::unique_ptr<AnimationManagement> m_AnimationThread;
};

#endif // MAINWINDOW_H
