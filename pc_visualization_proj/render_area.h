#ifndef RENDERAREA_H
#define RENDERAREA_H

#include <QtWidgets/QWidget>
#include <QPen>
#include <QBrush>
#include <cstdint>

class RenderArea : public QWidget
{
    Q_OBJECT
public:
    explicit RenderArea(QWidget *parent = nullptr);
//    QSize minimumSizeHint() const Q_DECL_OVERRIDE;
//    QSize sizeHint() const Q_DECL_OVERRIDE;

//    void redraw(QColor c);

public slots:
    /// called from transmission layer -> indicates that data must be displayed
    void redraw();
    /// @param pos the LED position, is counting from 0
    void writeLED(const uint8_t r, const uint8_t g, const uint8_t b, const std::size_t pos);

signals:

protected:
    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;


private:
    constexpr static const int LED_WIDTH = 4;
    QPen m_Pen;
    QBrush m_Brush;

    uint8_t m_LedsColors[300*3];
    std::size_t m_redrawCount;
};

#endif // RENDERAREA_H
