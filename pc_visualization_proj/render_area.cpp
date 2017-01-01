#include "render_area.h"
#include <QPalette>
#include <QPainter>

RenderArea::RenderArea(QWidget *parent)
    : QWidget(parent)
    , m_redrawCount(0)
{
    // white background
//    setBackgroundRole(QPalette::Base);
//    setStyleSheet("background-color:black;");

    // black background
    QPalette pal;
    pal.setColor(QPalette::Background, Qt::black);
    setPalette(pal);

    setAutoFillBackground(true);



    m_Pen.setColor(QColor(255,0,0,255));
    m_Pen.setStyle(Qt::NoPen);

    m_Brush.setColor(QColor(0,255,0, 255));
    m_Brush.setStyle(Qt::SolidPattern);
//    setMinimumWidth(800);
}

//QSize RenderArea::minimumSizeHint() const
//{
//    return QSize(100, 800);
//}

//QSize RenderArea::sizeHint() const
//{
//    return QSize(400, 800);
//}

//void RenderArea::redraw(QColor c)
//{
//    m_Brush.setColor(c);
//    update();
//}

void RenderArea::redraw()
{
    update();
}

void RenderArea::writeLED(const uint8_t r, const uint8_t g, const uint8_t b, const std::size_t pos)
{
    m_LedsColors[pos]   = r;
    m_LedsColors[pos+1] = g;
    m_LedsColors[pos+2] = b;

    m_redrawCount = pos+3;
}

void RenderArea::paintEvent(QPaintEvent *event)
{
//    static const QPoint points[2] = {
//        QPoint(10, 80),
//        QPoint(20, 10),
//        QPoint(80, 30),
//        QPoint(90, 70)
//    };



//    QPainterPath path;
//    path.moveTo(20, 80);
//    path.lineTo(20, 30);
//    path.cubicTo(80, 0, 50, 50, 80, 80);

//        int startAngle = 20 * 16;
//        int arcLength = 120 * 16;

    QPainter painter(this);
    painter.setPen(m_Pen);

//        painter.setRenderHint(QPainter::Antialiasing, true);

    painter.save();
    {
        constexpr const int LEDS_CNT = (sizeof(m_LedsColors)/sizeof(m_LedsColors[0]))/3;
        int t = (width()/2) - ((LEDS_CNT/2)*LED_WIDTH);
        if (t < 0) {
            t = 0;
        }
        painter.translate(t, (height()/2)-LED_WIDTH);
    }

    int xLoc = 0;
    int yLoc = 0;
    QColor ledColor;
    for (std::size_t i = 0; i < sizeof(m_LedsColors)/sizeof(m_LedsColors[0]); i += 3) {
        ledColor.setRed(m_LedsColors[i]);
        ledColor.setGreen(m_LedsColors[i+1]);
        ledColor.setBlue(m_LedsColors[i+2]);
        m_Brush.setColor(ledColor);
        painter.setBrush(m_Brush);
        painter.drawRect(QRect(xLoc, yLoc, LED_WIDTH, LED_WIDTH));
        xLoc += LED_WIDTH;
    }

    painter.restore();

    // paint the frame?
    painter.setRenderHint(QPainter::Antialiasing, false);
    painter.setPen(palette().dark().color());
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(QRect(0, 0, width() - 1, height() - 1));
}







