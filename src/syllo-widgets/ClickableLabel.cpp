#include <iostream>
#include <QtPlugin>

#include "clickablelabel.h"

ClickableLabel::ClickableLabel(QWidget * parent, Qt::WindowFlags f) : QLabel(parent,f)
{
     initialized = false;
     setMouseTracking(true);
     
     setScaledContents(true);
}

//ClickableLabel::ClickableLabel(const QString& text, QWidget * parent, Qt::WindowFlags f ) : QLabel(parent,f)
//{
//     this->setText(text);
//}
 
ClickableLabel::~ClickableLabel()
{
}

QSize ClickableLabel::sizeHint() const
{
     QSize size;
     if (pixmap() == 0) {
          size = QWidget::sizeHint();
     } else {
          size = pixmap()->size();
     }
     //std::cout << "sizeHint: " << size.rwidth() << "," <<  size.rheight() << std::endl; 
     return size;
}

void ClickableLabel::mousePressEvent ( QMouseEvent * event )
{
     //const QPoint p = event->pos();
     emit mousePressed( event );
     //emit clicked();
}

void ClickableLabel::mouseReleaseEvent ( QMouseEvent * event )
{
     //const QPoint p = event->pos();
     emit mouseReleased( event );
     //emit clicked();
}

void ClickableLabel::mouseMoveEvent ( QMouseEvent * event )
{
     //const QPoint p = event->pos();
     emit mouseMoved( event );
     //emit clicked();
}

void ClickableLabel::initialize(QDesignerFormEditorInterface *)
{
     if (initialized)
          return;

     initialized = true;
}

bool ClickableLabel::isInitialized() const
{
     return initialized;
}

QWidget *ClickableLabel::createWidget(QWidget *parent)
{
     return new ClickableLabel(parent);
}

QString ClickableLabel::name() const
{
     return "ClickableLabel";
}

QString ClickableLabel::group() const
{
     return "Display Widgets [Examples]";
}

QIcon ClickableLabel::icon() const
{
     return QIcon();
}

QString ClickableLabel::toolTip() const
{
     return "";
}

QString ClickableLabel::whatsThis() const
{
     return "";
}

bool ClickableLabel::isContainer() const
{
     return false;
}

QString ClickableLabel::domXml() const
{
     return "<ui language=\"c++\">\n"
          " <widget class=\"ClickableLabel\" name=\"clickableLabel\">\n"
          "  <property name=\"geometry\">\n"
          "   <rect>\n"
          "    <x>0</x>\n"
          "    <y>0</y>\n"
          "    <width>100</width>\n"
          "    <height>100</height>\n"
          "   </rect>\n"
          "  </property>\n"
          "  <property name=\"toolTip\" >\n"
          "   <string>The current time</string>\n"
          "  </property>\n"
          "  <property name=\"whatsThis\" >\n"
          "   <string>A clickable label for image processing.</string>\n"
          "  </property>\n"
          " </widget>\n"
          "</ui>\n";
}

QString ClickableLabel::includeFile() const
{
     return "clickablelabel.h";
}

Q_EXPORT_PLUGIN2(clickablelabel,ClickableLabel)
