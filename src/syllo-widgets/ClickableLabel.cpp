#include <QtPlugin>

#include "clickablelabel.h"

ClickableLabel::ClickableLabel(QWidget * parent, Qt::WindowFlags f) : QLabel(parent,f)
{
     
}

//ClickableLabel::ClickableLabel(const QString& text, QWidget * parent, Qt::WindowFlags f ) : QLabel(parent,f)
//{
//     this->setText(text);
//}
 
ClickableLabel::~ClickableLabel()
{
}
 
void ClickableLabel::mousePressEvent ( QMouseEvent * event )
{
     //const QPoint p = event->pos();
     //emit mousePressed( p );
    //emit clicked();
}

//Q_EXPORT_PLUGIN2(myclickablelabel,ClickableLabel)
Q_EXPORT_PLUGIN2(myclickablelabel,ClickableLabel)
//Q_EXPORT_PLUGIN2(myclickablelabel,ClickableLabel::ClickableLabel(const QString&, QWidget*, Qt::WindowFlags))
