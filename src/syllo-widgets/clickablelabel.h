#ifndef _CLICKABLE_LABEL_
#define _CLICKABLE_LABEL_

#include <QLabel>
#include <QMouseEvent>

class ClickableLabel : public QLabel
{
 
Q_OBJECT
 

public:
     explicit ClickableLabel( QWidget * parent = 0, Qt::WindowFlags f = 0);
     //explicit ClickableLabel( const QString& text ="", QWidget * parent = 0, Qt::WindowFlags f = 0);
     ~ClickableLabel();
 
signals:
     //void clicked();
     void mousePressed( const QPoint& );
 
protected:
     void mousePressEvent ( QMouseEvent * event ) ;
};

#endif
