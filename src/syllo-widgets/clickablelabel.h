#ifndef _CLICKABLE_LABEL_
#define _CLICKABLE_LABEL_

#include <QLabel>
#include <QMouseEvent>
#include <QtDesigner/QDesignerCustomWidgetInterface>
//#include <customwidget.h>

class ClickableLabel : public QLabel, public QDesignerCustomWidgetInterface
{
 
Q_OBJECT
Q_INTERFACES(QDesignerCustomWidgetInterface) 

public:
     explicit ClickableLabel( QWidget * parent = 0, Qt::WindowFlags f = 0);
     //explicit ClickableLabel( const QString& text ="", QWidget * parent = 0, Qt::WindowFlags f = 0);
     ~ClickableLabel();

     QSize sizeHint() const;     
     bool isContainer() const;
     bool isInitialized() const;
     QIcon icon() const;
     QString domXml() const;
     QString group() const;
     QString includeFile() const;
     QString name() const;
     QString toolTip() const;
     QString whatsThis() const;
     QWidget *createWidget(QWidget *parent);
     void initialize(QDesignerFormEditorInterface *core);
 
signals:
     //void clicked();
     void mousePressed( const QPoint& );
     void mouseReleased( const QPoint& );
     void mouseMoved( const QPoint& );
 
protected:
     void mousePressEvent ( QMouseEvent * event ) ;
     void mouseReleaseEvent ( QMouseEvent * event ) ;
     void mouseMoveEvent ( QMouseEvent * event ) ;

private:
     bool initialized;
};

#endif
