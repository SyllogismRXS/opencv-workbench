#include <QApplication>
#include <QPushButton>
 
int main(int argc, char **argv)
{
    QApplication app (argc, argv);
 
    QPushButton button ("Hello world !");
    button.setText("My Text");
    button.setToolTip("A tooltip!");
    button.show();
 
    return app.exec();
}
