#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // Set global Font Family to "Sans Serif"
    // QFont(const QString & family, int pointSize = -1, int weight = -1, bool italic = false)
    //QFont newFont("Sans Serif", -1, -1, false);
    //QApplication::setFont(newFont);

    MainWindow w;
    w.show();

    return a.exec();
}
