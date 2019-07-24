#include "../include/prototype/gui/mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  prototype::MainWindow w(argc, argv);
  w.threading();
  w.show();
  
  return a.exec();
}
