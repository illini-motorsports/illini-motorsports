/**
 * @file main.cpp
 * The main runner for the can-translator program.
 *
 * @author Andrew Mass
 * @date Created: 2014-06-24
 * @date Modified: 2014-06-24
 */
#include <QApplication>
#include "display.h"

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);

  AppDisplay display;
  display.show();

  if(display.successful) {
    return app.exec();
  } else {
    return 1;
  }
}

