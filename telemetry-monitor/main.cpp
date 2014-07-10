/**
 * @file main.cpp
 * The main runner for the telemetry-monitor program.
 *
 * @author Andrew Mass
 * @author George Schwieters
 * @date Created: 2014-03-03
 * @date Modified: 2014-05-06
 */
#include <QApplication>
#include "display.h"

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);

  AppDisplay display;
  display.show();

  return app.exec();
}

