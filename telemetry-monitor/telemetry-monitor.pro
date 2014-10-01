QT += gui core widgets serialport
CONFIG += qt serialport

DEPENDPATH += . telemetry-monitor
INCLUDEPATH += .

HEADERS += display.h data.h config.h
SOURCES += display.cpp main.cpp data.cpp
