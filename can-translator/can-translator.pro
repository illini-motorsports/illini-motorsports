TEMPLATE = app
TARGET = can-translator
DEPENDPATH += .
INCLUDEPATH += .

QT += gui core widgets
CONFIG += qt
CONFIG += c++11

HEADERS += config.h data.h display.h
SOURCES += config.cpp data.cpp display.cpp main.cpp
