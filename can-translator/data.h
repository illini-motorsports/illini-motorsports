/**
 * @file data.h
 * Handles the reading, conversion, and writing of data.
 *
 * @author Andrew Mass
 * @date Created: 2014-07-12
 * @date Modified: 2014-07-12
 */
#ifndef DATA_H
#define DATA_H

#include <QObject>
#include <iostream>
#include <fstream>
#include <map>
#include "config.h"

using std::cout;
using std::endl;
using std::ifstream;
using std::ios;
using std::map;

/**
 * Class which handles the scanning of CAN messages from config.txt.
 */
class AppData : public QObject {
  Q_OBJECT

  public:
    /**
     * Opens up a data file and iterates through it, converting the raw data
     * to a format that can be imported into Darab.
     */
    void readData();

  signals:

    /**
     * Error handler that will be connected to an error function in the display
     * class. Calling this method will display a error message box.
     *
     * @params error The error message to display.
     */
    void error(QString error);
};

#endif // DATA_H
