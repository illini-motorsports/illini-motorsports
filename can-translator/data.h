/**
 * @file data.h
 * Handles the reading, conversion, and writing of data.
 *
 * @author Andrew Mass
 * @date Created: 2014-07-12
 * @date Modified: 2014-07-16
 */
#ifndef DATA_H
#define DATA_H

#include <QObject>
#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include "config.h"

using std::endl;
using std::ifstream;
using std::ofstream;
using std::ios;
using std::map;
using std::vector;

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

    /**
     * Prints all the channels to the output file.
     */
    void writeAxis();

    /**
     * Prints a line of the latest data.
     */
    void writeLine();

  signals:

    /**
     * Error handler that will be connected to an error function in the display
     * class. Calling this method will display a error message box.
     *
     * @params error The error message to display.
     */
    void error(QString error);

    /**
     * Updates the progress bar with the latest progress counter.
     *
     * @param progress The latest progress counter to display.
     */
    void progress(int progress);

  private:

    map<unsigned short, int> messageIndices;

    vector< vector<double> > latestValues;
};

#endif // DATA_H
