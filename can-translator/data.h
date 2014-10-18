/**
 * @file data.h
 * Handles the reading, conversion, and writing of data.
 *
 * @author Andrew Mass
 * @date Created: 2014-07-12
 * @date Modified: 2014-10-18
 */
#ifndef DATA_H
#define DATA_H

#include <map>
#include <fstream>
#include <QObject>
#include <vector>
#include "config.h"

using std::ios;
using std::map;
using std::endl;
using std::vector;
using std::ifstream;
using std::ofstream;

/**
 * Class which handles the scanning of CAN messages from config.txt.
 */
class AppData : public QObject {
  Q_OBJECT

  public:

    /**
     * Opens up a data file and iterates through it, converting the raw data
     * to a format that can be imported into Darab.
     *
     * @params isVectorFile Whether the file to convert is in the Vector format.
     * @returns Whether the read was successful.
     */
    bool readData(bool isVectorFile);

    /**
     * Prints all the channels to the output file.
     *
     * @returns Whether the write was successful.
     */
    bool writeAxis();

    /**
     * Prints a line of the latest data.
     */
    void writeLine();

    /**
     * The name of the file to convert.
     */
    QString filename;

    /**
     * A map of message IDs to arrays of bools that contains info about which
     * channels the user selected to be converted.
     */
    map<unsigned short, vector<bool> > enabled;

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

    /**
     * A map of messageId to the position of the specified message in the
     * array of printed values.
     */
    map<unsigned short, int> messageIndices;

    /**
     * An array that holds the latest values for each message type. When a
     * message is read and converted, the new values are placed in the
     * appropriate spot in this array. Then the whole array is printed to a
     * line to output the new message and the latest values of other messages.
     */
    vector< vector<double> > latestValues;

    /**
     * Converts data from our custom uSD logging protocol. Opens up a data
     * file and iterates through it, converting the raw data to a format that
     * can be imported into Darab.
     *
     * @returns Whether the read was successful.
     */
    bool readDataCustom();

    /**
     * Converts data from the Vector logging protocol. Opens up a data
     * file and iterates through it, converting the raw data to a format that
     * can be imported into Darab.
     *
     * @returns Whether the read was successful.
     */
    bool readDataVector();

    /**
     * Loops through the provided buffer and converts the data within into the
     * output format. As the data is converted, it is written to the output file.
     *
     * @param buffer The data buffer to convert.
     * @param length The length of the buffer. (Thanks c++).
     */
    void processBuffer(unsigned char * buffer, int length);

    /**
     * Takes a single line of input from a Vector log file, converts the data,
     * and writes it to the output file.
     *
     * @param line The line of input from a Vector log file.
     */
    void processLine(QString line);
};

#endif // DATA_H
