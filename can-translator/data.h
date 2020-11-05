/**
 * @file data.h
 * Handles the reading, conversion, and writing of data.
 *
 * @author Andrew Mass
 * @date Created: 2014-07-12
 * @date Modified: 2015-06-07
 */
#ifndef DATA_H
#define DATA_H

#include "config.h"
#include <QObject>
#include <fstream>
#include <map>
#include <vector>

using std::endl;
using std::ifstream;
using std::ios;
using std::map;
using std::ofstream;
using std::vector;

#define LOGFILE_COALESCE_SEPARATION 30.0

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
   * Combines multiple logfiles into a single coalesced logfile. This function
   * will also add a column in the data for the logfile name, in order to be
   * able to determine which logfile a certain section of data is from. In
   * addition, the function makes the timestamps relative to each other so that
   * Darab shows the files in the correct strictly increasing order.
   *
   * @param filenames The list of logfile names to coalesce.
   * @returns Whether the coalesce was successful.
   */
  bool coalesceLogfiles(QStringList filenames);

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
  vector<vector<double>> latestValues;

  /**
   * Holds the latest values for the timestamp of the message
   */
  QString latestTimestamp;

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
  void processBuffer(unsigned char *buffer, int length);

  /**
   * Takes a single line of input from a Vector log file, converts the data,
   * and writes it to the output file.
   *
   * @param line The line of input from a Vector log file.
   */
  void processLine(QString line);

  /**
   * Stores the scanned CAN spec configuration.
   */
  map<uint16_t, Message> messages;

  /**
   * Stores the open handle to the output file.
   */
  ofstream outFile;
};

#endif // DATA_H
