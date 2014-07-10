/**
 * @file config.h
 * Handles the scanning of CAN messages from config.txt.
 *
 * @author Andrew Mass
 * @date Created: 2014-06-24
 * @date Modified: 2014-07-04 #AMERICA
 */
#ifndef CONFIG_H
#define CONFIG_H

#include <map>
#include <QString>
#include <QVector>
#include <QFile>
#include <QTextStream>
#include "display.h"

using std::map;

/**
 * Struct that represents a specific channel in a CAN message. Contains
 * information about the position of the channel in the message, the title of
 * the channel, a string representing the units, and a scalar for the channel.
 */
struct channel {
  unsigned char pos;
  QString title;
  bool isSigned;
  double scalar;
  double offset;
  QString units;
};

/**
 * Struct that represents a CAN message. Contains information about the CAN ID
 * of the message, the data length, the byte order, an array of structs that
 * hold all of the channels contained in the message, and a timestamp value.
 */
struct message {
  unsigned short id;
  unsigned char dlc;
  bool isBigEndian;
  QVector<channel> channels;

  message() {
    id = 0;
    dlc = 0;
    isBigEndian = false;
  }
};

/**
 * Class which handles the scanning of CAN messages from config.txt.
 */
class AppConfig : public QObject {
  Q_OBJECT

  public:

    /**
     * Reads config.txt from the current directory and extracts CAN message
     * information from the file. Builds a map from the CAN message ID to a struct
     * containing all the necessary information about the message.
     *
     * @returns A map from message IDs to a message struct.
     */
    map<unsigned char, message> getMessages();

  signals:

    /**
     * Error handler that will be connected to an error function in the display
     * class. Calling this method will display a error message box.
     *
     * @params error The error message to display.
     */
    void error(QString error);

  private:

    /**
     * Opens an input stream from config.txt and reads the file into a vector of
     * strings, where each element in the vector is a CAN message. Lines that
     * are blank or comments are ignored.
     *
     * @returns An array of each valid line read from the config file.
     */
    QVector<QString> readFile();

    /**
     * Takes a single line from the config file as input and returns a message
     * struct with all applicable data. Throws errors if input data is missing
     * or malformed.
     *
     * @params line A string representing a single line from the config file.
     * @returns A message struct with all applicable data.
     */
    message getMessage(QString line);

    /**
     * Takes a single section from a line in the config file and returns a
     * channel struct with all applicable data. Throws errors if input data is
     * missing or malformed.
     *
     * @params pos The position of this section in the parent line.
     * @params section The section of the line.
     * @returns A channel struct with all applicable data.
     */
    channel getChannel(int pos, QString section);
};

#endif // CONFIG_H
