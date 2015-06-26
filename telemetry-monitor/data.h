/**
 * @file data.h
 * Data class to read incoming serial data.
 *
 * @author George Schwieters
 * @author Andrew Mass
 * @date Created:
 * @date Modified: 2014-05-19
 */
#ifndef DATA_H
#define DATA_H

#include <map>
#include <vector>
#include <QIODevice>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>
#include "config.h"

using std::map;
using std::vector;

#define OT      0x11
#define ET      0x22
#define VOLT    0x33
#define OP      0x44
#define SPD     0x55
#define RPM     0x66
#define LAMD    0x77

#define DATA    0x01
#define ADDR    0x02
#define ERROR   0x03

#define INITIAL_REMAINDER 0xFFFFFFFF

#define BAUD_RATE QSerialPort::Baud9600

typedef unsigned long crc;

// Include this line so we can use a reference to the AppDisplay class.
class AppDisplay;

/**
 * Class which handles the reading and decoding of data sent over the serial
 * connection to the telemetry board on the car.
 */
class AppData : public QObject {
  Q_OBJECT

  public:

    /**
     * This function is called by the AppDisplay class when we first need to
     * open the serial port. All configuration of the serial port should be
     * handled.
     *
     * @param serialPort A reference to the QSerialPort to configure.
     * @param display A reference to the AppDisplay that we should update.
     * @returns A boolean representing whether function completed successfully.
     */
    void openSerialPort(QSerialPort & serialPort, AppDisplay & display);

    /**
     * This function closes an opened QSerialPort.
     *
     * @param serialPort A reference to the QSerialPort to close.
     */
    void closeSerialPort(QSerialPort & serialPort);

    /**
     * This function is called by the AppDisplay class when a readyRead()
     * signal is received. It should read a packet of data from the serial
     * port, then call the AppDisplay::updateData() method to update the
     * displays of any CAN messages that were received.
     *
     * @param serialPort A reference to the QSerialPort through which we
     *     should read the data.
     * @param display A reference to the AppDisplay that we should update.
     */
    void readData(QSerialPort & serialPort, AppDisplay & display);

  private:
    int prev_message_counter;

    int num_dropped_messages;

    map<unsigned char, message> messages;

    bool configureSerialPort(QSerialPort & serialPort,
        QString serialPortName, int serialPortBaudRate);

    bool updateChannelData(QSerialPort & serialPort,
        map<unsigned char, int> &channelData, long & messageCount);

    crc crcFast(QByteArray message, int nBytes);
};

#endif // DATA_H
