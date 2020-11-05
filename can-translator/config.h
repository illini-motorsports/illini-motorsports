/**
 * Config Header
 *
 * @author Andrew Mass
 * @date Created: 2014-06-24
 * @date Modified: 2016-03-13
 */
#ifndef CONFIG_H
#define CONFIG_H

#include <QCoreApplication>
#include <QFile>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <QVector>
#include <map>
#include <stdint.h>

using std::map;

// Struct that represents a specific signal in a CAN message definition
struct Signal {
  QString title;
  QString units;
  uint8_t startBit;
  uint8_t bitLen;
  bool isSigned;
  bool isBigEndian;
  double scalar;
  double offset;
  double min;
  double max;

  Signal() {
    title = "";
    units = "";
    startBit = 0;
    bitLen = 0;
    isSigned = false;
    isBigEndian = false;
    scalar = 0.0;
    offset = 0.0;
    min = 0;
    max = 0;
  }

  bool valid() { return !(title.isEmpty() && units.isEmpty()); }

  QString toString() {
    return title + "<" + units + ">" + " isS: " + (isSigned ? "T" : "F") +
           " isBE: " + (isBigEndian ? "T" : "F") +
           " S: " + QString::number(scalar) + " O: " + QString::number(offset) +
           " sb: " + QString::number(startBit) +
           " bl: " + QString::number(bitLen) + " min: " + QString::number(min) +
           " max: " + QString::number(max);
  }
};

// Struct that represents one CAN message definition
struct Message {
  uint16_t id;
  uint8_t dlc;
  QVector<Signal> sigs;

  Message() {
    id = 0;
    dlc = 0;
  }

  bool valid() { return !(id == 0 && dlc == 0 && sigs.size() == 0); }

  QString toString() {
    return "0x" + QString::number(id, 16) + " - DLC: " + QString::number(dlc) +
           " Signals: " + QString::number(sigs.size());
  }
};

/**
 * Class which configures the program to use the CAN spec specified in the
 * config file.
 */
class AppConfig : public QObject {
  Q_OBJECT

public:
  map<uint16_t, Message> getMessages();

signals:

  /**
   * Error handler that will be connected to an error function in the display
   * class. Calling this method will display a error message box.
   *
   * @params error The error message to display.
   */
  void error(QString error);

private:
  QVector<QVector<QString>> readFile();
  Message getMessage(QVector<QString> messageBlock);
  Signal getSignal(QString signalDef);
};

#endif // CONFIG_H
