/*
 * @file data.cpp
 * Implementation of the AppData class.
 *
 * @author George Schwieters
 * @author Andrew Mass
 * @date Created:
 * @date Modified: 2015-05-11
 */
#include "data.h"
#include "display.h"

void AppData::openSerialPort(QSerialPort & serialPort, AppDisplay & display) {
  messages = init_map();

  QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
  QSerialPortInfo port;
  if(ports.length() > 0) {
    port = ports[0];
  } else {
    display.showMessage("Serial Port Info", "No available serial ports.");
    return;
  }

  // This is specific to the Raspberry Pi.
  if(port.portName().compare("ttyAMA0") == 0) {
    port = ports[1];
  }

  serialPort.setPortName(port.portName());

  if(!serialPort.open(QIODevice::ReadOnly)) {
    display.showMessage("Serial Port Info", serialPort.errorString());
    return;
  }

  serialPort.setBaudRate(BAUD_RATE);
  serialPort.setDataBits(QSerialPort::Data8);
  serialPort.setParity(QSerialPort::NoParity);
  serialPort.setStopBits(QSerialPort::OneStop);
  serialPort.setFlowControl(QSerialPort::NoFlowControl);

  num_dropped_messages = 0;
  prev_message_counter = 0;

  display.updateDropCounter(num_dropped_messages);
}

void AppData::closeSerialPort(QSerialPort & serialPort) {
  serialPort.close();
}

void AppData::readData(QSerialPort & serialPort, AppDisplay & display) {
    static QByteArray read_Data;
    static int messageCount = 0;
    static vector<unsigned char> theMsg;
    static vector<int> channelAddr;
    static std::map<unsigned char, unsigned int> channelData;

    channelAddr.resize(6);
    channelAddr[0] = OT;
    channelAddr[1] = ET;
    channelAddr[2] = VOLT;
    channelAddr[3] = OP;
    channelAddr[4] = SPD;
    channelAddr[5] = RPM;

    read_Data.append(serialPort.readAll());

    if(read_Data.size() >= 4 + 12) {
        if((unsigned char)read_Data[0] == 0x80 && (unsigned char)read_Data[1] == 0x81 && (unsigned char)read_Data[2] == 0x82 && (unsigned char)read_Data[3] == 0x83) {
            theMsg.clear();

            // Get message bytes
            for(int i = 4; i < 12 + 4; i++) {
                theMsg.push_back(read_Data[i]);
            }

            // Insert data into channel array
            channelData[OT] = (theMsg[0] * 0x0100) + theMsg[1];
            channelData[ET] = (theMsg[2] * 0x0100) + theMsg[3];
            channelData[VOLT] = (theMsg[4] * 0x0100) + theMsg[5];
            channelData[OP] = (theMsg[6] * 0x0100) + theMsg[7];
            channelData[SPD] = (theMsg[8] * 0x0100) + theMsg[9];
            channelData[RPM] = (theMsg[10] * 0x0100) + theMsg[11];

            display.errorMessage(false);

            // Increase message count
            messageCount++;
            display.updateMessageCounter(messageCount);

            // Clean up now that we're done
            read_Data.remove(0, 4 + 12);
            theMsg.clear();

            for(unsigned int i = 0; i < channelAddr.size(); i++) {
              if(messages.count(channelAddr[i]) > 0) {
                display.updateData(channelAddr[i], ((double) channelData[channelAddr[i]]) * messages.at(channelAddr[i]).scalar);
              }
            }
        } else {
            // Something is wrong. Restart
            read_Data.clear();
            theMsg.clear();
        }
    }

    // Set the display to connected if a valid message has been received.
    if(messageCount > 0) {
      display.setConnected(true);
    }
}
