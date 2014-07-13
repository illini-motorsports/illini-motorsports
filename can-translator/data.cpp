/**
 * @file data.cpp
 * Implementation of the AppData class.
 *
 * @author Andrew Mass
 * @date Created: 2014-07-12
 * @date Modified: 2014-07-13
 */
#include "data.h"

void AppData::readData() {
  ifstream infile("./0015.TXT", ios::in|ios::binary);

  if(infile) {
    infile.seekg(0, infile.end);
    int length = infile.tellg();
    infile.seekg (0, infile.beg);

    char * bufferSigned = new char[length];
    infile.read(bufferSigned, length);
    unsigned char * buffer = (unsigned char *) bufferSigned;

    if(!infile || !infile.good()) {
      emit error(QString("Only %1 bytes were read").arg(infile.gcount()));
    }

    infile.close();

    AppConfig config;
    map<unsigned short, Message> messages = config.getMessages();

    unsigned int it = 0;
    while(it + 14 < length) {
      unsigned short msgId = buffer[it + 1] << 8 | buffer[it];
      it += 2;

      Message msg = messages[msgId];
      if(msg.id == 0 && msg.dlc == 0 && msg.channels.size() == 0) {
        emit error(QString("Invalid msgId: %1").arg(msgId, 0, 16));
        it += 12;
      } else {
        cout << "ID:: " << QString::number(msg.id, 16).toStdString() << endl;
        for(int i = 0; i < msg.channels.size(); i++) {
          Channel chn = msg.channels[i];
          double value;
          if(chn.isSigned) {
            signed int data = msg.isBigEndian ? buffer[it + 1] << 8 | buffer[it] : buffer[it] << 8 | buffer[it + 1];
            value = (double) data;
          } else {
            unsigned int data = msg.isBigEndian ? buffer[it + 1] << 8 | buffer[it] : buffer[it] << 8 | buffer[it + 1];
            value = (double) data;
          }
          cout << "  " << (value * chn.scalar) - chn.offset << " " << chn.units.toStdString() << endl;
          it += 2;
        }

        double upper = buffer[it + 3] << 8 | buffer[it + 2];
        double lower = ((double) (buffer[it + 1] << 8 | buffer[it])) / 0x8000;
        it += 4;

        double timestamp = upper + lower - 1.0;
        cout << "TIME:: " << timestamp << endl << endl;
      }
    }

    delete[] buffer;
  }
}
