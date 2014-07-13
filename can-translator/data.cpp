/**
 * @file data.cpp
 * Implementation of the AppData class.
 *
 * @author Andrew Mass
 * @date Created: 2014-07-12
 * @date Modified: 2014-07-12
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
      }

      //TODO: Read and process data.
      it += msg.dlc;

      double upper = buffer[it + 3] << 8 | buffer[it + 2];
      double lower = ((double) (buffer[it + 1] << 8 | buffer[it])) / 0x8000;
      it += 4;

      double timestamp = upper + lower - 1.0;
      cout << msg.id << " " << timestamp << endl;
    }

    delete[] buffer;
  }
}
