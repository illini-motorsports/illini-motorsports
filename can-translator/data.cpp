/**
 * @file data.cpp
 * Implementation of the AppData class.
 *
 * @author Andrew Mass
 * @date Created: 2014-07-12
 * @date Modified: 2014-07-19
 */
#include "data.h"

bool AppData::readData() {
  ifstream infile(this->filename.toLocal8Bit().data(), ios::in|ios::binary);

  if(infile) {
    infile.seekg(0, infile.end);
    int length = infile.tellg();
    infile.seekg (0, infile.beg);

    char * bufferSigned = new char[length];
    infile.read(bufferSigned, length);
    unsigned char * buffer = (unsigned char *) bufferSigned;

    emit progress(0);

    if(!infile || !infile.good()) {
      emit error(QString("Only %1 bytes were read").arg(infile.gcount()));
    }

    infile.close();

    AppConfig config;
    map<unsigned short, Message> messages = config.getMessages();

    int it = 0;
    int progressCounter = 0;
    bool badMessage = false;
    while(it + 14 < length) {

      if((((double) it) / ((double) length)) * 100.0 > progressCounter) {
        emit progress(++progressCounter);
      }

      unsigned short msgId = buffer[it + 1] << 8 | buffer[it];
      it += 2;

      Message msg = messages[msgId];
      if(msg.id == 0 && msg.dlc == 0 && msg.channels.size() == 0) {
        if(!badMessage) {
          emit error(QString("Invalid msgId: %1").arg(msgId, 0, 16));
        }
        badMessage = true;
        it += 2;
      } else {
        badMessage = false;
        for(int i = 0; i < msg.channels.size(); i++) {
          Channel chn = msg.channels[i];

          if(chn.title.compare("Unused") != 0 && chn.title.compare("Rsrvd") != 0) {
            double value;
            if(chn.isSigned) {
              signed int data = msg.isBigEndian ? buffer[it] << 8 | buffer[it + 1] : buffer[it + 1] << 8 | buffer[it];
              value = (double) data;
            } else {
              unsigned int data = msg.isBigEndian ? buffer[it] << 8 | buffer[it + 1] : buffer[it + 1] << 8 | buffer[it];
              value = (double) data;
            }
            latestValues[messageIndices[msg.id]][i] = (value - chn.offset) * chn.scalar;
          }
          it += 2;
        }

        double upper = buffer[it + 3] << 8 | buffer[it + 2];
        double lower = ((double) (buffer[it + 1] << 8 | buffer[it])) / 0x8000;
        it += 4;

        double timestamp = upper + lower - 1.0;

        latestValues[0][0] = timestamp;

        writeLine();
      }
    }

    delete[] buffer;
    buffer = NULL;
    bufferSigned = NULL;
    return true;
  } else {
    emit error(QString("Problem with input file. Try again or try another file."));
    return false;
  }
}

bool AppData::writeAxis() {
  ofstream outFile("out.txt", ios::out | ios::trunc); 

  if(!outFile || !outFile.good()) {
    emit error(QString("Problem opening output file."));
    return false;
   } else {
    outFile << "xtime [s]";

    vector<double> vec_time;
    vec_time.push_back(0.0);
    latestValues.push_back(vec_time);

    AppConfig config;
    map<unsigned short, Message> messages = config.getMessages();

    int indexCounter = 1;

    typedef map<unsigned short, Message>::iterator it_msg;
    for(it_msg msgIt = messages.begin(); msgIt != messages.end(); msgIt++) {
      Message msg = msgIt->second;

      vector<double> vec_msg;

      typedef QVector<Channel>::iterator it_chn;
      for(it_chn chnIt = msg.channels.begin(); chnIt != msg.channels.end(); chnIt++) {
        Channel chn = *chnIt;
        if(chn.title.compare("Unused") != 0 && chn.title.compare("Rsrvd") != 0) {
          outFile << "  " << chn.title.toStdString() << " [" << chn.units.toStdString() << "]";
          vec_msg.push_back(0.0);
        }
      }

      latestValues.push_back(vec_msg);
      messageIndices[msg.id] = indexCounter;
      indexCounter++;
    }

    outFile.close();
    return true;
  }
}

void AppData::writeLine() {
  ofstream outFile("./out.txt", ios::out | ios::app);
  if(!outFile || !outFile.good()) {
    emit error(QString("Problem opening output file."));
  } else {
    outFile << endl;
    for(unsigned int i = 0; i < latestValues.size(); i++) {
      for(unsigned int j = 0; j < latestValues[i].size(); j++) {
        if(i != 0) {
          outFile << " ";
        }
        outFile << latestValues[i][j]; 
      }
    } 
  }
}
