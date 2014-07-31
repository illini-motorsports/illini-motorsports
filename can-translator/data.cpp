/**
 * @file data.cpp
 * Implementation of the AppData class.
 *
 * @author Andrew Mass
 * @date Created: 2014-07-12
 * @date Modified: 2014-07-30
 */
#include "data.h"

bool AppData::readData() {
  ifstream infile(this->filename.toLocal8Bit().data(), ios::in | ios::binary);

  if(infile && infile.good()) {
    infile.seekg(0, infile.end);
    int length = infile.tellg();
    infile.seekg(0, infile.beg);

    char * bufferSigned = new char[length];
    infile.read(bufferSigned, length);
    unsigned char * buffer = (unsigned char *) bufferSigned;
    bufferSigned = NULL;

    if(!infile || !infile.good()) {
      emit error("Problem reading input file. Try again or try another file.");
      return false;
    }

    infile.close();

    processBuffer(buffer, length);
    return true;
  } else {
    emit error("Problem with input file. Try again or try another file.");
    return false;
  }
}

bool AppData::writeAxis() {
  ofstream outFile("out.txt", ios::out | ios::trunc);

  if(outFile && outFile.good()) {
    outFile << "xtime [s]";

    latestValues.clear();
    messageIndices.clear();

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
      vector<bool> msgEnabled = this->enabled[msg.id];

      for(int i = 0; i < msg.channels.size(); i++) {
        Channel chn = msg.channels[i];
        if(msgEnabled[i]) {
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
  } else {
    emit error("Problem opening output file.");
    return false;
  }
}

void AppData::writeLine() {
  ofstream outFile("./out.txt", ios::out | ios::app);

  if(outFile && outFile.good()) {
    outFile << endl;

    for(unsigned int i = 0; i < latestValues.size(); i++) {
      for(unsigned int j = 0; j < latestValues[i].size(); j++) {
        if(i != 0) {
          outFile << " ";
        }
        outFile << latestValues[i][j];
      }
    }

    outFile.close();
  } else {
    emit error(QString("Problem opening output file."));
  }
}

void AppData::processBuffer(unsigned char * buffer, int length) {
  AppConfig config;
  map<unsigned short, Message> messages = config.getMessages();

  emit progress(0);

  int iter = 0;
  int progressCounter = 0;
  bool badMsgFound = false;
  while(iter + 14 < length) {

    if((((double) iter) / ((double) length)) * 100.0 > progressCounter) {
      emit progress(++progressCounter);
    }

    unsigned short msgId = buffer[iter + 1] << 8 | buffer[iter];
    iter += 2;

    Message msg = messages[msgId];
    if(msg.valid()) {
      badMsgFound = false;
      vector<bool> msgEnabled = this->enabled[msg.id];

      int j = 0;
      for(int i = 0; i < msg.channels.size(); i++) {
        Channel chn = msg.channels[i];

        if(msgEnabled[i]) {
          double value;
          if(chn.isSigned) {
            signed int data = msg.isBigEndian ? buffer[iter] << 8 | buffer[iter + 1] : buffer[iter + 1] << 8 | buffer[iter];
            value = (double) data;
          } else {
            unsigned int data = msg.isBigEndian ? buffer[iter] << 8 | buffer[iter + 1] : buffer[iter + 1] << 8 | buffer[iter];
            value = (double) data;
          }
          latestValues[messageIndices[msg.id]][j] = (value - chn.offset) * chn.scalar;
          j++;
        }
        iter += 2;
      }

      double upper = buffer[iter + 3] << 8 | buffer[iter + 2];
      double lower = ((double) (buffer[iter + 1] << 8 | buffer[iter])) / 0x8000;
      latestValues[0][0] = (double) (upper + lower - 1.0);
      iter += 4;

      writeLine();
    } else {
      if(!badMsgFound) {
        emit error(QString("Invalid msgId: %1").arg(msgId, 0, 16));
      }
      badMsgFound = true;
      iter += 2;
    }
  }

  delete[] buffer;
  buffer = NULL;
}
