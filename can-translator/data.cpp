/**
 * @file data.cpp
 * Implementation of the AppData class.
 *
 * @author Andrew Mass
 * @date Created: 2014-07-12
 * @date Modified: 2014-10-18
 */
#include "data.h"

bool AppData::readData(bool isVectorFile) {
  return isVectorFile ? readDataVector() : readDataCustom();
}

bool AppData::readDataCustom() {
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

bool AppData::readDataVector() {
  QFile inputFile(this->filename);
  if(inputFile.open(QIODevice::ReadOnly)) {
    QTextStream inputStream(&inputFile);

    inputStream.readLine();
    int lineCounter = 0;
    while(!inputStream.atEnd()) {
      inputStream.readLine();
      lineCounter++;
    }
    inputStream.seek(0);

    int progressCounter = 0;
    int iter = 0;
    inputStream.readLine();
    while(!inputStream.atEnd()) {
      QString line = inputStream.readLine();
      if(!line.isEmpty()) {
        processLine(line.simplified());
      }

      if((((double) iter++) / ((double) lineCounter)) * 100.0 > progressCounter) {
        emit progress(++progressCounter);
      }
    }

    inputFile.close();
    return true;
  }
  emit error("Problem with input file. Try again or try another file.");
  return false;
}

bool AppData::writeAxis() {
  QString outFilename = this->filename;
  outFilename.replace(".txt", ".out.txt", Qt::CaseInsensitive);
  ofstream outFile(outFilename.toLocal8Bit().data(), ios::out | ios::trunc);

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
  QString outFilename = this->filename;
  outFilename.replace(".txt", ".out.txt", Qt::CaseInsensitive);
  ofstream outFile(outFilename.toLocal8Bit().data(), ios::out | ios::app);

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
  static int badMsgCounter = 0;
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
        if(++badMsgCounter < 6) {
            emit error(QString("Invalid msgId: %1").arg(msgId, 0, 16));
        } else if(badMsgCounter == 7) {
            emit error(QString("Invalid msgId maxed out."));
        }
      }
      badMsgFound = true;
      iter += 2;
    }
  }

  delete[] buffer;
  buffer = NULL;
}

void AppData::processLine(QString line) {
  AppConfig config;
  map<unsigned short, Message> messages = config.getMessages();

  QStringList sections = line.split(" ", QString::SkipEmptyParts);

  if(sections.size() == 2 && sections[1].compare("Trigger") == 0) {
    return;
  }

  if(sections.size() < 7) {
    emit error("Invalid log file line.");
    return;
  }

  bool successful = true;
  unsigned short msgId;
  msgId = sections[2].toUInt(&successful, 10);
  if(!successful) {
    emit error("Invalid message ID.");
    return;
  }

  Message msg = messages[msgId];

  if(msg.valid()) {
    vector<bool> msgEnabled = this->enabled[msg.id];

    int j = 0;
    for(int i = 0; i < msg.channels.size(); i++) {
      Channel chn = msg.channels[i];

      if(msgEnabled[i]) {
        double value;

        successful = true;
        unsigned char byteOne = sections[5 + (i * 2)].toUInt(&successful, 10);
        unsigned char byteTwo = sections[5 + (i * 2) + 1].toUInt(&successful, 10);
        if(!successful) {
          emit error("Invalid channel data.");
          return;
        }

        if(chn.isSigned) {
          signed int data = msg.isBigEndian ? byteOne << 8 | byteTwo :
            byteTwo << 8 | byteOne;
          value = (double) data;
        } else {
          unsigned int data = msg.isBigEndian ? byteOne << 8 | byteTwo :
            byteTwo << 8 | byteOne;
          value = (double) data;
        }
        latestValues[messageIndices[msg.id]][j] = (value - chn.offset) * chn.scalar;
        j++;
      }
    }

    latestValues[0][0] = sections[0].toDouble();

    writeLine();
  } else {
    //emit error(QString("Invalid msgId: %1").arg(msgId, 0, 16));
  }
}
