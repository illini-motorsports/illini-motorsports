/**
 * @file data.cpp
 * Implementation of the AppData class.
 *
 * @author Andrew Mass
 * @date Created: 2014-07-12
 * @date Modified: 2015-06-07
 */
#include "data.h"

bool AppData::readData(bool isVectorFile) {
  QString outFilename = this->filename;
  outFilename.replace(".txt", ".out.txt", Qt::CaseInsensitive);
  outFilename.replace(".asc", ".out.txt", Qt::CaseInsensitive);
  this->outFile.open(outFilename.toLocal8Bit().data(), ios::out | ios::app);

  if (!(this->outFile && this->outFile.good())) {
    emit error(QString("Problem opening output file."));
  }

  writeAxis();

  bool success = isVectorFile ? readDataVector() : readDataCustom();

  this->outFile.close();

  return success;
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
  AppConfig config;
  this->messages = config.getMessages();

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

bool AppData::coalesceLogfiles(QStringList filenames) {
  filenames.sort();

  QStringList info = filenames.at(0).split("/");
  QString firstFileNum = info.at(info.size() - 1).split(".").at(0);

  QString lastFileName = filenames.at(filenames.size() - 1);
  info = lastFileName.split("/");
  QString lastFileNum = info.at(info.size() - 1).split(".").at(0);

  QString directory = lastFileName.left(lastFileName.lastIndexOf('/') + 1);

  QString outFilename = QString("%1coalesce-%2-%3.txt").arg(directory)
    .arg(firstFileNum).arg(lastFileNum);
  ofstream outFileCoal(outFilename.toLocal8Bit().data(), ios::out | ios::trunc);

  if(outFileCoal && outFileCoal.good()) {
    QString header;
    QFile firstFile(filenames.at(0));
    if(firstFile.open(QIODevice::ReadOnly)) {
      QTextStream inputStream(&firstFile);
      header = inputStream.readLine();
      QString adjHeader = QString("%1  %2  %3").arg(header.section("  ", 0, 0)).arg("Logfile [file]")
        .arg(header.section("  ", 1));
      outFileCoal << adjHeader.toLocal8Bit().data() << '\n';
      firstFile.close();
    }

    double latestTimestamp = -LOGFILE_COALESCE_SEPARATION;

    for(int i = 0; i < filenames.size(); i++) {
      emit progress((((double) i) / filenames.size()) * 100.0);

      info = filenames.at(i).split("/");
      QString logNum = info.at(info.size() - 1).split(".").at(0);

      QFile inputFile(filenames.at(i));
      if(inputFile.open(QIODevice::ReadOnly)) {
        QTextStream inputStream(&inputFile);

        if(QString::compare(header, inputStream.readLine())) {
          emit error("Header mismatch. All logfiles must have the same headers.");
          outFileCoal.close();
          return false;
        }

        if(inputStream.atEnd()) {
            continue;
        }

        QString firstLine = inputStream.readLine();
        double firstTimestamp = firstLine.section(" ", 0, 0).toDouble();

        firstLine = QString("0.0 %1 %2").arg(logNum).arg(firstLine.section(" ", 1));
        outFileCoal << firstLine.toLocal8Bit().data() << '\n';

        latestTimestamp += LOGFILE_COALESCE_SEPARATION;
        double timestampOffset = latestTimestamp;

        while(!inputStream.atEnd()) {
          QString line = inputStream.readLine();
          double timestamp = line.section(" ", 0, 0).toDouble();
          latestTimestamp  = timestamp - firstTimestamp + timestampOffset;
          line = QString("%1 %2 %3").arg(latestTimestamp).arg(logNum).arg(line.section(" ", 1));
          outFileCoal << line.toLocal8Bit().data() << '\n';
        }
      }
    }
    emit progress(100.0);
  } else {
    emit error("Problem opening output file.");
    return false;
  }

  outFileCoal.close();
  return true;
}

bool AppData::writeAxis() {
  this->outFile << "xtime [s]";

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

    for(int i = 0; i < msg.sigs.size(); i++) {
      Signal sig = msg.sigs[i];
      this->outFile << "  " << sig.title.toStdString() << " [" << sig.units.toStdString() << "]";
      vec_msg.push_back(0.0);
    }

    latestValues.push_back(vec_msg);
    messageIndices[msg.id] = indexCounter;
    indexCounter++;
  }

  return true;
}

void AppData::writeLine() {
  this->outFile << endl;

  for(unsigned int i = 0; i < latestValues.size(); i++) {
    for(unsigned int j = 0; j < latestValues[i].size(); j++) {
      if (i == 0) {
        this->outFile << latestTimestamp.toStdString();
      } else {
        this->outFile << " ";
      }

      this->outFile << latestValues[i][j];
    }
  }
}

void AppData::processBuffer(unsigned char * buffer, int length) {
  AppConfig config;
  map<unsigned short, Message> messages = config.getMessages();

  emit progress(0);

  int iter = 0;
  int progressCounter = 0;
  static int badMsgCounter = 0;
  static int badTimeCounter = 0;
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

      int j = 0;
      vector<double> values;
      bool badChnFound = false;
      for(int i = 0; i < msg.sigs.size(); i++) {
        Signal sig = msg.sigs[i];

        double value;
        if(sig.isSigned) {
          signed int data = sig.isBigEndian ?
            buffer[iter] << 8 | buffer[iter + 1] : buffer[iter + 1] << 8 | buffer[iter];
          value = (double) data;
        } else {
          unsigned int data = sig.isBigEndian ?
            buffer[iter] << 8 | buffer[iter + 1] : buffer[iter + 1] << 8 | buffer[iter];
          value = (double) data;
        }

        values.push_back((value - sig.offset) * sig.scalar);

        // Check to see if the calculated value is out of range.
        if(values[j] < sig.min || values[j] > sig.max) {
          badChnFound = true;
        }

        j++;
        iter += 2;
      }

      if(badChnFound) {
        // Skip this message and don't write to the file.
        iter += 4;
        continue;
      }

      double upper = buffer[iter + 3] << 8 | buffer[iter + 2];
      double lower = ((double) (buffer[iter + 1] << 8 | buffer[iter])) / 0x8000;
      double timestamp = upper + lower - 1.0;
      iter += 4;

      /**
       * We were experiencing problems with corrupt messages (or messages not understood by the
       * translator interpreted as being corrupt) screwing up the natural, increasing order of
       * timestamps in the logfile. This would cause the conversion to darab format to fail. This
       * if statement checks to make sure the newly calculated timestamp within a small range of
       * time in either direction.
       */
      if(latestValues[0][0] == 0.0 || abs(timestamp - latestValues[0][0]) <= 1.0) {
        latestValues[0][0] = timestamp;
        for(int i = 0; i < j; i++) {
          latestValues[messageIndices[msg.id]][i] = values[i];
        }
        writeLine();
      } else {
        if(++badTimeCounter < 6) {
          emit error(QString("Invalid timestamp: %1. Previous: %2.")
              .arg(timestamp).arg(latestValues[0][0]));
        } else if(badTimeCounter == 7) {
          emit error(QString("Invalid timestamp maxed out."));
        }
      }
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
  QStringList sections = line.split(" ", QString::SkipEmptyParts);

  // Check for invalid lines (non data lines)
  if ((sections.size() == 2 && sections[1].compare("Trigger") == 0) ||
      sections.size() < 6 || sections[1].compare("1") || sections[3].compare("Rx") ||
      sections[2].endsWith('x')) {
    return;
  }

  bool successful = true;
  uint16_t msgId;
  msgId = sections[2].toUInt(&successful, 10);
  if (!successful) {
    //TODO: emit error(QString("Invalid msgId: 0x%1").arg(sections[2]));
    return;
  }

  Message msg = messages[msgId];
  if (!msg.valid()) {
    //TODO: emit error(QString("Invalid msgId: 0x%1").arg(msgId, 0, 16));
    return;
  }

  uint8_t dlc;
  dlc = sections[5].toUInt(&successful, 10);
  if (!successful) {
    emit error(QString("Invalid dlc"));
    return;
  }

  uint8_t dataBytes[8];
  for (int i = 0; i < 8; i++) {
    dataBytes[i] = (dlc >= i + 1) ? sections[6 + i].toUInt(&successful, 10) : 0;
  }
  uint64_t data = dataBytes[0] |
    ((uint64_t) dataBytes[1]) << 8 |
    ((uint64_t) dataBytes[2]) << 16 |
    ((uint64_t) dataBytes[3]) << 24 |
    ((uint64_t) dataBytes[4]) << 32 |
    ((uint64_t) dataBytes[5]) << 40 |
    ((uint64_t) dataBytes[6]) << 48 |
    ((uint64_t) dataBytes[7]) << 56;

  if (!successful) {
    emit error("Invalid signal data.");
    return;
  }

  int j = 0;
  for (int i = 0; i < msg.sigs.size(); i++) {
    Signal sig = msg.sigs[i];

    double value;

    uint8_t startBit = sig.isBigEndian ? sig.startBit - 7 : sig.startBit;
    uint64_t sigData = (data >> startBit) & ((uint64_t) (pow(2, sig.bitLen) - 1));

    if (sig.bitLen <= 8) {
      value = sig.isSigned ? ((int8_t) sigData) : ((uint8_t) sigData);
    } else if (sig.bitLen > 8 && sig.bitLen <= 16) {
      if (!sig.isBigEndian) {
        value = sig.isSigned ? ((int16_t) sigData) : ((uint16_t) sigData);
      } else {
        if (sig.isSigned) {
          uint8_t* sigDataArray = (uint8_t*) &sigData;
          value = (int16_t) (sigDataArray[0] << 8 | sigDataArray[1]);
        } else {
          uint8_t* sigDataArray = (uint8_t*) &sigData;
          value = (uint16_t) (sigDataArray[0] << 8 | sigDataArray[1]);
        }
      }
    } else if (sig.bitLen > 16 && sig.bitLen <= 32) {
      if (!sig.isBigEndian) {
        value = sig.isSigned ? ((int32_t) sigData) : ((uint32_t) sigData);
      } else {
        if (sig.isSigned) {
          uint8_t* sigDataArray = (uint8_t*) &sigData;
          value = (int32_t) (sigDataArray[0] << 24 | sigDataArray[1] << 16 |
              sigDataArray[2] << 8 | sigDataArray[3]);
        } else {
          uint8_t* sigDataArray = (uint8_t*) &sigData;
          value = (uint32_t) (sigDataArray[0] << 24 | sigDataArray[1] << 16 |
              sigDataArray[2] << 8 | sigDataArray[3]);
        }
      }
    } else if (sig.bitLen > 32) {
      if (!sig.isBigEndian) {
        value = sig.isSigned ? ((int64_t) sigData) : ((uint64_t) sigData);
      } else {
        if (sig.isSigned) {
          uint8_t* sigDataArray = (uint8_t*) &sigData;
          value = (int64_t) (((int64_t) sigDataArray[0]) << 56 |
              ((int64_t) sigDataArray[1]) << 48 | ((int64_t) sigDataArray[2]) << 40 |
              ((int64_t) sigDataArray[3]) << 32 | ((int64_t) sigDataArray[4]) << 24 |
              ((int64_t) sigDataArray[5]) << 16 | ((int64_t) sigDataArray[6]) << 8 |
              ((int64_t) sigDataArray[7]));
        } else {
          uint8_t* sigDataArray = (uint8_t*) &sigData;
          value = (uint64_t) (((uint64_t) sigDataArray[0]) << 56 |
              ((uint64_t) sigDataArray[1]) << 48 | ((uint64_t) sigDataArray[2]) << 40 |
              ((uint64_t) sigDataArray[3]) << 32 | ((uint64_t) sigDataArray[4]) << 24 |
              ((uint64_t) sigDataArray[5]) << 16 | ((uint64_t) sigDataArray[6]) << 8 |
              ((uint64_t) sigDataArray[7]));
        }
      }
    }

    latestValues[messageIndices[msg.id]][j] = (value * sig.scalar) + sig.offset;
    j++;
  }

  latestTimestamp = sections[0];

  writeLine();
}
