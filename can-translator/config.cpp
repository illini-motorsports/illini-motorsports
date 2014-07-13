/**
 * @file config.cpp
 * Implementation of the AppConfig class.
 *
 * @author Andrew Mass
 * @date Created: 2014-06-24
 * @date Modified: 2014-07-12
 */
#include "config.h"

map<unsigned short, Message> AppConfig::getMessages() {
  QVector<QString> lines = readFile();
  map<unsigned short, Message> messages;

  for(int i = 0; i < lines.size(); i++) {
    Message msg = getMessage(lines[i]);
    if(msg.id == 0 && msg.dlc == 0 && !msg.isBigEndian) {
      map<unsigned short, Message> empty;
      return empty;
    } else {
      messages[msg.id] = msg;
    }
  }

  return messages;
}

QVector<QString> AppConfig::readFile() {
  QVector<QString> lines;

  QFile configFile("./config.txt");
  if(configFile.open(QIODevice::ReadOnly)) {
    QTextStream inStream(&configFile);

    while(!inStream.atEnd()) {
      QString line = inStream.readLine();

      if(!line.startsWith("#", Qt::CaseInsensitive) && !line.isEmpty()) {
        lines.push_back(line.simplified());
      }
    }

    configFile.close();
  } else {
    emit error("File was not opened.");
  }
  return lines;
}

Message AppConfig::getMessage(QString line) {
  QStringList sections = line.split(" ", QString::SkipEmptyParts);
  QString msgDefString = sections[0];
  QStringList msgDef = msgDefString.split(",", QString::SkipEmptyParts);

  Message msg;

  bool conv = true;
  msg.id = msgDef[0].toUInt(&conv, 16);
  if(!conv) {
    emit error(QString("Invalid message ID"));
    return Message();
  }

  msg.dlc = msgDef[1].toUInt(&conv);
  if(!conv) {
    emit error(QString("Invalid DLC for message with ID: %1").arg(msg.id));
    return Message();
  }

  if(QString::compare(msgDef[2], "false") == 0) {
    msg.isBigEndian = false;
  } else if(QString::compare(msgDef[2], "true") == 0) {
    msg.isBigEndian = true;
  } else {
    emit error(QString("Boolean parse failed for message with ID: %1").arg(msg.id));
    return Message();
  }

  for(int i = 1; i < sections.size(); i++) {
    Channel chn = getChannel(i - 1, sections[i]);
    if(chn.title.isEmpty() && chn.units.isEmpty()) {
      return Message();
    }
    msg.channels.push_back(chn);
  }

  //TODO: Add support for GPS message (0203).
  if(msg.channels.size() != msg.dlc / 2) {
    emit error(QString("Invalid number of channels for message with ID: %1").arg(msg.id));
    return Message();
  }

  return msg;
}

Channel AppConfig::getChannel(int pos, QString section) {
  QStringList chnDef = section.split(",", QString::SkipEmptyParts);

  Channel chn;
  chn.pos = pos;

  chn.title = chnDef[0];
  if(chn.title.length() > 7) {
    emit error(QString("Channel title length too long for channel with title: %1").arg(chn.title));
    Channel blank;
    return blank;
  }


  if(QString::compare(chnDef[1], "false") == 0) {
    chn.isSigned = false;
  } else if(QString::compare(chnDef[1], "true") == 0) {
    chn.isSigned = true;
  } else {
    emit error(QString("Boolean parse failed for channel with title: %1").arg(chn.title));
    Channel blank;
    return blank;
  }

  bool conv = true;
  chn.scalar = chnDef[2].toDouble(&conv);
  if(!conv) {
    emit error(QString("Invalid scalar for channel with title: %1").arg(chn.title));
    Channel blank;
    return blank;
  }

  chn.offset = chnDef[3].toDouble(&conv);
  if(!conv) {
    emit error(QString("Invalid offset for channel with title: %1").arg(chn.title));
    Channel blank;
    return blank;
  }

  chn.units = chnDef[4];
  if(chn.units.length() > 4) {
    emit error(QString("Units length too long for channel with title: %1").arg(chn.title));
    Channel blank;
    return blank;
  }

  return chn;
}
