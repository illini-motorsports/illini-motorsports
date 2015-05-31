/**
 * @file config.cpp
 * Implementation of the AppConfig class.
 *
 * @author Andrew Mass
 * @date Created: 2014-06-24
 * @date Modified: 2015-05-31
 */
#include "config.h"

map<unsigned short, Message> AppConfig::getMessages() {
  QVector<QString> lines = readFile();
  map<unsigned short, Message> messages;

  for(int i = 0; i < lines.size(); i++) {
    Message msg = getMessage(lines[i]);
    if(msg.valid()) {
      messages[msg.id] = msg;
    } else {
      map<unsigned short, Message> empty;
      return empty;
    }
  }

  return messages;
}

QVector<QString> AppConfig::readFile() {
  QVector<QString> lines;

  QFile configFile(QCoreApplication::applicationDirPath().append("/config.txt"));
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
  if(sections.size() < 2) {
    emit error("Invalid message definition.");
    return Message();
  }

  QStringList msgDef = sections[0].split(",", QString::SkipEmptyParts);
  if(msgDef.size() != 3) {
    emit error("Invalid message definition.");
    return Message();
  }

  Message msg;

  bool successful = true;
  msg.id = msgDef[0].toUInt(&successful, 16);
  if(!successful) {
    emit error(QString("Invalid message ID"));
    return Message();
  }

  msg.dlc = msgDef[1].toUInt(&successful);
  if(!successful) {
    emit error(QString("Invalid DLC for message with ID: %1").arg(QString::number(msg.id, 16)));
    return Message();
  }

  if(QString::compare(msgDef[2], "false") == 0) {
    msg.isBigEndian = false;
  } else if(QString::compare(msgDef[2], "true") == 0) {
    msg.isBigEndian = true;
  } else {
    emit error(QString("Boolean parse failed for message with ID: %1").arg(QString::number(msg.id, 16)));
    return Message();
  }

  for(int i = 1; i < sections.size(); i++) {
    Channel chn = getChannel(i - 1, sections[i]);
    if(!chn.valid()) {
      return Message();
    }
    msg.channels.push_back(chn);
  }

  //TODO: Add support for GPS message (0203).
  if(msg.channels.size() != msg.dlc / 2) {
    emit error(QString("Invalid number of channels for message with ID: %1").arg(QString::number(msg.id, 16)));
    return Message();
  }

  return msg;
}

Channel AppConfig::getChannel(int pos, QString section) {
  QStringList chnDef = section.split(",", QString::SkipEmptyParts);

  if(chnDef.size() != 7) {
    emit error("Invalid channel definition.");
    return Channel();
  }

  Channel chn;
  chn.pos = pos;

  chn.title = chnDef[0];
  if(chn.title.length() > 7) {
    emit error(QString("Channel title length too long for channel with title: %1").arg(chn.title));
    return Channel();
  }

  if(QString::compare(chnDef[1], "false") == 0) {
    chn.isSigned = false;
  } else if(QString::compare(chnDef[1], "true") == 0) {
    chn.isSigned = true;
  } else {
    emit error(QString("Boolean parse failed for channel with title: %1").arg(chn.title));
    return Channel();
  }

  bool successful = true;
  chn.scalar = chnDef[2].toDouble(&successful);
  if(!successful) {
    emit error(QString("Invalid scalar for channel with title: %1").arg(chn.title));
    return Channel();
  }

  chn.offset = chnDef[3].toDouble(&successful);
  if(!successful) {
    emit error(QString("Invalid offset for channel with title: %1").arg(chn.title));
    return Channel();
  }

  chn.units = chnDef[4];
  if(chn.units.length() > 4) {
    emit error(QString("Units length too long for channel with title: %1").arg(chn.title));
    return Channel();
  }

  chn.min = chnDef[5].toDouble(&successful);
  if(!successful) {
    emit error(QString("Invalid min for channel with title: %1").arg(chn.title));
    return Channel();
  }

  chn.max = chnDef[6].toDouble(&successful);
  if(!successful) {
    emit error(QString("Invalid max for channel with title: %1").arg(chn.title));
    return Channel();
  }

  return chn;
}
