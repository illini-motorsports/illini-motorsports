/**
 * Config
 *
 * @author Andrew Mass
 * @date Created: 2014-06-24
 * @date Modified: 2016-03-13
 */
#include "config.h"

/**
 * map<uint16_t, Message> AppConfig::getMessages()
 *
 * Reads the config.dbc file in the current directory and extracts a CAN spec
 * from it. The function will then build a map from CAN ID to a Message struct
 * which will encompass the entire spec.
 *
 * @returns The CAN spec in map<uint16_t, Message> form. If an error was
 *     encountered processing any of the messages, this function will return
 *     an empty map.
 */
map<uint16_t, Message> AppConfig::getMessages() {
  QVector< QVector<QString> > messageBlocks = readFile();

  map<uint16_t, Message> messages;

  for (QVector<QString> messageBlock: messageBlocks) {
    Message msg = getMessage(messageBlock);
    if (msg.valid()) {
      messages[msg.id] = msg;
    } else {
      map<uint16_t, Message> empty;
      return empty;
    }
  }

  return messages;
}

/**
 * QVector< QVector<QString> > AppConfig::readFile()
 *
 * Opens an input stream from the config file and parses it into a vector of
 * blocks containing all the lines for one message. Lines that are blank,
 * comments, or otherwise invalid are ignored.
 *
 * @returns A vector of vectors of strings containing CAN message definitions
 */
QVector< QVector<QString> > AppConfig::readFile() {
  QVector< QVector<QString> > messageBlocks;
  QVector<QString> lines;

  // Read all valid lines of the config file into a QString vector
  QFile configFile(QCoreApplication::applicationDirPath().append("/config.dbc"));
  if (configFile.open(QIODevice::ReadOnly)) {
    QTextStream inStream(&configFile);

    while (!inStream.atEnd()) {
      QString line = inStream.readLine().simplified();

      if (line.isEmpty() ||
          !(line.startsWith("BO_ ") || line.startsWith("SG_ "))) {
        continue;
      }

      lines.append(line);
    }

    configFile.close();
  } else {
    emit error("File was not opened.");
  }

  // Organize lines read from config file into a block of lines for each message
  for (int32_t i = 0; i < lines.size();) {
    QVector<QString> block;
    block.append(lines[i++]);

    while (i < lines.size() && lines[i].startsWith("SG_ ")) {
      block.append(lines[i++]);
    }

    messageBlocks.append(block);
  }

  return messageBlocks;
}

/**
 * Message AppConfig::getMessage(QVector<QString> messageBlock)
 *
 * Takes a block of strings corresponding to one CAN message definition as
 * input and returns a Message struct with all applicable parameters set. This
 * function will throw errors if the input data is missing or malformed.
 *
 * @param messageBlock - The block of strings corresponding to one CAN message
 *     defintion
 * @returns A Message struct with all applicable parameters set, or an empty
 *     Message struct if an error occurred.
 */
Message AppConfig::getMessage(QVector<QString> messageBlock) {
  QString msgDef = messageBlock[0];
  QStringList sections = msgDef.split(" ", QString::SkipEmptyParts);
  if (sections.size() != 5) {
    emit error("Invalid message definition.");
    return Message();
  }

  Message msg;

  bool successful = true;
  msg.id = sections[1].toUInt(&successful);
  if (!successful) {
    emit error(QString("Invalid message ID"));
    return Message();
  }

  msg.dlc = sections[3].toUInt(&successful);
  if (!successful) {
    emit error(QString("Invalid DLC for message with ID: %1").arg(QString::number(msg.id, 16)));
    return Message();
  }

  for (int i = 1; i < messageBlock.size(); i++) {
    Signal sig = getSignal(messageBlock[i]);
    if (!sig.valid()) {
      return Message();
    }
    msg.sigs.push_back(sig);
  }

  return msg;
}

/**
 * Signal AppConfig::getSignal(QString signalDef)
 *
 * This function takes a single line representing a signal definition and
 * parses it into a Signal struct. It will throw errors if input data is
 * missing or malformed.
 *
 * @param signalDef - A string containing all info for one signal definition
 * @returns A Signal struct with all applicable parameters set
 */
Signal AppConfig::getSignal(QString signalDef) {
  QStringList sections = signalDef.split(" ", QString::SkipEmptyParts);

  if (sections.size() != 8) {
    emit error("Invalid signal definition.");
    return Signal();
  }

  Signal sig;

  sig.title = sections[1];
  /* TODO: Figure out what to do for Darab Title limit
  if (sig.title.length() > 7) {
    emit error(QString("Signal title length too long for signal with title: %1").arg(sig.title));
    return Signal();
  }
  */

  QString signedInfo = sections[3].split("@")[1].right(1);
  if (QString::compare(signedInfo, "+") == 0) {
    sig.isSigned = false;
  } else if (QString::compare(signedInfo, "-") == 0) {
    sig.isSigned = true;
  } else {
    emit error(QString("Signedness parse failed for signal with title: %1").arg(sig.title));
    return Signal();
  }

  QString endianInfo = sections[3].split("@")[1].left(1);
  if (QString::compare(endianInfo, "1") == 0) {
    sig.isBigEndian = false;
  } else if (QString::compare(endianInfo, "0") == 0) {
    sig.isBigEndian = true;
  } else {
    emit error(QString("Endianness parse failed for signal with title: %1").arg(sig.title));
    return Signal();
  }

  bool successful = true;

  QString scalarInfo = sections[4].split(",")[0].remove(0, 1);
  sig.scalar = scalarInfo.toDouble(&successful);
  if (!successful) {
    emit error(QString("Invalid scalar for signal with title: %1").arg(sig.title));
    return Signal();
  }

  QString offsetInfo = sections[4].split(",")[1];
  offsetInfo = offsetInfo.left(offsetInfo.length() - 1);
  sig.offset = offsetInfo.toDouble(&successful);
  if (!successful) {
    emit error(QString("Invalid offset for signal with title: %1").arg(sig.title));
    return Signal();
  }

  sig.units = sections[6].replace("\"", "");

  QString startBitInfo = sections[3].split("@")[0].split("|")[0];
  sig.startBit = startBitInfo.toUShort(&successful);
  if (!successful) {
    emit error(QString("Invalid start bit for signal with title: %1").arg(sig.title));
    return Signal();
  }

  QString bitLenInfo = sections[3].split("@")[0].split("|")[1];
  sig.bitLen = bitLenInfo.toUShort(&successful);
  if (!successful) {
    emit error(QString("Invalid bit length for signal with title: %1").arg(sig.title));
    return Signal();
  }

  QString minInfo = sections[5].split("|")[0].replace("[", "");
  sig.min = minInfo.toDouble(&successful);
  if (!successful) {
    emit error(QString("Invalid min for signal with title: %1").arg(sig.title));
    return Signal();
  }

  QString maxInfo = sections[5].split("|")[1].replace("]", "");
  sig.max = maxInfo.toDouble(&successful);
  if (!successful) {
    emit error(QString("Invalid max for signal with title: %1").arg(sig.title));
    return Signal();
  }

  return sig;
}
