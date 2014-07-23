/**
 * @file display.cpp
 * Implementation of the AppDisplay class.
 *
 * @author Andrew Mass
 * @date Created: 2014-06-24
 * @date Modified: 2014-07-22
 */
#include "display.h"

AppDisplay::AppDisplay() : QWidget() {
  this->successful = false;
  this->resize(WIDTH, HEIGHT);
  this->setWindowTitle("Illini Motorsports CAN Translator - 2015");
  this->setLayout(&layout);

  connect(&data, SIGNAL(error(QString)), this, SLOT(handleError(QString)));
  connect(&data, SIGNAL(progress(int)), this, SLOT(updateProgress(int)));
  connect(&config, SIGNAL(error(QString)), this, SLOT(handleError(QString)));

  layout.addLayout(&layout_headers);

  QFont font_header("Helvetica", 20, QFont::Bold);
  QFont font_subheader("Helvetica", 15);
  QFont font_messageCount("Helvetica", 25);
  QFont font_message("Helvetica", 10, QFont::Bold);

  lbl_header.setText("Illini Motorsports CAN Translator - 2015");
  lbl_header.setFont(font_header);
  lbl_header.setStyleSheet("QLabel { color: black; }");
  lbl_header.setAlignment(Qt::AlignCenter);
  layout_headers.addWidget(&lbl_header, 1);

  lbl_subheader.setText("Created By: Andrew Mass");
  lbl_subheader.setFont(font_subheader);
  lbl_subheader.setAlignment(Qt::AlignCenter);
  layout_headers.addWidget(&lbl_subheader, 1);

  btn_read.setText("Read Data");
  layout.addWidget(&btn_read, 1);

  layout.addWidget(&bar_convert, 1);

  lbl_messageCount.setFont(font_messageCount);
  lbl_messageCount.setAlignment(Qt::AlignCenter);
  layout.addWidget(&lbl_messageCount, 1);

  lbl_messages.setFont(font_message);
  layout.addWidget(&lbl_messages, 4);

  map<unsigned short, Message> messages = config.getMessages();
  if(messages.size() > 0) {
    this->successful = true;
  }
  lbl_messageCount.setText(QString("Number of Messages: %1").arg(messages.size()));

  typedef map<unsigned short, Message>::iterator it_msg;
  for(it_msg msgIt = messages.begin(); msgIt != messages.end(); msgIt++) {
    Message msg = msgIt->second;
    QString line = "\nId: " + QString::number(msg.id, 16) + ". L: " + QString::number(msg.dlc) + ". BigEndian: " +
        (msg.isBigEndian ? "T " : "F ");

    typedef QVector<Channel>::iterator it_chn;
    for(it_chn chnIt = msg.channels.begin(); chnIt != msg.channels.end(); chnIt++) {
      Channel chn = *chnIt;
      line += " ||| Pos: " + QString::number(chn.pos) + " T: " + chn.title + " isS: " + (chn.isSigned ? "T" : "F") +
          " S: " + QString::number(chn.scalar) + " O: " + QString::number(chn.offset) + " U: " + chn.units + " ";
    }

    lbl_messages.setText(lbl_messages.text() + line);
  }

  connect(&btn_read, SIGNAL(clicked()), this, SLOT(readData()));
}

void AppDisplay::readData() {
  data.filename = QFileDialog::getOpenFileName(this, "Open File", ".", "Files (*.*)");
  if(data.writeAxis() && data.readData()) {
    QMessageBox::information(this, "Conversion Completed!", "Output File: ./out.txt");
  }
}

void AppDisplay::handleError(QString error) {
  QMessageBox::critical(this, "Critical Error", error);
}

void AppDisplay::updateProgress(int progress) {
  bar_convert.setValue(progress);
}
