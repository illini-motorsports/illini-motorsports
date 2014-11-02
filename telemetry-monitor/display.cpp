/**
 * @file display.cpp
 * Implementation of the AppDisplay class.
 *
 * @author Andrew Mass
 * @date Created: 2014-03-03
 * @date Modified: 2014-05-19
 */
#include "display.h"

AppDisplay::AppDisplay(QWidget* parent) : QWidget(parent) {
  this->resize(WIDTH, HEIGHT);

  // Move the window to the center of the screen.
  QDesktopWidget *desktop = QApplication::desktop();
  this->move((desktop->width() - WIDTH) / 2, (desktop->height() - HEIGHT) / 2);

  this->setWindowTitle("Illini Motorsports Telemetry Monitor - 2014");

  // Initializes the array of CAN messages and the data associated with each.
  this->messages = init_map();

  // Initiate the QSerialPort that we will use to communicate with the car.
  this->serialPort = new QSerialPort(this);

  layout = new QVBoxLayout();
  this->setLayout(layout);

  QFont font_header("Helvetica", 20, QFont::Bold);
  QFont font_subheader("Helvetica", 15);
  QFont font_metric_title("Helvetica", 25);
  QFont font_metric("Helvetica", 55);
  QFont font_bar_data("Helvetica", 35);
  QFont font_stats_title("Helvetica", 20);
  QFont font_stats_data("Helvetica", 35);

  layout_above = new QHBoxLayout();
  layout->addLayout(layout_above, 3);

  layout_stats_left = new QVBoxLayout();
  layout_above->addLayout(layout_stats_left, 2);

  layout_headers = new QVBoxLayout();
  layout_above->addLayout(layout_headers, 3);

  layout_stats_right = new QVBoxLayout();
  layout_above->addLayout(layout_stats_right, 2);

  lbl_stats_left_title = new QLabel("Message Counter");
  lbl_stats_left_title->setFont(font_stats_title);
  lbl_stats_left_title->setAlignment(Qt::AlignCenter);
  layout_stats_left->addWidget(lbl_stats_left_title, 3);

  lbl_stats_left_data = new QLabel("0");
  lbl_stats_left_data->setFont(font_stats_data);
  lbl_stats_left_data->setAlignment(Qt::AlignCenter);
  layout_stats_left->addWidget(lbl_stats_left_data, 7);

  lbl_stats_right_title = new QLabel("Dropped Messages");
  lbl_stats_right_title->setFont(font_stats_title);
  lbl_stats_right_title->setStyleSheet("QLabel { background-color: yellow; }");
  lbl_stats_right_title->setAlignment(Qt::AlignCenter);
  layout_stats_right->addWidget(lbl_stats_right_title, 3);

  lbl_stats_right_data = new QLabel("0");
  lbl_stats_right_data->setFont(font_stats_data);
  lbl_stats_right_data->setAlignment(Qt::AlignCenter);
  layout_stats_right->addWidget(lbl_stats_right_data, 7);

  lbl_header = new QLabel("Illini Motorsports Telemetry Monitor");
  lbl_header->setFont(font_header);
  lbl_header->setStyleSheet("QLabel { color: black; }");
  lbl_header->setAlignment(Qt::AlignCenter);
  layout_headers->addWidget(lbl_header, 1);

  lbl_subheader = new QLabel("Created By: Andrew Mass and George Schwieters");
  lbl_subheader->setFont(font_subheader);
  lbl_subheader->setAlignment(Qt::AlignCenter);
  layout_headers->addWidget(lbl_subheader, 1);

  lbl_ports = new QLabel("List serial [p]orts");
  lbl_ports->setFont(font_subheader);
  lbl_ports->setAlignment(Qt::AlignCenter);
  layout_headers->addWidget(lbl_ports, 1);

  btn_quit = new QPushButton("[Q]uit");
  btn_quit->setToolTip("Quit the program");
  layout_headers->addWidget(btn_quit, 1);

  lbl_connected = new QLabel();
  lbl_connected->setAlignment(Qt::AlignCenter);
  lbl_connected->setFont(font_subheader);
  layout->addWidget(lbl_connected, 1);

  layout_sub = new QHBoxLayout();
  layout->addLayout(layout_sub, 10);

  layout_left = new QVBoxLayout();
  layout_sub->addLayout(layout_left);

  layout_right = new QVBoxLayout();
  layout_sub->addLayout(layout_right);

  for(int i = 0; i < NUM_METRICS; i++) {
    layouts_left[i] = new QHBoxLayout();
    layout_left->addLayout(layouts_left[i]);

    layouts_right[i] = new QHBoxLayout();
    layout_right->addLayout(layouts_right[i]);

    lbls_left_title[i] = new QLabel();
    lbls_left_title[i]->setText(messages.at(metric_ids[i]).title);
    lbls_left_title[i]->setFont(font_metric_title);
    layouts_left[i]->addWidget(lbls_left_title[i], 1);

    lbls_right_title[i] = new QLabel();
    lbls_right_title[i]->setText(messages.at(metric_ids[NUM_METRICS + i]).title);
    lbls_right_title[i]->setFont(font_metric_title);
    layouts_right[i]->addWidget(lbls_right_title[i], 1);

    lbls_left_data[i] = new QLabel();
    lbls_left_data[i]->setText("- " + messages.at(metric_ids[i]).units);
    lbls_left_data[i]->setFont(font_metric);
    lbls_left_data[i]->setStyleSheet("QLabel { background-color: yellow; }");
    lbls_left_data[i]->setAlignment(Qt::AlignCenter);
    layouts_left[i]->addWidget(lbls_left_data[i], 2);

    lbls_right_data[i] = new QLabel();
    lbls_right_data[i]->setText("- " + messages.at(metric_ids[NUM_METRICS + i]).units);
    lbls_right_data[i]->setFont(font_metric);
    lbls_right_data[i]->setStyleSheet("QLabel { background-color: yellow; }");
    lbls_right_data[i]->setAlignment(Qt::AlignCenter);
    layouts_right[i]->addWidget(lbls_right_data[i], 2);
  }

  for(int i = 0; i < NUM_BARS; i++) {
    layouts_bar[i] = new QHBoxLayout();
    layout->addLayout(layouts_bar[i], 2);

    lbls_bar_title[i] = new QLabel();
    lbls_bar_title[i]->setText(messages.at(bar_ids[i]).title);
    lbls_bar_title[i]->setFont(font_metric_title);
    layouts_bar[i]->addWidget(lbls_bar_title[i], 1);

    lbls_bar_data[i] = new QLabel();
    lbls_bar_data[i]->setText("- " + messages.at(bar_ids[i]).units);
    lbls_bar_data[i]->setAlignment(Qt::AlignCenter);
    lbls_bar_data[i]->setFont(font_bar_data);
    lbls_bar_data[i]->setStyleSheet("QLabel { background-color: yellow; }");
    layouts_bar[i]->addWidget(lbls_bar_data[i], 1);

    bars_bar[i] = new QProgressBar();
    bars_bar[i]->setTextVisible(false);
    bars_bar[i]->setStyleSheet(bars_bar[i]->property("defaultStyleSheet").toString() + "QProgressBar::chunk { background-color: green; }");
    bars_bar[i]->setMinimum(messages.at(bar_ids[i]).minBar);
    bars_bar[i]->setMaximum(messages.at(bar_ids[i]).maxBar);
    layouts_bar[i]->addWidget(bars_bar[i], 4);
  }

  /*
   * Connects the error signal from the serial port to the handleError()
   * function in the display.
   */
  connect(serialPort, SIGNAL(error(QSerialPort::SerialPortError)), this,
      SLOT(handleError(QSerialPort::SerialPortError)));

  /*
   * Connects the readyRead() signal from the serial port to the readData()
   * function in the display. This should tell the AppData class when there
   * is data that is ready to be processed on the serial port buffer.
   */
  connect(serialPort, SIGNAL(readyRead()), this, SLOT(readData()));

  /* Connects the clicked() signal from the quit button to the onQuit()
   * function in the display. Closes the app when the quit button is pressed.
   */
  connect(btn_quit, SIGNAL(clicked()), this, SLOT(onQuit()));

  this->setConnected(false);

  data.openSerialPort(*serialPort, *this);
}

void AppDisplay::updateData(unsigned char msgId, double data) {
  for(int i = 0; i < NUM_METRICS * 2; i++) {
    if(metric_ids[i] == msgId) {
      if(i < NUM_METRICS) {
        lbls_left_data[i]->setText(QString::number(data) + " " +
            messages.at(metric_ids[i]).units);

        if(data >= messages.at(metric_ids[i]).danger_high ||
            data <= messages.at(metric_ids[i]).danger_low) {
          lbls_left_data[i]->setStyleSheet("QLabel { background-color: red; color: white; }");
        } else {
          lbls_left_data[i]->setStyleSheet("QLabel { background-color: green; color: white; }");
        }
      } else {
        lbls_right_data[i - NUM_METRICS]->setText(QString::number(data) +
            " " + messages.at(metric_ids[i]).units);

        if(data >= messages.at(metric_ids[i]).danger_high ||
            data <= messages.at(metric_ids[i]).danger_low) {
          lbls_right_data[i - NUM_METRICS]->setStyleSheet("QLabel { background-color: red; color: white; }");
        } else {
          lbls_right_data[i - NUM_METRICS]->setStyleSheet("QLabel { background-color: green; color: white; }");
        }
      }
    }
  }

  for(int i = 0; i < NUM_BARS; i++) {
    if(bar_ids[i] == msgId) {
      lbls_bar_data[i]->setText(QString::number(data) + " " +
          messages.at(metric_ids[i]).units);
      bars_bar[i]->setValue(data);

      if(data >= messages.at(bar_ids[i]).danger_high ||
          data <= messages.at(bar_ids[i]).danger_low) {
        lbls_bar_data[i]->setStyleSheet("QLabel { background-color: red; color: white; }");
        bars_bar[i]->setStyleSheet(bars_bar[i]->property("defaultStyleSheet").toString() + "QProgressBar::chunk { background-color: red; }");
      } else {
        lbls_bar_data[i]->setStyleSheet("QLabel { background-color: green; color: white; }");
        bars_bar[i]->setStyleSheet(bars_bar[i]->property("defaultStyleSheet").toString() + "QProgressBar::chunk { background-color: green; }");
      }
    }
  }
}

void AppDisplay::errorMessage(bool hasError) {
    if(hasError)
        lbl_stats_right_title->setStyleSheet("QLabel { background-color: red; }");
    else
        lbl_stats_right_title->setStyleSheet("QLabel { background-color: green; }");
}

void AppDisplay::updateMessageCounter(unsigned int data) {
  lbl_stats_left_data->setText(QString::number(data));
}

void AppDisplay::updateDropCounter(unsigned int data) {
  lbl_stats_right_data->setText(QString::number(data));
}

void AppDisplay::setConnected(bool connected) {
  lbl_connected->setText(connected ? "Connected" : "Not Connected");
  lbl_connected->setStyleSheet(connected ? "QLabel { background-color: green; color: white; }" :
      "QLabel { background-color: red; color: black; }");
}

void AppDisplay::keyPressEvent(QKeyEvent* e) {

  // Displays the availiable serial ports in a label on the GUI.
  if(e->text() == "p") {
    lbl_ports->setText("");

    foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
      lbl_ports->setText(lbl_ports->text() + " " + info.portName() + " " + info.manufacturer() + " " + info.description());
    }
    lbl_ports->setText(lbl_ports->text() + " ");
  }

  // Quits the application.
  if(e->text() == "q") {
    onQuit();
  }
  if(e->text() == "g") {
      errorMessage(false);
  }
  if(e->text() == "r") {
      errorMessage(true);
  }
}

void AppDisplay::readData() {
  data.readData(*serialPort, *this);
}

void AppDisplay::handleError(QSerialPort::SerialPortError error) {
  if(error == QSerialPort::ResourceError) {
    QMessageBox::critical(this, tr("Critical Error"), serialPort->errorString());
    setConnected(false);
    data.closeSerialPort(*serialPort);
  }
}

void AppDisplay::showMessage(QString header, QString message) {
  QMessageBox::information(this, header, message);
}

void AppDisplay::onQuit() {
  setConnected(false);
  data.closeSerialPort(*serialPort);
  QApplication::quit();
}
