/**
 * @file display.cpp
 * Implementation of the AppDisplay class.
 *
 * @author Andrew Mass
 * @date Created: 2014-06-24
 * @date Modified: 2016-03-20
 */
#include "display.h"

AppDisplay::AppDisplay() : QWidget() {
  config = new AppConfig();
  data = new AppData();
  computeThread = new ComputeThread();
  coalesceComputeThread = new CoalesceComputeThread();

  layout = new QVBoxLayout();

  this->successful = false;
  this->resize(WIDTH, HEIGHT);
  this->setWindowTitle("Illini Motorsports CAN Translator - 2015-2016");
  this->setLayout(layout);

  connect(data, SIGNAL(error(QString)), this, SLOT(handleError(QString)));
  connect(data, SIGNAL(progress(int)), this, SLOT(updateProgress(int)));
  connect(config, SIGNAL(error(QString)), this, SLOT(handleError(QString)));

  computeThread->data = data;
  coalesceComputeThread->data = data;

  layout_headers = new QVBoxLayout();
  layout_reads = new QHBoxLayout();
  layout_main = new QHBoxLayout();
  layout_config = new QVBoxLayout();
  layout_progress = new QVBoxLayout();

  layout->addLayout(layout_headers);

  QFont font_header("Helvetica", 20, QFont::Bold);
  QFont font_subheader("Helvetica", 15);
  QFont font_message("Helvetica", 15, QFont::Black);
  QFont font_signal("Helvetica", 9);

  lbl_header = new QLabel();
  lbl_header->setText("Illini Motorsports CAN Translator - 2015-2016");
  lbl_header->setFont(font_header);
  lbl_header->setStyleSheet("QLabel { color: black; }");
  lbl_header->setAlignment(Qt::AlignCenter);
  layout_headers->addWidget(lbl_header, 1);

  lbl_subheader = new QLabel();
  lbl_subheader->setText("Created By: Andrew Mass");
  lbl_subheader->setFont(font_subheader);
  lbl_subheader->setAlignment(Qt::AlignCenter);
  layout_headers->addWidget(lbl_subheader, 1);

  lbl_keymaps = new QLabel();
  lbl_keymaps->setText("[c] Convert Custom File     [v] Convert Vector File     [s] Coalesce Converted Logfiles     [q] Quit");
  lbl_keymaps->setFont(font_subheader);
  lbl_keymaps->setAlignment(Qt::AlignCenter);
  layout_headers->addWidget(lbl_keymaps, 1);

  btn_read_custom = new QPushButton();
  btn_read_custom->setText("Select Custom File to Convert");
  layout_reads->addWidget(btn_read_custom, 1);

  btn_read_vector = new QPushButton();
  btn_read_vector->setText("Select Vector File to Convert");
  layout_reads->addWidget(btn_read_vector, 1);

  btn_coalesce = new QPushButton();
  btn_coalesce->setText("Coalesce Converted Logfiles");
  layout_reads->addWidget(btn_coalesce, 1);

  layout->addLayout(layout_reads);

  // Configure config area (left side)
  area_config = new QScrollArea();
  area_config_helper = new QWidget();
  area_config_helper->setLayout(layout_config);
  area_config->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
  area_config->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  area_config->setWidgetResizable(true);
  area_config->setWidget(area_config_helper);

  // Configure progress area (right side)
  bar_convert = new QProgressBar();
  layout_progress->addWidget(bar_convert, 1);

  // Add config and progress areas to main layout
  layout_main->addWidget(area_config);
  layout_main->addLayout(layout_progress);
  layout->addLayout(layout_main);

  // Parse CAN spec from config file
  map<uint16_t, Message> messages = config->getMessages();
  if (messages.size() > 0) {
    this->successful = true;
  }

  // Add message defintions to config area
  typedef map<uint16_t, Message>::iterator it_msg;
  for(it_msg msgIt = messages.begin(); msgIt != messages.end(); msgIt++) {
    Message msg = msgIt->second;

    QGroupBox* messageGroup = new QGroupBox(msg.toString());
    QVBoxLayout* signalsLayout = new QVBoxLayout();
    messageGroup->setLayout(signalsLayout);

    for (Signal sig: msg.sigs) {
      signalsLayout->addWidget(new QLabel(sig.toString()));
    }

    layout_config->addWidget(messageGroup);
  }

  connect(computeThread, SIGNAL(finish(bool)), this, SLOT(convertFinish(bool)));
  connect(computeThread, SIGNAL(addFileProgress(QString)), this, SLOT(addFileProgress(QString)));
  connect(coalesceComputeThread, SIGNAL(finish(bool)), this, SLOT(coalesceFinish(bool)));

  connect(btn_read_custom, SIGNAL(clicked()), this, SLOT(readDataCustom()));
  connect(btn_read_vector, SIGNAL(clicked()), this, SLOT(readDataVector()));
  connect(btn_coalesce, SIGNAL(clicked()), this, SLOT(coalesceLogfiles()));
}

void AppDisplay::addFileProgress(QString filename) {
  layout_progress->addWidget(new QLabel(filename));
}

void AppDisplay::readDataCustom() {
  readData(false);
}

void AppDisplay::readDataVector() {
  readData(true);
}

void AppDisplay::coalesceLogfiles() {
  btn_read_custom->setEnabled(false);
  btn_read_vector->setEnabled(false);
  btn_coalesce->setEnabled(false);

  QFileDialog dialog(this);
  dialog.setDirectory(".");
  dialog.setNameFilter("*.out.txt");
  dialog.setFileMode(QFileDialog::ExistingFiles);
  if(dialog.exec()) {
    coalesceComputeThread->filenames = dialog.selectedFiles();
    coalesceComputeThread->start();
  } else {
    QMessageBox::critical(this, "File Dialog Error",
        "A team of highly trained monkeys has been dispatched to help you.");
    coalesceFinish(false);
  }
}

void AppDisplay::readData(bool isVectorFile) {
  btn_read_custom->setEnabled(false);
  btn_read_vector->setEnabled(false);
  btn_coalesce->setEnabled(false);

  QFileDialog dialog(this);
  dialog.setDirectory(".");
  dialog.setNameFilter(isVectorFile ? "*.asc *.ASC" : "*.txt *.TXT");
  dialog.setFileMode(QFileDialog::ExistingFiles);
  if(dialog.exec()) {
    computeThread->filenames = dialog.selectedFiles();

    computeThread->isVectorFile = isVectorFile;
    computeThread->start();
  } else {
    QMessageBox::critical(this, "File Dialog Error",
        "A team of highly trained monkeys has been dispatched to help you.");
    convertFinish(false);
  }
}

void AppDisplay::convertFinish(bool success) {
  if(success) {
    if(computeThread->filenames.size() == 1) {
      QString filename = data->filename;
      QMessageBox::information(this, "Conversion Completed!",
          QString("Output File: %1").arg(
            filename.replace(".txt", ".out.txt", Qt::CaseInsensitive)
                    .replace(".asc", ".out.txt", Qt::CaseInsensitive)));
    } else {
      QMessageBox::information(this, "Convesion Completed!",
          "Output files are stored in the same directory as the input files.");
    }
  }

  btn_read_custom->setEnabled(true);
  btn_read_vector->setEnabled(true);
  btn_coalesce->setEnabled(true);
}

void AppDisplay::coalesceFinish(bool success) {
  if(success) {
    QMessageBox::information(this, "Coalesce Completed!",
        "Output file is stored in the same directory as the input files.");
  }

  btn_read_custom->setEnabled(true);
  btn_read_vector->setEnabled(true);
  btn_coalesce->setEnabled(true);
}

void AppDisplay::handleError(QString error) {
  QMessageBox::critical(this, "Critical Error", error);
}

void AppDisplay::updateProgress(int progress) {
  bar_convert->setValue(progress);
}

void AppDisplay::keyPressEvent(QKeyEvent* e) {
  // Opens file conversion dialog for custom data files.
  if(e->text() == "c") {
    btn_read_custom->click();
  }

  // Opens file conversion dialog for vector data files.
  if(e->text() == "v") {
    btn_read_vector->click();
  }

  // Opens converted logfile dialog.
  if(e->text() == "s") {
    btn_coalesce->click();
  }

  // Quits the application.
  if(e->text() == "q") {
    QApplication::quit();
  }
}
