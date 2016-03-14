/**
 * @file display.cpp
 * Implementation of the AppDisplay class.
 *
 * @author Andrew Mass
 * @date Created: 2014-06-24
 * @date Modified: 2015-06-07
 */
#include "display.h"

void ComputeThread::run() {
  for(int i = 0; i < filenames.size(); i++) {
    this->data->filename = this->filenames.at(i);
    if(!(this->data->writeAxis() && this->data->readData(this->isVectorFile))) {
      finish(false);
      return;
    }
  }
  finish(true);
}

void CoalesceComputeThread::run() {
  finish(this->data->coalesceLogfiles(this->filenames));
}

AppDisplay::AppDisplay() : QWidget() {
  this->successful = false;
  this->resize(WIDTH, HEIGHT);
  this->setWindowTitle("Illini Motorsports CAN Translator - 2015-2016");
  this->setLayout(&layout);

  connect(&data, SIGNAL(error(QString)), this, SLOT(handleError(QString)));
  connect(&data, SIGNAL(progress(int)), this, SLOT(updateProgress(int)));
  connect(&config, SIGNAL(error(QString)), this, SLOT(handleError(QString)));

  computeThread.data = &data;
  coalesceComputeThread.data = &data;

  layout.addLayout(&layout_headers);

  QFont font_header("Helvetica", 20, QFont::Bold);
  QFont font_subheader("Helvetica", 15);
  QFont font_message("Helvetica", 15, QFont::Black);
  QFont font_signal("Helvetica", 9);

  lbl_header.setText("Illini Motorsports CAN Translator - 2015");
  lbl_header.setFont(font_header);
  lbl_header.setStyleSheet("QLabel { color: black; }");
  lbl_header.setAlignment(Qt::AlignCenter);
  layout_headers.addWidget(&lbl_header, 1);

  lbl_subheader.setText("Created By: Andrew Mass");
  lbl_subheader.setFont(font_subheader);
  lbl_subheader.setAlignment(Qt::AlignCenter);
  layout_headers.addWidget(&lbl_subheader, 1);

  lbl_keymaps.setText("[c] Convert Custom File     [v] Convert Vector File     [s] Coalesce Converted Logfiles     [a] Select All     [n] Select None     [q] Quit");
  lbl_keymaps.setFont(font_subheader);
  lbl_keymaps.setAlignment(Qt::AlignCenter);
  layout_headers.addWidget(&lbl_keymaps, 1);

  btn_read_custom.setText("Select Custom File to Convert");
  layout_reads.addWidget(&btn_read_custom, 1);

  btn_read_vector.setText("Select Vector File to Convert");
  layout_reads.addWidget(&btn_read_vector, 1);

  btn_coalesce.setText("Coalesce Converted Logfiles");
  layout_reads.addWidget(&btn_coalesce, 1);

  layout.addLayout(&layout_reads);

  btn_select_all.setText("Select All Signals");
  layout_selects.addWidget(&btn_select_all, 1);

  btn_select_none.setText("Select No Signals");
  layout_selects.addWidget(&btn_select_none, 1);

  layout.addLayout(&layout_selects);

  layout.addWidget(&bar_convert, 1);

  map<unsigned short, Message> messages = config.getMessages();
  if(messages.size() > 0) {
    this->successful = true;
  }

  table.setRowCount(messages.size());
  table.setColumnCount(7);

  QStringList headers;
  headers << "ID" << "L" << "BE" << "Signal 0" << "Signal 1" << "Signal 2" << "Signal 3";
  table.setHorizontalHeaderLabels(headers);

  table.horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
  table.horizontalHeader()->setSectionResizeMode(1, QHeaderView::Fixed);
  table.horizontalHeader()->setSectionResizeMode(2, QHeaderView::Fixed);
  table.horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);
  table.horizontalHeader()->setSectionResizeMode(4, QHeaderView::Stretch);
  table.horizontalHeader()->setSectionResizeMode(5, QHeaderView::Stretch);
  table.horizontalHeader()->setSectionResizeMode(6, QHeaderView::Stretch);
  table.verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);

  int i = 0;
  typedef map<unsigned short, Message>::iterator it_msg;
  for(it_msg msgIt = messages.begin(); msgIt != messages.end(); msgIt++) {
    Message msg = msgIt->second;

    QTableWidgetItem* item_id = new QTableWidgetItem(QString::number(msg.id, 16));
    item_id->setFlags(Qt::NoItemFlags);
    table.setItem(i, 0, item_id);

    QTableWidgetItem* item_dlc = new QTableWidgetItem(QString::number(msg.dlc));
    item_dlc->setFlags(Qt::NoItemFlags);
    table.setItem(i, 1, item_dlc);

    int j = 0;
    typedef QVector<Signal>::iterator it_sig;
    for(it_sig sigIt = msg.sigs.begin(); sigIt != msg.sigs.end(); sigIt++) {
      Signal sig = *sigIt;
      QTableWidgetItem* item = new QTableWidgetItem("     " + sig.title + "<" + sig.units + ">" + " isS: " + (sig.isSigned ? "T" : "F") +
          " S: " + QString::number(sig.scalar) + " O: " + QString::number(sig.offset));
      item->setFlags(Qt::NoItemFlags);
      item->setFont(font_signal);
      table.setItem(i, 3 + j, item);

      QCheckBox* box = new QCheckBox();
      box->setFocusPolicy(Qt::NoFocus);
      box->setChecked(!(sig.title.compare("Unused") == 0 || sig.title.compare("Rsrvd") == 0));
      box->setStyleSheet("QCheckBox:hover { background-color: rgba(255, 255, 255, 0); }");
      table.setCellWidget(i, 3 + j, box);
      j++;
    }

    while(j < 4) {
      QTableWidgetItem* item = new QTableWidgetItem();
      item->setFlags(Qt::NoItemFlags);
      table.setItem(i, 3 + j, item);
      j++;
    }

    i++;
  }

  table.setFocusPolicy(Qt::NoFocus);
  table.setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
  table.setMinimumHeight(550);
  table.setMinimumWidth(1400);
  table.setFont(font_message);
  table.resizeColumnsToContents();
  table.show();
  layout.addWidget(&table, 1);

  connect(&computeThread, SIGNAL(finish(bool)), this, SLOT(convertFinish(bool)));
  connect(&coalesceComputeThread, SIGNAL(finish(bool)), this, SLOT(coalesceFinish(bool)));

  connect(&btn_select_all, SIGNAL(clicked()), this, SLOT(selectAll()));
  connect(&btn_select_none, SIGNAL(clicked()), this, SLOT(selectNone()));
  connect(&btn_read_custom, SIGNAL(clicked()), this, SLOT(readDataCustom()));
  connect(&btn_read_vector, SIGNAL(clicked()), this, SLOT(readDataVector()));
  connect(&btn_coalesce, SIGNAL(clicked()), this, SLOT(coalesceLogfiles()));
}

map<unsigned short, vector<bool> > AppDisplay::getEnabled() {
  map<unsigned short, vector<bool> > enabled;
  map<unsigned short, Message> messages = config.getMessages();

  for(int i = 0; i < table.rowCount(); i++) {
    bool conv;
    vector<bool> msgEnabled;
    Message msg = messages[table.item(i, 0)->text().toUInt(&conv, 16)];

    int j = 0;
    typedef QVector<Signal>::iterator it_sig;
    for(it_sig sigIt = msg.sigs.begin(); sigIt != msg.sigs.end(); sigIt++) {
      msgEnabled.push_back(((QCheckBox*) table.cellWidget(i, 3 + j))->isChecked());
      j++;
    }
    enabled[msg.id] = msgEnabled;
  }

  return enabled;
}

void AppDisplay::selectAll() {
  selectBoxes(true);
}

void AppDisplay::selectNone() {
  selectBoxes(false);
}

void AppDisplay::selectBoxes(bool checked) {
  map<unsigned short, Message> messages = config.getMessages();
  bool conv;
  for(int i = 0; i < table.rowCount(); i++) {
    Message msg = messages[table.item(i, 0)->text().toUInt(&conv, 16)];

    for(int j = 0; j < msg.sigs.size(); j++) {
      ((QCheckBox*) table.cellWidget(i, 3 + j))->setChecked(checked);
    }
  }
}

void AppDisplay::enableBoxes(bool enabled) {
  map<unsigned short, Message> messages = config.getMessages();
  bool conv;
  for(int i = 0; i < table.rowCount(); i++) {
    Message msg = messages[table.item(i, 0)->text().toUInt(&conv, 16)];

    for(int j = 0; j < msg.sigs.size(); j++) {
      ((QCheckBox*) table.cellWidget(i, 3 + j))->setEnabled(enabled);
    }
  }
}

void AppDisplay::readDataCustom() {
  readData(false);
}

void AppDisplay::readDataVector() {
  readData(true);
}

void AppDisplay::coalesceLogfiles() {
  btn_read_custom.setEnabled(false);
  btn_read_vector.setEnabled(false);
  btn_coalesce.setEnabled(false);
  btn_select_all.setEnabled(false);
  btn_select_none.setEnabled(false);
  enableBoxes(false);

  QFileDialog dialog(this);
  dialog.setDirectory(".");
  dialog.setNameFilter("*.out.txt");
  dialog.setFileMode(QFileDialog::ExistingFiles);
  if(dialog.exec()) {
    coalesceComputeThread.filenames = dialog.selectedFiles();
    coalesceComputeThread.start();
  } else {
    QMessageBox::critical(this, "File Dialog Error",
        "A team of highly trained monkeys has been dispatched to help you.");
    coalesceFinish(false);
  }
}

void AppDisplay::readData(bool isVectorFile) {
  btn_read_custom.setEnabled(false);
  btn_read_vector.setEnabled(false);
  btn_coalesce.setEnabled(false);
  btn_select_all.setEnabled(false);
  btn_select_none.setEnabled(false);
  enableBoxes(false);

  QFileDialog dialog(this);
  dialog.setDirectory(".");
  dialog.setNameFilter("*.txt *.TXT");
  dialog.setFileMode(QFileDialog::ExistingFiles);
  if(dialog.exec()) {
    computeThread.filenames = dialog.selectedFiles();
    data.enabled = this->getEnabled();

    computeThread.isVectorFile = isVectorFile;
    computeThread.start();
  } else {
    QMessageBox::critical(this, "File Dialog Error",
        "A team of highly trained monkeys has been dispatched to help you.");
    convertFinish(false);
  }
}

void AppDisplay::convertFinish(bool success) {
  if(success) {
    if(computeThread.filenames.size() == 1) {
      QString filename = data.filename;
      QMessageBox::information(this, "Conversion Completed!",
          QString("Output File: %1").arg(
            filename.replace(".txt", ".out.txt", Qt::CaseInsensitive)));
    } else {
      QMessageBox::information(this, "Convesion Completed!",
          "Output files are stored in the same directory as the input files.");
    }
  }

  btn_read_custom.setEnabled(true);
  btn_read_vector.setEnabled(true);
  btn_coalesce.setEnabled(true);
  btn_select_all.setEnabled(true);
  btn_select_none.setEnabled(true);
  enableBoxes(true);
}

void AppDisplay::coalesceFinish(bool success) {
  if(success) {
    QMessageBox::information(this, "Coalesce Completed!",
        "Output file is stored in the same directory as the input files.");
  }

  btn_read_custom.setEnabled(true);
  btn_read_vector.setEnabled(true);
  btn_coalesce.setEnabled(true);
  btn_select_all.setEnabled(true);
  btn_select_none.setEnabled(true);
  enableBoxes(true);
}

void AppDisplay::handleError(QString error) {
  QMessageBox::critical(this, "Critical Error", error);
}

void AppDisplay::updateProgress(int progress) {
  bar_convert.setValue(progress);
}

void AppDisplay::keyPressEvent(QKeyEvent* e) {
  // Opens file conversion dialog for custom data files.
  if(e->text() == "c") {
    btn_read_custom.click();
  }

  // Opens file conversion dialog for vector data files.
  if(e->text() == "v") {
    btn_read_vector.click();
  }

  // Opens converted logfile dialog.
  if(e->text() == "s") {
    btn_coalesce.click();
  }

  // Selects all signal checkboxes.
  if(e->text() == "a") {
    btn_select_all.click();
  }

  // Selects no signal checkboxes.
  if(e->text() == "n") {
    btn_select_none.click();
  }

  // Quits the application.
  if(e->text() == "q") {
    QApplication::quit();
  }
}

