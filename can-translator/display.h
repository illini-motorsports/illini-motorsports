/**
 * @file display.h
 * Display class for the can-translator program.
 *
 * @author Andrew Mass
 * @date Created: 2014-06-24
 * @date Modified: 2016-03-20
 */
#ifndef APP_DISPLAY_H
#define APP_DISPLAY_H

#include "compute.h"
#include "config.h"
#include "data.h"
#include <QApplication>
#include <QCheckBox>
#include <QFileDialog>
#include <QFont>
#include <QGroupBox>
#include <QKeyEvent>
#include <QLabel>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>

#define WIDTH 1400
#define HEIGHT 720

/**
 * Class which handles the construction of the GUI and button presses.
 */
class AppDisplay : public QWidget {
  Q_OBJECT

public:
  /**
   * Default constructor for AppDisplay class. Essentially the main entry
   * point for the entire application.
   */
  AppDisplay();

  /**
   * If true, the configuration scanning process was successful and it is
   * safe to continue to execution of the program.
   */
  bool successful;

protected:
  /**
   * Called when the user presses a key on their keyboard. Not called
   * when a user presses a key that isn't on their keyboard.
   *
   * @params e The key event that occurred.
   */
  void keyPressEvent(QKeyEvent *e);

private slots:

  /**
   * Calls the cooresponding readData() method in the data class when
   * btn_read is pressed.
   */
  void readDataCustom();

  /**
   * Calls the cooresponding readData() method in the data class when
   * btn_read is pressed.
   */
  void readDataVector();

  /**
   * Calls the cooresponding coalesceLogfiles() function in the data
   * class when btn_coalesce is pressed.
   */
  void coalesceLogfiles();

  /**
   * Called when the thread has fininshed the conversion process.
   *
   * @param success Whether the conversion was successful.
   */
  void convertFinish(bool success);

  /**
   * Called when the coalesce thread has finished.
   *
   * @param success Whether the coalesce was successful.
   */
  void coalesceFinish(bool success);

  /**
   * Slot to catch whenever we need to add another file progress bar to the
   * progress bar layout.
   *
   * @param filename - The name of the file to add the progress bar for.
   */
  void addFileProgress(QString filename);

  /**
   * Displays the error message in a critical error message box.
   *
   * @param error The error message to display.
   */
  void handleError(QString error);

  /**
   * Updates the progress bar with the latest progress counter.
   *
   * @param progress The latest progress counter to display.
   */
  void updateProgress(int progress);

private:
  /**
   * Calls the cooresponding readData() method in the data class when
   * btn_read is pressed.
   *
   * @params isVectorFile Whether the file to convert is in the Vector format.
   */
  void readData(bool isVectorFile);

  AppConfig *config;
  AppData *data;

  QVBoxLayout *layout;
  QVBoxLayout *layout_headers;
  QHBoxLayout *layout_reads;
  QHBoxLayout *layout_main;
  QVBoxLayout *layout_config;
  QVBoxLayout *layout_progress;

  QScrollArea *area_config;
  QWidget *area_config_helper;

  QLabel *lbl_header;
  QLabel *lbl_subheader;
  QLabel *lbl_keymaps;

  QPushButton *btn_read_custom;
  QPushButton *btn_read_vector;
  QPushButton *btn_coalesce;

  QProgressBar *bar_convert;

  ComputeThread *computeThread;
  CoalesceComputeThread *coalesceComputeThread;
};

#endif // APP_DISPLAY_H
