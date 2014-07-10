/**
 * @file display.h
 * Display class for the can-translator program.
 *
 * @author Andrew Mass
 * @date Created: 2014-06-24
 * @date Modified: 2014-07-05
 */
#ifndef APP_DISPLAY_H
#define APP_DISPLAY_H

#include <QApplication>
#include <QDesktopWidget>
#include <QMessageBox>
#include <QLabel>
#include <QVBoxLayout>
#include <QProgressBar>
#include <QFont>
#include "config.h"

#define WIDTH 1280
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

  private slots:

    /**
     * Displays the error message in a critical error message box.
     */
    void handleError(QString error);

  private:

    QVBoxLayout* layout;

    QVBoxLayout* layout_headers;

    QLabel* lbl_header;
    QLabel* lbl_subheader;
    QLabel* lbl_messageCount;
    QLabel* lbl_messages;

    QProgressBar* bar_convert;
    QProgressBar* bar_normalize;
    QProgressBar* bar_write;
};

#endif // APP_DISPLAY_H
