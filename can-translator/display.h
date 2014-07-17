/**
 * @file display.h
 * Display class for the can-translator program.
 *
 * @author Andrew Mass
 * @date Created: 2014-06-24
 * @date Modified: 2014-07-16
 */
#ifndef APP_DISPLAY_H
#define APP_DISPLAY_H

#include <QApplication>
#include <QDesktopWidget>
#include <QMessageBox>
#include <QLabel>
#include <QVBoxLayout>
#include <QProgressBar>
#include <QPushButton>
#include <QFont>
#include "config.h"
#include "data.h"

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
     * Calls the cooresponding readData() method in the data class when
     * btn_read is pressed.
     */
    void readData();

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
    
    AppData* data;

    QVBoxLayout* layout;
    QVBoxLayout* layout_headers;

    QLabel* lbl_header;
    QLabel* lbl_subheader;
    QLabel* lbl_messageCount;
    QLabel* lbl_messages;

    QProgressBar* bar_convert;
    QPushButton* btn_read;
};

#endif // APP_DISPLAY_H
