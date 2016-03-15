/**
 * @file display.h
 * Display class for the can-translator program.
 *
 * @author Andrew Mass
 * @date Created: 2014-06-24
 * @date Modified: 2016-03-15
 */
#ifndef APP_DISPLAY_H
#define APP_DISPLAY_H

#include <QFont>
#include <QLabel>
#include <QThread>
#include <QCheckBox>
#include <QKeyEvent>
#include <QGroupBox>
#include <QFileDialog>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QScrollArea>
#include <QApplication>
#include <QProgressBar>
#include "data.h"
#include "config.h"

#define WIDTH 1400
#define HEIGHT 720

/**
 * Class which handles multithreading of the conversion process so that we
 * don't lock up the GUI thread while converting data.
 */
class ComputeThread : public QThread {
  Q_OBJECT

  public:

    /**
     * Boolean which represents whether this thread is going to be used to
     * convert a Vector log file or a custom log file.
     */
    bool isVectorFile;

    /**
     * A list of names of files to convert.
     */
    QStringList filenames;

    /**
     * Pointer to the instance of the data class used for the computation.
     */
    AppData* data;

  signals:

    /**
     * Signal to be executed upon finishing the computation.
     *
     * @param success Whether the conversion was successful.
     */
    void finish(bool success);

  private:

    /**
     * Starts the thread's main computation.
     */
    void run();
};

/**
 * Class with handles multithreading of the coalesce process so that we
 * don't lock up the GUI thread while coalescing logfiles.
 */
class CoalesceComputeThread : public QThread {
  Q_OBJECT

  public:

    /**
     * A list of logfiles to coalesce.
     */
    QStringList filenames;

    /**
     * Pointer to the instance of the data class used for the computation.
     */
    AppData* data;

  signals:

    /**
     * Signal to be executed upon finishing the computation.
     *
     * @param success Whether the conversion was successful.
     */
    void finish(bool success);

  private:

    /**
     * Starts the thread's main computation.
     */
    void run();
};

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
    void keyPressEvent(QKeyEvent* e);

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
     * Scans the grid of checkboxes to see which channels the user wants
     * to convert, then returns this info.
     *
     * @returns A map of message IDs to arrays of bools that contains info about which
     *     channels the user selected to be converted.
     */
    map<uint16_t, vector<bool> > getEnabled();

    /**
     * Sets all channel checkboxes as checked.
     */
    void selectAll();

    /**
     * Sets all channel checkboxes as not checked.
     */
    void selectNone();

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

    /**
     * Sets all channel checkboxes as either checked or not checked.
     *
     * @param checked Whether to set the boxes as checked or not checked.
     */
    void selectBoxes(bool checked);

    /**
     * Sets all channel checkboxes as either enabled or not enabled.
     *
     * @param enabled Whether to set the boxes as enabled or not enabled.
     */
    void enableBoxes(bool enabled);

    AppConfig config;
    AppData data;

    QVBoxLayout layout;
    QVBoxLayout layout_headers;
    QHBoxLayout layout_selects;
    QHBoxLayout layout_reads;
    QHBoxLayout layout_main;
    QVBoxLayout layout_config;
    QVBoxLayout layout_progress;

    QScrollArea area_config;
    QWidget area_config_helper;

    QLabel lbl_header;
    QLabel lbl_subheader;
    QLabel lbl_keymaps;

    QPushButton btn_read_custom;
    QPushButton btn_read_vector;
    QPushButton btn_coalesce;
    QPushButton btn_select_all;
    QPushButton btn_select_none;

    QProgressBar bar_convert;

    ComputeThread computeThread;
    CoalesceComputeThread coalesceComputeThread;
};

#endif // APP_DISPLAY_H
