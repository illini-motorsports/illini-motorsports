/**
 * @file display.h
 * Display class for the telemetry-monitor.
 *
 * @author Andrew Mass
 * @date Created: 2014-03-03
 * @date Modified: 2014-05-06
 */
#ifndef APP_DISPLAY_H
#define APP_DISPLAY_H

#include <QApplication>
#include <QDesktopWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QProgressBar>
#include <QFont>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include "config.h"
#include "data.h"

#define WIDTH 1280
#define HEIGHT 720

/**
 * Class which handles the construction of the GUI and the updating of
 * data as received by the AppData class.
 */
class AppDisplay : public QWidget {
  Q_OBJECT

  public:
    AppDisplay(QWidget* parent = 0);

  public slots:

    /**
     * Function that searches through all the metrics and bars and updates
     * the data at one that is currently displaying the CAN message with Id
     * msgId. The new value of the metric or bar will be data.
     *
     * @param msgId The id of the CAN message to update values for.
     * @param data The data to display for the specified CAN message.
     */
    void updateData(unsigned char msgId, double data);

    void updateMessageCounter(unsigned int num);

    void updateDropCounter(unsigned int num);

    void showMessage(QString header, QString message);

    void setConnected(bool connected);

  protected:
    void keyPressEvent(QKeyEvent* e);

  private slots:
    void readData();
    void handleError(QSerialPort::SerialPortError error);
    void onQuit();

  private:
    map<unsigned char, message> messages;

    QSerialPort* serialPort;

    AppData data;

    QVBoxLayout* layout;
    QHBoxLayout* layout_above;
    QHBoxLayout* layout_sub;

    QVBoxLayout* layout_headers;
    QVBoxLayout* layout_stats_left;
    QVBoxLayout* layout_stats_right;

    QVBoxLayout* layout_left;
    QVBoxLayout* layout_right;

    QHBoxLayout* layouts_left[NUM_METRICS];
    QHBoxLayout* layouts_right[NUM_METRICS];

    QHBoxLayout* layouts_bar[NUM_BARS];

    QLabel* lbl_header;
    QLabel* lbl_subheader;
    QLabel* lbl_ports;
    QPushButton* btn_quit;
    QLabel* lbl_connected;

    QLabel* lbl_stats_left_title;
    QLabel* lbl_stats_left_data;
    QLabel* lbl_stats_right_title;
    QLabel* lbl_stats_right_data;

    QLabel* lbls_left_title[NUM_METRICS];
    QLabel* lbls_right_title[NUM_METRICS];

    QLabel* lbls_left_data[NUM_METRICS];
    QLabel* lbls_right_data[NUM_METRICS];

    QLabel* lbls_bar_title[NUM_BARS];
    QLabel* lbls_bar_data[NUM_BARS];
    QProgressBar* bars_bar[NUM_BARS];
};

#endif // APP_DISPLAY_H
