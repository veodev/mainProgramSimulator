#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include "tmrussian.h"
#include <QTimer>

namespace Ui {
class Widget;
}

enum Headers {
    StartRegistration,
    StopRegistration,
    CurrentMeter,
    Mark,
    UpdateState
};

class QTcpServer;
class QTcpSocket;

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = 0);
    ~Widget();

private:
    void viewMark();
    void updateViewStartCoordinate();
    void updateViewCurrentCoordinate();
    void sendData(Headers header);
    void updateState();

private slots:
    void onNewConnection();
    void onConnectionDisconnected();
    void onReadyRead();
    void onBytesWritten(quint64 bytes);
    void on_startRegistrationButton_released();
    void on_stopRegistrationButton_released();
    void on_startKmSpinBox_valueChanged(int value);
    void on_startPkSpinBox_valueChanged(int value);
    void on_startMSpinBox_valueChanged(int value);
    void on_currentSpeedSpinBox_valueChanged(int value);
    void onTimerTimeout();
    void onSendData();
    void on_increaseRadioButton_clicked();
    void on_decreaseRadioButton_clicked();
    void on_nextButton_released();
    void on_prevButton_released();
    void on_markButton_toggled(bool checked);
    void on_pushButton_toggled(bool checked);

private:
    Ui::Widget *ui;

    QTcpServer * _tcpServer;
    QTcpSocket * _tcpSocketIn;
    TMRussian _trackMarks;
    QTimer* _timer;
    QTimer* _sendTimer;
    bool _isIncrease;
    bool _isRegistrationOn;
};

#endif // WIDGET_H
