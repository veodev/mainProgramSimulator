#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include "tmrussian.h"
#include <QTimer>

namespace Ui {
class Widget;
}

class QTcpServer;
class QTcpSocket;

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = 0);
    ~Widget();

private slots:
    void onNewConnection();
    void onConnectionDisconnected();
    void onReadyRead();
    void on_startRegistrationButton_released();
    void on_stopRegistrationButton_released();
    void on_startKmSpinBox_valueChanged(int value);
    void on_startPkSpinBox_valueChanged(int value);
    void on_startMSpinBox_valueChanged(int value);
    void on_currentSpeedSpinBox_valueChanged(int value);

    void onTimerTimeout();

    void on_increaseRadioButton_clicked();

    void on_decreaseRadioButton_clicked();


    void on_nextButton_released();

    void on_prevButton_released();

private:
    void updateViewStartCoordinate();
    void updateViewCurrentCoordinate();

private:
    Ui::Widget *ui;

    QTcpServer * _tcpServer;
    QTcpSocket * _tcpSocketIn;
    int _startValue;
    int _currentValue;

    TMRussian _trackMarks;
    QTimer* _timer;
    bool _isIncrease;

};

#endif // WIDGET_H
