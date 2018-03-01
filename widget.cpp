#include "widget.h"
#include "ui_widget.h"

#include <QTcpServer>
#include <QTcpSocket>
#include <QNetworkInterface>
#include <QDebug>

Widget::Widget(QWidget *parent) : QWidget(parent)
  , ui(new Ui::Widget)
  , _tcpSocketIn(Q_NULLPTR)
  , _isIncrease(true)
{
    ui->setupUi(this);
    ui->connectionLabel->setStyleSheet("color: red");
    ui->connectionLabel->setText("Disconnected");
    _tcpServer = new QTcpServer(this);
    connect(_tcpServer, &QTcpServer::newConnection, this, &Widget::onNewConnection);

    if (_tcpServer->listen(QHostAddress::Any, 49001)) {
        qDebug() << "Server is started!";
    }
    else {
        qDebug() << QString("Server is not started: %1").arg(_tcpServer->errorString());
    }

    _trackMarks.setPk(1);
    _trackMarks.updatePost();
    updateViewStartCoordinate();

    _timer = new QTimer(this);
    connect(_timer, &QTimer::timeout, this, &Widget::onTimerTimeout);
}

Widget::~Widget()
{
    _tcpServer->deleteLater();
    _tcpSocketIn->deleteLater();
    delete ui;
}

void Widget::onNewConnection()
{
    if (_tcpSocketIn == Q_NULLPTR) {
        _tcpSocketIn = _tcpServer->nextPendingConnection();
        connect(_tcpSocketIn, &QTcpSocket::disconnected, this, &Widget::onConnectionDisconnected);
        connect(_tcpSocketIn, &QTcpSocket::readyRead, this, &Widget::onReadyRead);
        ui->connectionLabel->setStyleSheet("color: green");
        ui->connectionLabel->setText("Connected");
        qDebug() << "New incoming connection!";
    }
    else {
        _tcpServer->nextPendingConnection()->close();
    }
}

void Widget::onConnectionDisconnected()
{
    disconnect(_tcpSocketIn, &QTcpSocket::disconnected, this, &Widget::onConnectionDisconnected);
    disconnect(_tcpSocketIn, &QTcpSocket::readyRead, this, &Widget::onReadyRead);
    _tcpSocketIn->deleteLater();
    _tcpSocketIn = Q_NULLPTR;
    ui->connectionLabel->setStyleSheet("color: red");
    ui->connectionLabel->setText("Disconnected");
    qDebug() << "Disconnected!";
}

void Widget::onReadyRead()
{
    QDataStream input(_tcpSocketIn);
    input >> _currentValue;
}

void Widget::on_startRegistrationButton_released()
{
    updateViewCurrentCoordinate();
    _currentValue = _startValue;
    if (ui->currentSpeedSpinBox->value()!= 0) {
        _timer->start();
    }
    QDataStream output(_tcpSocketIn);
    output << _trackMarks.getKm() << _trackMarks.getPk() << _trackMarks.getM() << ui->currentSpeedSpinBox->value();
}

void Widget::on_stopRegistrationButton_released()
{
    _timer->stop();
}

void Widget::updateViewStartCoordinate()
{
    qDebug() << _trackMarks.getPostKm(0) << "/" << _trackMarks.getPostKm(1) << " km " << _trackMarks.getPostPk(0) << "/" << _trackMarks.getPostPk(1) << " pk || " << _trackMarks.getKm() << " km " << _trackMarks.getPk() << " pk";

}

void Widget::updateViewCurrentCoordinate()
{
    ui->currentKmLcd->display(_trackMarks.getKm());
    ui->currentPkLcd->display(_trackMarks.getPk());
    ui->currentMLcd->display(_trackMarks.getM());
}


void Widget::on_startKmSpinBox_valueChanged(int value)
{
    _trackMarks.setKm(value);
    _trackMarks.updatePost();
    updateViewStartCoordinate();
}

void Widget::on_startPkSpinBox_valueChanged(int value)
{
    _trackMarks.setPk(value);
    _trackMarks.updatePost();
    updateViewStartCoordinate();
}

void Widget::on_startMSpinBox_valueChanged(int value)
{
    _trackMarks.setM(value);
    _trackMarks.updatePost();
    updateViewStartCoordinate();
}

void Widget::on_currentSpeedSpinBox_valueChanged(int value)
{
    double period = (((value * 1000) / 3600.0));
    int interval = (1 / period) * 1000;
    _timer->setInterval(interval);
}

void Widget::onTimerTimeout()
{
    if (_isIncrease) {
        _trackMarks.setM(_trackMarks.getM() + 1);
    }
    else {
        _trackMarks.setM(_trackMarks.getM() - 1);
    }
    _trackMarks.updatePost();
    updateViewCurrentCoordinate();
    QDataStream output(_tcpSocketIn);
    output << _trackMarks.getKm() << _trackMarks.getPk() << _trackMarks.getM() << ui->currentSpeedSpinBox->value();
}

void Widget::on_increaseRadioButton_clicked()
{
    _isIncrease = true;
    _trackMarks.setDirection(ForwardDirection);
}

void Widget::on_decreaseRadioButton_clicked()
{
    _isIncrease = false;
    _trackMarks.setDirection(BackwardDirection);
}

void Widget::on_nextButton_released()
{
    _trackMarks.next();
    _trackMarks.resetMeter();
    updateViewCurrentCoordinate();
}

void Widget::on_prevButton_released()
{
    _trackMarks.prev();
    _trackMarks.resetMeter();
    updateViewCurrentCoordinate();
}
