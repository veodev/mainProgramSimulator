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
  , _isRegistrationOn(false)
{
    ui->setupUi(this);
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
    connect(_timer, &QTimer::timeout, this, &Widget::onTimerTimeout, Qt::DirectConnection);

    _sendTimer = new QTimer(this);
    _sendTimer->setInterval(100);
    connect(_sendTimer, &QTimer::timeout, this, &Widget::onSendData, Qt::DirectConnection);
}

Widget::~Widget()
{
    _sendTimer->stop();
    if (_tcpSocketIn != Q_NULLPTR) {
        disconnect(_tcpSocketIn, &QTcpSocket::disconnected, this, &Widget::onConnectionDisconnected);
    }
//    _tcpSocketIn->close();
//    _tcpServer->close();
//    _tcpServer->deleteLater();
//    _tcpSocketIn->deleteLater();
    delete ui;
}

void Widget::onNewConnection()
{
    if (_tcpSocketIn == Q_NULLPTR) {
        _tcpSocketIn = _tcpServer->nextPendingConnection();
        connect(_tcpSocketIn, &QTcpSocket::disconnected, this, &Widget::onConnectionDisconnected);
        connect(_tcpSocketIn, &QTcpSocket::readyRead, this, &Widget::onReadyRead);
        ui->connectionLabel->setPixmap(QPixmap(":/icons/online_48px.png"));
        qDebug() << "New incoming connection!";
        sendData(UpdateState);
        _sendTimer->start();
    }
    else {
        _tcpServer->nextPendingConnection()->close();
    }
}

void Widget::onConnectionDisconnected()
{
    _sendTimer->stop();
    disconnect(_tcpSocketIn, &QTcpSocket::disconnected, this, &Widget::onConnectionDisconnected);
    disconnect(_tcpSocketIn, &QTcpSocket::readyRead, this, &Widget::onReadyRead);
    _tcpSocketIn->deleteLater();
    _tcpSocketIn = Q_NULLPTR;
    ui->connectionLabel->setPixmap(QPixmap(":/icons/offline_48px.png"));
    qDebug() << "Disconnected!";
}

void Widget::onReadyRead()
{
    QDataStream input(_tcpSocketIn);
}

void Widget::onBytesWritten(quint64 bytes)
{
    qDebug() << "Write: " << bytes;
}

void Widget::on_startRegistrationButton_released()
{
    _isRegistrationOn = true;
    updateViewCurrentCoordinate();
    sendData(Headers::StartRegistration);
}

void Widget::on_stopRegistrationButton_released()
{
    _isRegistrationOn = false;
    sendData(Headers::StopRegistration);
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

void Widget::sendData(Headers header)
{
    if (_tcpSocketIn == Q_NULLPTR) {
        return;
    }

    QDataStream output(_tcpSocketIn);
    output << header;

    switch (header) {
    case StartRegistration:
        output << _isIncrease << _trackMarks.getKm() << _trackMarks.getPk() << _trackMarks.getM();
        break;
    case StopRegistration:
        break;
    case CurrentMeter:
        output << _trackMarks.getM() << ui->currentSpeedSpinBox->value();
        break;
    case Mark:
        break;
    case UpdateState:
        output << _isRegistrationOn << _isIncrease << _trackMarks.getKm() << _trackMarks.getPk() << _trackMarks.getM() << ui->currentSpeedSpinBox->value();
        break;
    }
    _tcpSocketIn->flush();
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
    value == 0 ? _timer->stop() : _timer->start();
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
}

void Widget::onSendData()
{
    if (_tcpSocketIn != Q_NULLPTR) {
        sendData(Headers::CurrentMeter);
    }
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

void Widget::on_markButton_toggled(bool checked)
{
    if (checked) {
        ui->markLabel->setPixmap(QPixmap(":/icons/mark_96px.png"));
    }
    else {
        ui->markLabel->setPixmap(QPixmap());
    }
}

void Widget::on_pushButton_toggled(bool checked)
{
    if (checked) {
        _timer->stop();
    }
    else {
        _timer->start();
    }
}

void Widget::on_closeConectionButton_released()
{
    if (_tcpSocketIn != Q_NULLPTR) {
        _tcpSocketIn->close();
    }
}
