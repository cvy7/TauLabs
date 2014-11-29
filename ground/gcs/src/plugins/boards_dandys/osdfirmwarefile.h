#ifndef OSDFIRMWAREFILE_H
#define OSDFIRMWAREFILE_H

#include <QFile>
#include <QDateTime>
#include <QString>

class OsdFirmwareFile : public QFile
{
    Q_OBJECT
public:
    explicit OsdFirmwareFile(QObject *parent = 0);
    explicit OsdFirmwareFile(const QString &name);
    explicit OsdFirmwareFile(const QString & name, QObject * parent);
    virtual bool open(OpenMode mode);

    bool imageValid() const;
    QDateTime imageTimeStamp() const;
    QString imageVersion() const;
signals:

public slots:

private:
    static quint32 crc32(quint32 oldCrc, quint8 new_byte);
    static quint32 calcCrc32(const QByteArray &ba);
    bool crcOk;
    QDateTime timeStamp;
    QString version;
};

#endif // OSDFIRMWAREFILE_H
