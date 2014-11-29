#include <QtEndian>
#include <QTextCodec>
#include <QDebug>
#include "osdfirmwarefile.h"

OsdFirmwareFile::OsdFirmwareFile(QObject *parent) :
    QFile(parent),
    crcOk(false)
{
}

OsdFirmwareFile::OsdFirmwareFile(const QString &name) :
    QFile(name),
    crcOk(false)
{
}

OsdFirmwareFile::OsdFirmwareFile(const QString &name, QObject *parent) :
    QFile(name, parent),
    crcOk(false)
{
}

bool OsdFirmwareFile::open(OpenMode mode)
{
    bool result = QFile::open(mode);

    if (!result)
        return result;

    quint64 sz = size();
    if ((sz > 262144) || (sz < 556))
        return result;

    QByteArray content = readAll();
    seek(0);

    quint32 fsz = qFromLittleEndian<quint32>((const quint8*)content.mid(512).constData());
    if ((fsz > sz) || (fsz < 556))
        return result;
    quint32 crc = qFromLittleEndian<quint32>((const quint8*)content.mid(fsz - 4).constData());
    quint32 crcComputed = calcCrc32(content.mid(0, fsz - 4));

    if (crc != crcComputed)
        return result;

    crcOk = true;

    quint32 unixTime = qFromLittleEndian<quint32>((const quint8*)content.mid(fsz - 8).constData());
    timeStamp = QDateTime::fromMSecsSinceEpoch((quint64)unixTime * 1000);
    version = QTextCodec::codecForMib(3)->toUnicode(content.mid(fsz - 40).constData());
    return result;
}

bool OsdFirmwareFile::imageValid() const
{
    return crcOk;
}

QDateTime OsdFirmwareFile::imageTimeStamp() const
{
    return timeStamp;
}

QString OsdFirmwareFile::imageVersion() const
{
    return version;
}

quint32 OsdFirmwareFile::crc32(quint32 oldCrc, quint8 new_byte)
{
    quint32  crc;
    quint8 i;

    crc = oldCrc ^ new_byte;
    i = 8;
    do {
        if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320L; // CRC-32 reverse polynom
        else crc = crc >> 1;
    } while(--i);
    return (crc);
}

quint32 OsdFirmwareFile::calcCrc32(const QByteArray &ba)
{
    quint32 crc = 0xffffffff;
    const quint8 *p = (const quint8*)ba.constData();
    int cnt = ba.size();

    while (cnt--)
        crc = crc32(crc, *p++);
    crc ^= 0xffffffff;

    return crc;
}
