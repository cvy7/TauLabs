/**
 ******************************************************************************
 *
 * @file       draco.cpp
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013
 *
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup Boards_Dandys Dandys boards support Plugin
 * @{
 * @brief Plugin to support boards by Dandys
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "draco.h"

#include <uavobjectmanager.h>
#include "uavobjectutil/uavobjectutilmanager.h"
#include <extensionsystem/pluginmanager.h>

#include "hwdraco.h"

/**
 * @brief Draco::Draco
 *  This is the Draco board definition
 */
Draco::Draco(void)
{
    // Initialize our USB Structure definition here:
    USBInfo board;
    board.vendorID = 0x1d50;
    board.productID = 0xaaaa;

    setUSBInfo(board);

    boardType = 0xff;

    // Define the bank of channels that are connected to a given timer
    channelBanks.resize(6);
    channelBanks[0] = QVector<int> () << 1 << 2;
    channelBanks[1] = QVector<int> () << 3 << 4 << 5 << 6;
    channelBanks[2] = QVector<int> () << 7 << 8 << 9 << 10;
    channelBanks[3] = QVector<int> () << 11 << 12; // TODO: check if 12 channels are possible here (TL is limited to 10 by UAVOs)
}

Draco::~Draco()
{

}

QString Draco::shortName()
{
    return QString("draco");
}

QString Draco::boardDescription()
{
    return QString("draco flight control rev. 1 by Dandys");
}

//! Return which capabilities this board has
bool Draco::queryCapabilities(BoardCapabilities capability)
{
    switch(capability) {
    case BOARD_CAPABILITIES_GYROS:
        return true;
    case BOARD_CAPABILITIES_ACCELS:
        return true;
    case BOARD_CAPABILITIES_MAGS:
        return true;
    case BOARD_CAPABILITIES_BAROS:
        return true;
    case BOARD_CAPABILITIES_RADIO:
        return false;
    }
    return false;
}


/**
 * @brief Draco::getSupportedProtocols
 *  TODO: this is just a stub, we'll need to extend this a lot with multi protocol support
 * @return
 */
QStringList Draco::getSupportedProtocols()
{
    return QStringList("uavtalk");
}

QPixmap Draco::getBoardPicture()
{
    return QPixmap(":/dandys/images/draco.png");
}

QString Draco::getHwUAVO()
{
    return "HwDraco";
}

int Draco::queryMaxGyroRate()
{
    ExtensionSystem::PluginManager *pm = ExtensionSystem::PluginManager::instance();
    UAVObjectManager *uavoManager = pm->getObject<UAVObjectManager>();
    HwDraco *hwDraco = HwDraco::GetInstance(uavoManager);
    Q_ASSERT(hwDraco);
    if (!hwDraco)
        return 0;

    HwDraco::DataFields settings = hwDraco->getData();

    switch(settings.GyroRange) {
    case HwDraco::GYRORANGE_250:
        return 250;
    case HwDraco::GYRORANGE_500:
        return 500;
    case HwDraco::GYRORANGE_1000:
        return 1000;
    case HwDraco::GYRORANGE_2000:
        return 2000;
    default:
        return 500;
    }
}
