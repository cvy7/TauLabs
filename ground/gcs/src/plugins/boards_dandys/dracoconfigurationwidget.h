/**
 ******************************************************************************
 *
 * @file       dracoconfigurationwidget.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2014
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup Boards_Dandys Dandys boards support Plugin
 * @{
 * @brief  Widget for draco configuration
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
#ifndef DRACOCONFIGURATIONWIDGETt_H
#define DRACOCONFIGURATIONWIDGETt_H

#include "ui_dracoconfiguration.h"
#include "hwfieldselector.h"
#include "../uavobjectwidgetutils/configtaskwidget.h"
#include "extensionsystem/pluginmanager.h"
#include "uavobjectmanager.h"
#include "uavobject.h"
#include "osdfirmwarefile.h"
#include <QList>
#include <QWidget>
#include <QTimer>
#include <QMutex>
#include <hwdracoosdfwupdate.h>

class Ui_Widget;

class DracoConfigurationWidget : public ConfigTaskWidget
{
    Q_OBJECT

public:
    explicit DracoConfigurationWidget(QWidget *parent = 0);
    ~DracoConfigurationWidget();

private slots:
    void settingsUpdated(UAVObject*,bool);
    void on_buttonOpenFwFile_clicked();
    void on_buttonFlashFw_clicked();
    void onOsdFwUpdateUpdated(UAVObject *obj);
    void onOsdFwUpdateActionTimeout();

private:
    enum OsdFwAction {
        NoAction,
        ActionVersion,
        ActionWriteStart,
        ActionWriteChunk,
        ActionExitBootloader,
        ActionEnterBootloader,
    };
    void startOsdFwAction(OsdFwAction action);

    void setOsdFwStatusMessage(quint8 severity, const QString &message);
    void updateFields();
    Ui_dracoconfiguration *dracoConfigurationWidget;

    QList<QString> allHwSettings;
    UAVObject *hwSettingsObject;
    bool settingSelected;    
    QString osdFwFileName;
    QByteArray osdFwFileData;
    quint32 osdFwFileDataOffset;

    QList <HwFieldSelector *> fieldWidgets;
    UAVObjectManager    *objManager;
    HWDracoOsdFwUpdate *osdFwUpdate;
    OsdFirmwareFile *osdFwFile;
    QTimer *actionTimeoutTimer;
    OsdFwAction osdFwAction;
};

#endif // DRACOCONFIGURATIONWIDGETt_H
