/**
 ******************************************************************************
 * @file       draoconfigurationwidget.cpp
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2014
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup Boards_Dandys Dandys boards support Plugin
 * @{
 * @brief Widget for draco configuration
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
#include "dracoconfigurationwidget.h"
#include "../../plugins/config/hwfieldselector.h"
#include <QMutexLocker>
#include <QErrorMessage>
#include <QDebug>
#include <QFileDialog>

namespace Ui {
    class DracoConfigurationWidget;
}
/**
 * @brief DracoConfigurationWidget::DracoConfigurationWidget Constructed when a new
 * board connection is established
 * @param parent The main configuration widget
 */
DracoConfigurationWidget::DracoConfigurationWidget(QWidget *parent) :
        ConfigTaskWidget(parent),
        dracoConfigurationWidget(new Ui_dracoconfiguration),
        hwSettingsObject(NULL),
        settingSelected(false),
        objManager(NULL),
        osdFwUpdate(NULL),
        osdFwFile(NULL),
        actionTimeoutTimer(new QTimer(this)),
        osdFwAction(NoAction)
{
    dracoConfigurationWidget->setupUi(this);

    fieldWidgets.clear();

    addApplySaveButtons(dracoConfigurationWidget->applyButton,dracoConfigurationWidget->saveButton);
    addReloadButton(dracoConfigurationWidget->reloadButton, 0);

    // Query the board plugin for the connected board to get the specific
    // hw settings object
    ExtensionSystem::PluginManager *pm = ExtensionSystem::PluginManager::instance();
    Q_ASSERT(pm != NULL);
    objManager = pm->getObject<UAVObjectManager>();
    Q_ASSERT(objManager != NULL);

    UAVObjectUtilManager* uavoUtilManager = pm->getObject<UAVObjectUtilManager>();
    Core::IBoardType* board = uavoUtilManager->getBoardType();
    Q_ASSERT(board != NULL);

    QString hwSwettingsObject = board->getHwUAVO();

    UAVObject *obj = getObjectManager()->getObject(hwSwettingsObject);
    Q_ASSERT(obj != NULL);
    qDebug() << "Checking object " << obj->getName();
    connect(obj,SIGNAL(transactionCompleted(UAVObject*,bool)), this, SLOT(settingsUpdated(UAVObject*,bool)));
    obj->requestUpdate();

    osdFwUpdate = HWDracoOsdFwUpdate::GetInstance(objManager);
    Q_ASSERT(osdFwUpdate != NULL);

    connect(osdFwUpdate, SIGNAL(objectUpdated(UAVObject*)), this, SLOT(onOsdFwUpdateUpdated(UAVObject*)));
    connect(actionTimeoutTimer, SIGNAL(timeout()), SLOT(onOsdFwUpdateActionTimeout()));
    actionTimeoutTimer->setInterval(2000);
    actionTimeoutTimer->setSingleShot(true);
    startOsdFwAction(ActionVersion);
    disableMouseWheelEvents();
}

DracoConfigurationWidget::~DracoConfigurationWidget()
{
    delete dracoConfigurationWidget;
    delete actionTimeoutTimer;
    if (osdFwFile != NULL)
        delete osdFwFile;
}

/**
 * @brief DracoConfigurationWidget::onOsdFwUpdateUpdated Evaluates statuses and error codes
 * of OSD firmware update operations (actions) and depending on it starting next action
 */
void DracoConfigurationWidget::onOsdFwUpdateUpdated(UAVObject *obj)
{
    Q_UNUSED(obj);
    HWDracoOsdFwUpdate::DataFields osdFwUpdateData = osdFwUpdate->getData();
    if (osdFwUpdateData.Control == HWDracoOsdFwUpdate::CONTROL_STATEIDLE ||
            osdFwUpdateData.Control == HWDracoOsdFwUpdate::CONTROL_STATEIDLEBOOTLOADER)
    {
        if (osdFwAction == ActionVersion) {
            startOsdFwAction(NoAction);
            if (osdFwUpdateData.Error != HWDracoOsdFwUpdate::ERROR_NOERROR) {
                startOsdFwAction(NoAction);
                setOsdFwStatusMessage(2, tr("Failed to get FW version from OSD MCU"));
            } else {
                QByteArray versionBa = QByteArray((char*)osdFwUpdateData.Data,
                           (osdFwUpdateData.DataSize > sizeof(osdFwUpdateData.Data))
                           ? sizeof(osdFwUpdateData.Data)
                           : osdFwUpdateData.DataSize);
                QString version = QString(versionBa);
                if (osdFwUpdateData.Control == HWDracoOsdFwUpdate::CONTROL_STATEIDLEBOOTLOADER)
                    version += " (bootloader)";
                dracoConfigurationWidget->labelFwVersion->setText(version);
            }
        } else if (osdFwAction == ActionEnterBootloader) {
            if (osdFwUpdateData.Error != HWDracoOsdFwUpdate::ERROR_NOERROR) {
                startOsdFwAction(NoAction);
                setOsdFwStatusMessage(2, tr("Failed to enter bootloader"));
            } else {
                startOsdFwAction(ActionWriteStart);
            }
        } else if (osdFwAction == ActionWriteStart) {
            if (osdFwUpdateData.Error != HWDracoOsdFwUpdate::ERROR_NOERROR) {
                startOsdFwAction(NoAction);
                setOsdFwStatusMessage(2, tr("Failed to start flashing"));
            } else {
                startOsdFwAction(ActionWriteChunk);
            }
        } else if (osdFwAction == ActionWriteChunk) {
            if (osdFwUpdateData.Error != HWDracoOsdFwUpdate::ERROR_NOERROR) {
                startOsdFwAction(NoAction);
                setOsdFwStatusMessage(2, tr("Failed to write flash memory"));
            } else {
                if ((int)osdFwFileDataOffset >= osdFwFileData.size())
                    startOsdFwAction(ActionExitBootloader);
                else
                    startOsdFwAction(ActionWriteChunk);
            }
        } else if (osdFwAction == ActionExitBootloader) {
            if (osdFwUpdateData.Error != HWDracoOsdFwUpdate::ERROR_NOERROR) {
                startOsdFwAction(NoAction);
                setOsdFwStatusMessage(2, tr("Failed to boot flashed image"));
            } else {
                startOsdFwAction(NoAction);
                setOsdFwStatusMessage(0, tr("Firmware successfuly uploaded"));
                dracoConfigurationWidget->labelFwVersion->setText(dracoConfigurationWidget->labelFileVersion->text());
            }
        }
    }
}

/**
 * @brief DracoConfigurationWidget::startOsdFwAction Begins OSD firmware update actions, every action will eventually result
 * in @ref onOsdFwUpdateUpdated where its result is evaluated and next action is started, or it will timeout in @ref onOsdFwUpdateActionTimeout
 * @param action selects action
 */
void DracoConfigurationWidget::startOsdFwAction(OsdFwAction action)
{
    HWDracoOsdFwUpdate::DataFields osdFwUpdateData = osdFwUpdate->getData();
    osdFwAction = action;
    if (action == NoAction) {
        setOsdFwStatusMessage(0, tr("Ready"));
        dracoConfigurationWidget->buttonFlashFw->setEnabled(!osdFwFileName.isEmpty());
        dracoConfigurationWidget->buttonOpenFwFile->setEnabled(true);

        actionTimeoutTimer->stop();
    } else if (action == ActionVersion) {
        setOsdFwStatusMessage(0, tr("Getting FW version from OSD MCU"));
        osdFwUpdateData.Control = HWDracoOsdFwUpdate::CONTROL_GETVERSION;
        osdFwUpdate->setData(osdFwUpdateData);
        actionTimeoutTimer->start();

        dracoConfigurationWidget->buttonFlashFw->setEnabled(false);
        dracoConfigurationWidget->buttonOpenFwFile->setEnabled(true);

    } else if (action == ActionEnterBootloader) {
        setOsdFwStatusMessage(0, tr("Entering bootloader mode"));
        osdFwUpdateData.Control = HWDracoOsdFwUpdate::CONTROL_ENTERBOOTLOADER;
        osdFwUpdate->setData(osdFwUpdateData);
        actionTimeoutTimer->start();

        dracoConfigurationWidget->buttonFlashFw->setEnabled(false);
        dracoConfigurationWidget->buttonOpenFwFile->setEnabled(false);
        dracoConfigurationWidget->progressBar->setValue(0);
    } else if (action == ActionWriteStart) {
        setOsdFwStatusMessage(0, tr("Flashing"));
        osdFwFileDataOffset = 0;
        osdFwUpdateData.Control = HWDracoOsdFwUpdate::CONTROL_STARTWRITEFW;
        osdFwUpdateData.DataSize = osdFwFileData.size();
        osdFwUpdate->setData(osdFwUpdateData);
        actionTimeoutTimer->start();
    } else if (action == ActionWriteChunk) {
        quint32 chunkSize = sizeof(osdFwUpdateData.Data);
        if ((osdFwFileData.size() - osdFwFileDataOffset) <= chunkSize)
            chunkSize = osdFwFileData.size() - osdFwFileDataOffset;

        osdFwUpdateData.Control = HWDracoOsdFwUpdate::CONTROL_WRITECHUNK;
        osdFwUpdateData.DataSize = chunkSize;
        memcpy(osdFwUpdateData.Data, osdFwFileData.mid(osdFwFileDataOffset).constData(), chunkSize);

        osdFwFileDataOffset += chunkSize;
        osdFwUpdate->setData(osdFwUpdateData);
        actionTimeoutTimer->start();
        dracoConfigurationWidget->progressBar->setValue((100 * osdFwFileDataOffset) / osdFwFileData.size());
    } else if (action == ActionExitBootloader) {
        setOsdFwStatusMessage(0, tr("Booting uploaded firmware"));
        osdFwFileDataOffset = 0;
        osdFwUpdateData.Control = HWDracoOsdFwUpdate::CONTROL_EXITBOOTLOADER;
        osdFwUpdate->setData(osdFwUpdateData);
        actionTimeoutTimer->start();

        dracoConfigurationWidget->progressBar->setValue(100);
    }
}

/**
 * @brief DracoConfigurationWidget::onOsdFwUpdateActionTimeout Timeout of action started by
 * @ref startOsdFwAction
 */
void DracoConfigurationWidget::onOsdFwUpdateActionTimeout()
{
    startOsdFwAction(NoAction);
    setOsdFwStatusMessage(2, tr("Action timeout, OSD not responding"));
}

/**
 * @brief DracoConfigurationWidget::settingsUpdated called when HW settings are updated
 * @param obj HW UAVO
 * @param success false when update failed
 */
void DracoConfigurationWidget::settingsUpdated(UAVObject *obj, bool success)
{
    if (success && !settingSelected) {
        qDebug() << "Selected object " << obj->getName();
        settingSelected = true;

        hwSettingsObject = obj;
        updateFields();

        QList<int> reloadGroups;
        reloadGroups << 0;

        addUAVObject(obj, &reloadGroups);
        refreshWidgetsValues();

        // Have to force the form as clean (unedited by user) since refreshWidgetsValues forces it to dirty.
        setDirty(false);
    }
}

/**
 * @brief DracoConfigurationWidget::setOsdFwStatusMessage display OSD firmware update status to user
 * @param severity sverity of message (0 = normal, 1 = warning, 2 = error)
 * @param message text message
 */
void DracoConfigurationWidget::setOsdFwStatusMessage(quint8 severity, const QString &message)
{
    dracoConfigurationWidget->labelFwStatus->setText(message);
    switch (severity) {
    case 0:
        dracoConfigurationWidget->labelFwStatus->setStyleSheet("QLabel { color : black; }");
        break;
    case 1:
        dracoConfigurationWidget->labelFwStatus->setStyleSheet("QLabel { color : orange; }");
        break;
    case 2:
    default:
        dracoConfigurationWidget->labelFwStatus->setStyleSheet("QLabel { color : red; }");
        break;
    }
}

/**
 * @brief DracoConfigurationWidget::on_buttonFlashFw_clicked Start OSD firmware update
 */
void DracoConfigurationWidget::on_buttonFlashFw_clicked()
{
    if (!osdFwFile->open(QIODevice::ReadOnly)) {
        setOsdFwStatusMessage(2, tr("Failed to load firmware file"));
        osdFwFileName = "";
        dracoConfigurationWidget->buttonFlashFw->setEnabled(false);
        dracoConfigurationWidget->labelFileVersion->setText("");
        dracoConfigurationWidget->labelFileName->setText("");
        return;
    }
    osdFwFileData = osdFwFile->readAll();
    osdFwFileDataOffset = 0;
    osdFwFile->close();
    startOsdFwAction(ActionEnterBootloader);
}

/**
 * @brief DracoConfigurationWidget::on_buttonOpenFwFile_clicked Open, load and check OSD firmware file
 */
void DracoConfigurationWidget::on_buttonOpenFwFile_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Open File"),QDir::homePath(),"*.bin");
    if (filename.isEmpty())
        return;

    bool fileOk = false;
    if (osdFwFile != NULL)
        delete osdFwFile;
    osdFwFile = new OsdFirmwareFile(filename);
    if (osdFwFile->open(QIODevice::ReadOnly)) {
        if (osdFwFile->imageValid()) {
            setOsdFwStatusMessage(0, tr("Firmware file loaded"));
            dracoConfigurationWidget->buttonFlashFw->setEnabled(true);
            osdFwFileName = filename;
            fileOk = true;
            dracoConfigurationWidget->labelFileVersion->setText(osdFwFile->imageVersion());
            dracoConfigurationWidget->labelFileName->setText(filename);
        }
        osdFwFile->close();
    }
    if (!fileOk) {
        setOsdFwStatusMessage(2, tr("Firmware file invalid"));
        osdFwFileName = "";
        dracoConfigurationWidget->buttonFlashFw->setEnabled(false);
        dracoConfigurationWidget->labelFileVersion->setText("");
        dracoConfigurationWidget->labelFileName->setText("");
    }
}

/**
 * @brief DracoConfigurationWidget::updateFields Update the list of fields and show all of them
 * on the UI.  Connect each to the smart save system.
 */
void DracoConfigurationWidget::updateFields()
{
    Q_ASSERT(settingSelected);
    Q_ASSERT(hwSettingsObject != NULL);

    QLayout *layout = dracoConfigurationWidget->portSettingsFrame->layout();
    for (int i = 0; i < fieldWidgets.size(); i++)
        layout->removeWidget(fieldWidgets[i]);
    fieldWidgets.clear();

    QList <UAVObjectField*> fields = hwSettingsObject->getFields();
    for (int i = 0; i < fields.size(); i++) {
        if (fields[i]->getType() != UAVObjectField::ENUM)
            continue;
        HwFieldSelector *sel = new HwFieldSelector(this);
        layout->addWidget(sel);
        sel->setUavoField(fields[i]);
        fieldWidgets.append(sel);
        addUAVObjectToWidgetRelation(hwSettingsObject->getName(),fields[i]->getName(),sel->getCombo());
    }

    QBoxLayout *boxLayout = dynamic_cast<QBoxLayout *>(layout);
    if (boxLayout) {
        boxLayout->addStretch();
    }

    // Prevent mouse wheel from changing items
    disableMouseWheelEvents();
}
