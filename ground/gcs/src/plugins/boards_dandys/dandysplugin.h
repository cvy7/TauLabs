/**
 ******************************************************************************
 *
 * @file       dandysplugin.h
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2014
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
#ifndef DANDYSPLUGIN_H
#define DANDYSPLUGIN_H

#include <extensionsystem/iplugin.h>

class DandysPlugin : public ExtensionSystem::IPlugin
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "TauLabs.plugins.Dandys" FILE "Dandys.json")

public:
   DandysPlugin();
   ~DandysPlugin();

   void extensionsInitialized();
   bool initialize(const QStringList & arguments, QString * errorString);
   void shutdown();

};

#endif // DANDYSPLUGIN_H
