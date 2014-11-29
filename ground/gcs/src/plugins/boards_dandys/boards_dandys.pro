TEMPLATE = lib
TARGET = Dandys
include(../../taulabsgcsplugin.pri)
include(../../plugins/uavobjects/uavobjects.pri)
include(../../plugins/coreplugin/coreplugin.pri)
include(../../plugins/uavobjectutil/uavobjectutil.pri)
include(../../plugins/uavobjectwidgetutils/uavobjectwidgetutils.pri)

OTHER_FILES += Dandys.pluginspec

CONFIG += console

INCLUDEPATH += ../../plugins/config/

HEADERS += \
    ../../plugins/config/hwfieldselector.h \
    dandysplugin.h \
    draco.h \
    dracoconfigurationwidget.h \
    osdfirmwarefile.h

SOURCES += \
    ../../plugins/config/hwfieldselector.cpp \
    dandysplugin.cpp \
    draco.cpp \
    dracoconfigurationwidget.cpp \
    osdfirmwarefile.cpp

RESOURCES += \
    dandys.qrc

FORMS += \
    ../../plugins/config/hwfieldselector.ui \
    dracoconfiguration.ui
