TEMPLATE = lib
TARGET = Dandys
include(../../taulabsgcsplugin.pri)
include(../../plugins/uavobjects/uavobjects.pri)
include(../../plugins/coreplugin/coreplugin.pri)

OTHER_FILES += Dandys.pluginspec

HEADERS += \
    dandysplugin.h \
    draco.h

SOURCES += \
    dandysplugin.cpp \
    draco.cpp

RESOURCES += \
    dandys.qrc
