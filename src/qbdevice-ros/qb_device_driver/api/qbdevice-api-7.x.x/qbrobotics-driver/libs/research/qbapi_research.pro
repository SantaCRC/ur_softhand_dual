TEMPLATE = lib
CONFIG += c++latest
CONFIG += staticlib
VERSION = 7.0.0
TARGET = libqbapi-$$VERSION

#!!! Per compilare è necessario che questo repo (qbrobotics-driver-internal)
#    si trovi nella stessa cartella del repo serial (da cui pescare gli header)

INCLUDEPATH += include
INCLUDEPATH += ../../../serial/include # <-- header per le funzioni serial
DEPENDPATH += include

SOURCES  = src/qbrobotics_research_api.cpp
SOURCES += src/qbmove_research_api.cpp
SOURCES += src/qbsofthand_research_api.cpp

win32 {
    LIBS += -lsetupapi
}
