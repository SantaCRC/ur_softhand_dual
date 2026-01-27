TEMPLATE = lib
CONFIG += c++latest
CONFIG += staticlib
#DEFINES += MAKE_TEST_LIB
#VERSION = 1.0.0
TARGET = libserial

INCLUDEPATH = include
DEPENDPATH = include

SOURCES = src/serial.cpp

win32 {
    SOURCES += src/impl/impl_win.cpp \
            src/impl/list_ports/list_ports_win.cpp
    LIBS += -lsetupapi
}
unix {
    SOURCES += src/impl/impl_unix.cpp \
            src/impl/list_ports/list_ports_linux.cpp
}
