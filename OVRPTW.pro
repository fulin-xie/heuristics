TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

CONFIG += c++11

SOURCES += main.cpp \
    dataClass.cpp \
    tabusearch.cpp \
    localSearch.cpp

HEADERS += \
    dataClass.h \
    tabusearch.h \
    tabuAttribute.h \
    localSearch.h

