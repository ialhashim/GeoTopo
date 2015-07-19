include($$[STARLAB])
include($$[SURFACEMESH])
StarlabTemplate(none)

QT     += core gui
CONFIG += console

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = GeoTopoCorrespond
TEMPLATE = app

SOURCES += main.cpp\
        mainwindow.cpp
HEADERS  += mainwindow.h

FORMS    += mainwindow.ui

INCLUDEPATH += ..

CONFIG(debug, debug|release) {
    CFG = debug
} else {
    CFG = release
}

# NURBS library
LIBS += -L$$PWD/../src/NURBS/$$CFG/lib -lNURBS
INCLUDEPATH += ../src/NURBS

# StructureGraph library
LIBS += -L$$PWD/../src/StructureGraphLib/$$CFG/lib -lStructureGraphLib
INCLUDEPATH += ../src/StructureGraphLib

# GeoTopo library
LIBS += -L$$PWD/../src/GeoTopoLib/$$CFG/lib -lGeoTopoLib
INCLUDEPATH += ../src/GeoTopoLib
