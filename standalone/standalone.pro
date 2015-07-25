include($$PWD/../src/external/Eigen/Eigen.prf)# Eigen library
include($$PWD/../src/external/nanoflann/nanoflann.prf) # nanoflann library

QT     += core gui opengl xml
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

# Surface mesh library
LIBS += -L$$PWD/../src/external/SurfaceMesh/$$CFG/lib -lSurfaceMesh
INCLUDEPATH += ../src/external/SurfaceMesh ../src/external/SurfaceMesh/surface_mesh

# StructureGraph library
LIBS += -L$$PWD/../src/StructureGraphLib/$$CFG/lib -lStructureGraphLib
INCLUDEPATH += ../src/StructureGraphLib

# GeoTopo library
LIBS += -L$$PWD/../src/GeoTopoLib/$$CFG/lib -lGeoTopoLib
INCLUDEPATH += ../src/GeoTopoLib
