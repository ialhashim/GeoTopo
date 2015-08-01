include($$PWD/../source/external/Eigen/Eigen.prf)# Eigen library
include($$PWD/../source/external/nanoflann/nanoflann.prf) # nanoflann library

QT     += core gui opengl xml
CONFIG += console

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

DESTDIR = ../../bin
TARGET = GeoTopoCorrespond
TEMPLATE = app

SOURCES +=  main.cpp\
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
LIBS += -L$$PWD/../source/NURBS/$$CFG/lib -lNURBS
INCLUDEPATH += ../source/NURBS

# Surface mesh library
LIBS += -L$$PWD/../source/external/SurfaceMesh/$$CFG/lib -lSurfaceMesh
INCLUDEPATH += ../source/external/SurfaceMesh ../source/external/SurfaceMesh/surface_mesh

# StructureGraph library
LIBS += -L$$PWD/../source/StructureGraphLib/$$CFG/lib -lStructureGraphLib
INCLUDEPATH += ../source/StructureGraphLib

# GeoTopo library
LIBS += -L$$PWD/../source/GeoTopoLib/$$CFG/lib -lGeoTopoLib
INCLUDEPATH += ../source/GeoTopoLib
