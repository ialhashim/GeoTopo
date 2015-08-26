include($$PWD/../source/external/Eigen/Eigen.prf)# Eigen library
include($$PWD/../source/external/nanoflann/nanoflann.prf) # nanoflann library

QT     += core gui opengl xml
CONFIG += console

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG(debug, debug|release) {
    CFG = debug
} else {
    CFG = release
}

DESTDIR = ../../bin
TARGET = GeoTopoCorrespond
TEMPLATE = app

CONFIG(debug, debug|release) {
    TARGET = GeoTopoCorrespondDebug
}

SOURCES +=  main.cpp\
            mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui

INCLUDEPATH += ..

# GeoTopo library
LIBS += -L$$PWD/../source/GeoTopoLib/lib/$$CFG -lGeoTopoLib
INCLUDEPATH += ../source/GeoTopoLib

# StructureGraph library
LIBS += -L$$PWD/../source/StructureGraphLib/lib/$$CFG -lStructureGraphLib
INCLUDEPATH += ../source/StructureGraphLib

# Surface mesh library
LIBS += -L$$PWD/../source/external/SurfaceMesh/lib/$$CFG -lSurfaceMesh
INCLUDEPATH += ../source/external/SurfaceMesh ../source/external/SurfaceMesh/surface_mesh

# NURBS library
LIBS += -L$$PWD/../source/NURBS/lib/$$CFG -lNURBS
INCLUDEPATH += ../source/NURBS

linux-g++{ LIBS += -lGLU }

# Parallelism
win32{
    QMAKE_CXXFLAGS *= /openmp
    QMAKE_CXXFLAGS *= /MP
}
