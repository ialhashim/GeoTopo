# Eigen library
include($$PWD/../Eigen/Eigen.prf)

# Warnings
win32{QMAKE_CXXFLAGS *= /wd4800 /wd4244 /wd4267}

TEMPLATE = lib
CONFIG += staticlib
QT += opengl

# Build flag
CONFIG(debug, debug|release) {
    CFG = debug
} else {
    CFG = release
}

INCLUDEPATH *= surface_mesh

# Library name and destination
TARGET = SurfaceMesh
DESTDIR = $$PWD/$$CFG/lib

SOURCES += SurfaceMeshModel.cpp
HEADERS += SurfaceMeshModel.h
