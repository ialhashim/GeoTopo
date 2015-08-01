# Eigen library
include($$PWD/../external/Eigen/Eigen.prf)

# Warnings
win32{QMAKE_CXXFLAGS *= /wd4800 /wd4244 /wd4267}

TEMPLATE = lib
CONFIG += staticlib
QT += opengl

# Build flag
CONFIG(debug, debug|release) {CFG = debug} else {CFG = release}

# Library name and destination
TARGET = NURBS
DESTDIR = $$PWD/lib/$$CFG

SOURCES += \
    ParametricSurface.cpp \
    NURBSRectangle.cpp \
    NURBSCurve.cpp \
    BSplineRectangle.cpp \
    BSplineCurve.cpp \
    SingleCurve.cpp \
    Surface.cpp \
    Curve.cpp \
    BSplineBasis.cpp \
    LineSegment.cpp \
    NurbsDraw.cpp

HEADERS += \
    ParametricSurface.h \
    NURBSRectangle.h \
    NURBSCurve.h \
    BSplineRectangle.h \
    BSplineCurve.h \
    SingleCurve.h \
    Surface.h \
    NURBSGlobal.h \
    Curve.h \
    BSplineBasis.h \
    Integrate1.h \
    LineSegment.h \
    NurbsDraw.h
