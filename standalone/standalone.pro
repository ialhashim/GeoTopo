include($$[STARLAB])
include($$[SURFACEMESH])
include($$[NANOFLANN])
StarlabTemplate(none)

QMAKE_CXXFLAGS -= /MP

QT     += core gui
CONFIG += console

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = geotopCorrespond
TEMPLATE = app

SOURCES += main.cpp\
        mainwindow.cpp \
    ../DeformToFit.cpp \
    ../EnergyGuidedDeformation.cpp \
    ../EvaluateCorrespondence.cpp \
    ../PropagateProximity.cpp \
    ../PropagateSymmetry.cpp \
    ../BatchProcess.cpp \
    ../StructureAnalysis.cpp

HEADERS  += mainwindow.h \
    ../AStarSearch.h \
    ../DeformToFit.h \
    ../EncodeDecodeGeometry.h \
    ../EnergyGuidedDeformation.h \
    ../EvaluateCorrespondence.h \
    ../PropagateProximity.h \
    ../PropagateSymmetry.h \
    ../ShapeGraph.h \
    ../stlastar.h \
    ../BatchProcess.h \
    ../StructureAnalysis.h

FORMS    += mainwindow.ui

INCLUDEPATH += ..


CONFIG(debug, debug|release) {
    CFG = debug
} else {
    CFG = release
}

# NURBS library
LIBS += -L$$PWD/../../NURBS/$$CFG/lib -lNURBS
INCLUDEPATH += ../../NURBS

# StructureGraph library
LIBS += -L$$PWD/../../StructureGraphLib/$$CFG/lib -lStructureGraphLib
INCLUDEPATH += ../../StructureGraphLib
