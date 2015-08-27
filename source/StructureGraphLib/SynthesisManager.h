#pragma once

#include <QObject>
#include <QMap>
#include <QStack>
#include <QColor>
#include <vector>

#ifdef SPLAT_RENDERING
#include "../GlSplatRendererLib/GLVertex.h"
#else
struct GLVertex{
    float x, y, z;
    float nx, ny, nz;
    GLVertex(float X=0, float Y=0, float Z=0, float nX=0, float nY=0, float nZ=0) : x(X), y(Y), z(Z), nx(nX), ny(nY), nz(nZ) {}
};
#endif

// Forward declare
namespace Structure{ struct Graph; struct Node; }
class GraphCorresponder;
class Scheduler;
class TopoBlender;
typedef QMap<QString, QMap<QString, QVariant> > SynthData;

// Proxies
typedef std::vector< std::vector<double> > NodeProxy;

#include <Eigen/Core>
struct SimplePolygon{
    SimplePolygon(std::vector<Eigen::Vector3d> v = std::vector<Eigen::Vector3d>(), QColor c = QColor(0,0,0), bool isWireframe = false) :
        vertices(v), c(c), isWireframe(isWireframe){}
    std::vector<Eigen::Vector3d> vertices; QColor c; bool isWireframe;
};

extern QStack<double> nurbsQuality;

#include "Curve.h"
static void inline beginFastNURBS(){
    nurbsQuality.clear();
    nurbsQuality.push(TIME_ITERATIONS);
    nurbsQuality.push(CURVE_TOLERANCE);
    nurbsQuality.push(RombergIntegralOrder);

    TIME_ITERATIONS			= 6;
    CURVE_TOLERANCE			= 1e-05;
    RombergIntegralOrder	= 5;
}

static void inline endFastNURBS(){
    if(nurbsQuality.size() < 3) return;
    RombergIntegralOrder = nurbsQuality.pop();
    CURVE_TOLERANCE = nurbsQuality.pop();
    TIME_ITERATIONS = nurbsQuality.pop();
}

class SynthesisManager : public QObject
{
    Q_OBJECT
public:
    SynthesisManager(GraphCorresponder * gcorr=nullptr, Scheduler * scheduler=nullptr,
                     TopoBlender * blender=nullptr, int samplesCount = 20000);
    
    GraphCorresponder * gcorr;
    Scheduler * scheduler;
    TopoBlender * blender;

    QVariantMap property;

    QVector<Structure::Graph*> graphs();
    Structure::Graph * graphNamed(QString graphName);

    // Synthesis data [graph][node][data]
    QMap<QString, QMap<QString, QMap<QString,QVariant> > > synthData;
    SynthData renderData;
    int samplesCount;

    std::vector<GLVertex> vertices;
    SynthData currentData;
    QMap<QString, QVariant> currentGraph;

    // Visualization
    QMap<QString, QMap<QString,QVariant> > sampled;

    // Proxies
    QMap<QString, QMap<QString,NodeProxy> > proxies;
    QMap<QString, QVariant> proxyOptions;
    static Array2D_Vector3 proxyRays( Array1D_Vector3 spineJoints, Array1D_Vector3 spineNormals, int numSides = 10 );

    // Options
    bool isSplatRenderer;
    double splatSize;
    float pointSize;
    QColor color;

    bool samplesAvailable(QString graph, QString nodeID);

    std::vector<SimplePolygon> drawWithProxies(Structure::Graph * g);

    typedef QPair< QVector<Eigen::Vector3f>,QVector<Eigen::Vector3f> > OrientedCloud;
    OrientedCloud reconstructGeometryNode(Structure::Node *n, double t);

public slots:
    void generateSynthesisData();
    void setSampleCount(int numSamples);
    void saveSynthesisData(QString parentFolder = "");
    void loadSynthesisData(QString parentFolder = "");
    void genSynData();
    void reconstructXYZ();
    void outputXYZ();
    void clear();

    void doRenderAll();
    void renderAll();
    void renderCurrent();
    void renderCurrent( Structure::Graph * currentGraph, QString path = "" );
    void renderGraph( Structure::Graph & graph, QString filename, bool isOutPointCloud, int reconLevel, bool isOutGraph = false, bool isOutParts = true );

    void drawSampled();
    void geometryMorph( SynthData & data, Structure::Graph * graph, bool isApprox, int limit = -1 );
    void drawSynthesis( Structure::Graph * activeGraph );

    void bufferCleanup();

    void makeProxies(int numSides = 10, int numSpineJoints = 10);

    void emitSynthDataReady();

signals:
    void setMessage(QString);
    void progressChanged(double);
    void synthDataReady();
    void updateViewer();
};
