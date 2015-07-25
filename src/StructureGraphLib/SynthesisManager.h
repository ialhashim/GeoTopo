#pragma once

#include <QObject>
#include <QMap>
#include <QStack>
#include <vector>

#include "StructureGraph.h"

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
class GraphCorresponder;
class Scheduler;
class TopoBlender;
typedef QMap<QString, QMap<QString, QVariant> > SynthData;

// Proxies
typedef std::vector< std::vector<double> > NodeProxy;
struct SimplePolygon{ 
	SimplePolygon(std::vector<Vector3> v = std::vector<Vector3>(), QColor c = QColor(0,0,0), bool isWireframe = false) : 
		vertices(v), c(c), isWireframe(isWireframe){} 
	std::vector<Vector3> vertices; QColor c; bool isWireframe;
};

extern QStack<double> nurbsQuality;

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

	PropertyMap property;

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
	void renderGraph( Structure::Graph graph, QString filename, bool isOutPointCloud, int reconLevel, bool isOutGraph = false, bool isOutParts = true );

	void drawSampled();
	void geometryMorph( SynthData & data, Structure::Graph * graph, bool isApprox, int limit = -1 );
	void drawSynthesis( Structure::Graph * activeGraph );

	void bufferCleanup();

    void makeProxies(int numSides = 10, int numSpineJoints = 10);
    std::vector<SimplePolygon> drawWithProxies(Structure::Graph * g);

	void emitSynthDataReady();

signals:
	void setMessage(QString);
	void progressChanged(double);
	void synthDataReady();
	void updateViewer();
};
