#pragma once

#include <QThread>
#include <QProgressDialog>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QVariantMap>

#include <QOpenGLWidget>
#include <QOpenGLFunctions>

namespace Structure{ struct ShapeGraph; }
namespace opengp{ namespace SurfaceMesh{class SurfaceMeshModel;} }

class RenderingWidget : public QOpenGLWidget, protected QOpenGLFunctions{
	Q_OBJECT
public:
	RenderingWidget(int width, QWidget * parent);
	Structure::ShapeGraph * cur_shape;
	QImage render(Structure::ShapeGraph * shape);
	QImage buffer;
protected:
	void initializeGL();
	void paintGL();
    opengp::SurfaceMesh::SurfaceMeshModel * model;
};

class BatchProcess : public QThread
{
	Q_OBJECT
public:
    BatchProcess(QString filename);
	BatchProcess(QString sourceFilename, QString targetFilename, QVariantMap options = QVariantMap());
	~BatchProcess();

	QSharedPointer<QProgressDialog> pd;
	QSharedPointer<RenderingWidget> renderer;
	int jobUID;

    typedef QVector< QPair<QStringList, QStringList> > EnergyAssignments;

	void init();
	void run();

	static void appendJob(QVariantMap job, QString filename);
	void exportJobFile(QString filename);

	double executeJob(QString sourceFile, QString targetFile, QJsonObject & job, 
        EnergyAssignments & assignments, QVariantMap & jobReport, int jobIdx);

	// Job properties:
	QString jobfilename;
	int resultsCount;
	QString outputPath;
	bool isDPsearch;
	bool isSwapped;
	bool isSaveReport;
	bool isKeepThread;
	bool isOutputMatching;
	bool isShowDeformed;
	bool isManyTypesJobs;
	bool isVisualize;
	int thumbWidth;
	QJsonArray jobsArray;

	int dpTopK, dpTopK_2;

    QVector<QVariantMap> jobReports;

    // Shapes loaded from memory
    QSharedPointer<Structure::ShapeGraph> cachedShapeA, cachedShapeB;

public:
    static void visualizeCorrespondence(QString sourceFile, QString targetFile, QVariantMap corr);

public slots:
	void setJobsArray(QJsonArray);

signals:
	void jobFinished(int);
	void allJobsFinished();
	void reportMessage(QString,double);
	void setLabelText(QString);
};

// Utility:
static inline void convertFromGLImage(QImage &img, int w, int h, bool alpha_format, bool include_alpha)
{
	if (QSysInfo::ByteOrder == QSysInfo::BigEndian) {
		// OpenGL gives RGBA; Qt wants ARGB
		uint *p = (uint*)img.bits();
		uint *end = p + w*h;
		if (alpha_format && include_alpha) {
			while (p < end) {
				uint a = *p << 24;
				*p = (*p >> 8) | a;
				p++;
			}
		}
		else {
			// This is an old legacy fix for PowerPC based Macs, which
			// we shouldn't remove
			while (p < end) {
				*p = 0xff000000 | (*p >> 8);
				++p;
			}
		}
	}
	else {
		// OpenGL gives ABGR (i.e. RGBA backwards); Qt wants ARGB
		for (int y = 0; y < h; y++) {
			uint *q = (uint*)img.scanLine(y);
			for (int x = 0; x < w; ++x) {
				const uint pixel = *q;
				if (alpha_format && include_alpha) {
					*q = ((pixel << 16) & 0xff0000) | ((pixel >> 16) & 0xff)
						| (pixel & 0xff00ff00);
				}
				else {
					*q = 0xff000000 | ((pixel << 16) & 0xff0000)
						| ((pixel >> 16) & 0xff) | (pixel & 0x00ff00);
				}

				q++;
			}
		}

	}
	img = img.mirrored();
}

