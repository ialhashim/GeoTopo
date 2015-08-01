#pragma once
#include "StructureGraph.h"
#include "GraphCorresponder.h"

#include "SoftwareRenderer.h"
#include "MarchingSquares.h"

#include "glhelper.h"

// Triangulation library
#include "polypartition.hpp"

/* Simplification library
#include "psimpl.h"
inline Array1D_Vector3 simplify( Array1D_Vector3 & original_curve, int count )
{
	Array1D_Vector3 simplified_curve;
	std::vector<double> curve, simple_curve;
	for(auto p : original_curve) for(int i = 0 ; i < 3; i++) curve.push_back( p[i] );
	psimpl::simplify_douglas_peucker_n<3>( curve.begin(), curve.end(), count, std::back_inserter(simple_curve) );
	for(size_t i = 0; i < simple_curve.size() / 3; i++)
		simplified_curve.push_back( Vector3(simple_curve[(i*3)+0], simple_curve[(i*3)+1], simple_curve[(i*3)+2]) );
	return simplified_curve;
}*/

struct NodeProjection{
	Array2D_Vector3 skeletonFrames;
	Array1D_Vector3 boundary;
};

inline bool rayLineIntersect2D(Eigen::Vector3d rayOrigin, Eigen::Vector3d rayDirection, 
							 Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d & ptIntersection)
{
	Vector3d rayEnd = rayOrigin + rayDirection * std::max((rayOrigin - p1).norm(), (rayOrigin - p2).norm()) * 2;
	double L1X1 = rayOrigin[0], L1Y1 = rayOrigin[1], L1X2 = rayEnd[0], L1Y2 = rayEnd[1];
	double L2X1 = p1[0], L2Y1 = p1[1], L2X2 = p2[0], L2Y2 = p2[1];

	// Denominator for ua and ub are the same, so store this calculation
	double d = (L2Y2 - L2Y1) * (L1X2 - L1X1)-(L2X2 - L2X1) * (L1Y2 - L1Y1);

	//n_a and n_b are calculated as separate values for readability
	double n_a =(L2X2 - L2X1) * (L1Y1 - L2Y1)-(L2Y2 - L2Y1) * (L1X1 - L2X1);
	double n_b =(L1X2 - L1X1) * (L1Y1 - L2Y1)-(L1Y2 - L1Y1) * (L1X1 - L2X1);

	if (d == 0) return false;
	double ua = n_a / d, ub = n_b / d;
	if (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1){
		ptIntersection[0] = L1X1 + (ua * (L1X2 - L1X1));
		ptIntersection[1] = L1Y1 + (ua * (L1Y2 - L1Y1));
		ptIntersection[2] = 0;
		return true;
	}
	return false;
}

inline bool closestIntersectionPolygon(Eigen::Vector3d rayOrigin, Eigen::Vector3d rayDirection, Array1D_Vector3 boundary, Vector3d & best_isect)
{
	bool isIntersect = false;
	double minDist = DBL_MAX;
	for(size_t b = 0; b + 1 < boundary.size(); b++){
		Vector3d isect(0,0,0);
		if( rayLineIntersect2D(rayOrigin, rayDirection, boundary[b], boundary[b+1], isect)){
			double dist = (rayOrigin - isect).norm();
			if(dist < minDist){
				best_isect = isect;
				minDist = dist;
				isIntersect = true;
			}
		}
	}
	return isIntersect;
}

inline Eigen::Vector3d closestPoint( Eigen::Vector3d point, Array1D_Vector3 pnts ){
	Eigen::Vector3d closest = pnts.front();
	double minDist = DBL_MAX;
	for(auto p : pnts){
		double dist = (p-point).norm();
		if(dist < minDist){
			closest = p;
			minDist = dist;
		}
	}
	return closest;
}

inline bool pnpoly(Eigen::Vector3d test, Array1D_Vector3 verts)
{
	double testx = test[0], testy = test[1];
	int nvert = (int)verts.size();
	int i, j, c = 0;
	for (i = 0, j = nvert-1; i < nvert; j = i++) {
		if ( ((verts[i][1]>testy) != (verts[j][1]>testy)) 
			&& (testx < (verts[j][0]-verts[i][0]) * (testy-verts[i][1]) / (verts[j][1]-verts[i][1]) + verts[i][0]) )
			c = !c;
	}
	return c;
}

class ProjectedStructureGraph
{
public:
	ProjectedStructureGraph(Structure::Graph *inputGraph, int screenWidth, bool isUseSkeleton = true, bool isVisaulize = false) 
		: graph(inputGraph), screenWidth(screenWidth), isBuildSkeletonFrames( isUseSkeleton ), isVisualDebug( isVisaulize )
	{
		if(!inputGraph) return;

		// Debug
		if( isVisualDebug )
		{
			debugImage = QImage(screenWidth, screenWidth, QImage::Format_ARGB32_Premultiplied);
			debugImage.fill(qRgba(255,255,255,255));
		}

		// Setup projection and camera
		updateCamera();

		// Encode projected geometries
		viewport = Eigen::Vector4i(0, 0, screenWidth, screenWidth);
		mvp = projectionMatrix * cameraMatrix;

		// Encode
		projectNodes();
	}
	
    Structure::Graph * graph;
	bool isBuildSkeletonFrames;

	// Projection parameters
	int screenWidth;
    Eigen::Matrix4d projectionMatrix, cameraMatrix;
	Eigen::Matrix4d mvp;
	Eigen::Vector4i viewport;

	// Debug
	QImage debugImage;
	bool isVisualDebug;

	// Node projections
	QMap<QString, NodeProjection> projections;
	void projectNodes()
	{
		// Re-sample skeleton to 'N' points
		int N = 20;

		projections.clear();

		for(auto n : graph->nodes)
		{
			Array2D_Vector3 skeletonFrames;
			Array1D_Vector3 boundary;
			Array1D_Vector3 skeletonPoints;

			// Render geometry as one solid blob to a 'buffer'
			{
				QVector< QVector<Vector3> > triangles;
				auto nodeMesh = n->property["mesh"].value< QSharedPointer<SurfaceMeshModel> >();
				for(auto f : nodeMesh->faces()){
					QVector<Vector3> face;
					for(auto v : nodeMesh->vertices(f)){
						Vector3 p = nodeMesh->vertex_coordinates()[v];
						p = (mvp * p.homogeneous()).colwise().hnormalized();
						p[0] = viewport[0] + viewport[2] * (p[0]+1)/2;
						p[1] = viewport[1] + viewport[3] * (-p[1]+1)/2; // notice '-y' to get from top left
						face.push_back( p );
					}
					triangles.push_back( face );
				}

				MatrixXd buffer = SoftwareRenderer::renderTriangles2D(triangles, viewport[2], viewport[3]);

				// If we use skeletons, make sure we thicker boundary
				if( isBuildSkeletonFrames )
				{
					// Sample node skeleton
					double res = n->bbox().sizes().norm() * 0.1;
					skeletonPoints = n->discretizedAsCurve( res );

					// Project skeleton to screen space
					for(auto & p : skeletonPoints)
					{
						p = (mvp * p.homogeneous()).colwise().hnormalized();
						p[0] = viewport[0] + viewport[2] * (p[0]+1)/2;
						p[1] = viewport[1] + viewport[3] * (-p[1]+1)/2; // notice '-y' to get from top left
						p[2] = p[2];
					}

					NURBS::NURBSCurved::createCurveFromPoints(skeletonPoints).SubdivideByLength(N, skeletonPoints);

					for(size_t i = 0; i + 1 < skeletonPoints.size(); i++)
					{
						Vector3 p0 = skeletonPoints[i], p1 = skeletonPoints[i+1];

						Array1D_Vector3 frame(3, Vector3(0,0,0));
						frame[0] = p0; // position
						frame[1] = (p1-p0).normalized(); // tangent
						frame[2] = rotatedVec(frame[1], M_PI_2, Vector3(0,0,1)); // normal

						skeletonFrames.push_back( frame );

						// Last frame
						if(i + 2 == skeletonPoints.size()){
							skeletonFrames.push_back(frame);
							skeletonFrames.back().front() = skeletonPoints.back();
						}

						SoftwareRenderer::DrawBlineThick(buffer, p0, p1, 1.0, 2);
					}
				}

				//showImage(SoftwareRenderer::matrixToImage(buffer));

				for(auto p : MarchingSquares::march(buffer, 1.0))
					boundary.push_back( Vector3(p.x(), p.y(), 0) );
			}

			// Build skeleton frames
			if( isBuildSkeletonFrames )
			{
				if( isVisualDebug )
				{
					QPainter painter(&debugImage);
					for(auto frame : skeletonFrames)
					{
						double s = 10;
						drawArrow( frame[0], frame[0] + frame[1] * s, Qt::red, painter );
						drawArrow( frame[0], frame[0] + frame[2] * s, Qt::green, painter );
					}
				}

				// Sampled boundary
				Array1D_Vector3 sampledBoundary;

				for(size_t r = 0; r + 1 < (skeletonFrames.size() * 2); r++)
				{
					int steps = N / 4;

					// At end points
					if(r == 0 || r == (skeletonFrames.size()-1))
					{
						Vector3d rayOrigin = skeletonFrames[r][0];
						Vector3d rayDirection = skeletonFrames[r].back().normalized();
						if(r == 0) rayDirection *= -1;

						double delta = M_PI / (steps * 3);

						for(double theta = 0; theta <= M_PI; theta += delta)
						{
							Vector3d isect;
							bool isIntersect = closestIntersectionPolygon(rayOrigin, rayDirection, boundary, isect);
							
							// Hack to fix edge case, highly twisted skeleton
							{
								Vector3d isect_other;
								if(closestIntersectionPolygon(rayOrigin + rayDirection * 1e-5, rayDirection, skeletonPoints, isect_other))
								{
									isect = closestPoint(rayOrigin, boundary);
									isIntersect = false;
								}
							}

							sampledBoundary.push_back( isect );

							if( !isIntersect && isVisualDebug )
							{
								QPainter painter(&debugImage);
								drawArrow( rayOrigin, isect, Qt::blue, painter, 10 );
							}

							rayDirection = rotatedVec( rayDirection, -delta, Vector3d(0,0,1) );
						}
					}
					
					// Mid-points
					if(r != (skeletonFrames.size()-1))
					{
						bool isOtherway = r > ((skeletonFrames.size()-1));

						int idx = (int)r;
						if( isOtherway ) idx = (int) ((skeletonFrames.size()-1) - (r % skeletonFrames.size()));
						int idxNext = idx + (isOtherway ? -1 : 1);

						Vector3 start = skeletonFrames[idx][0];
						Vector3 end = skeletonFrames[idxNext][0];

						double flip = (r < skeletonFrames.size()) ? 1 : -1;

						Vector3 rayDirection = skeletonFrames[isOtherway ? idxNext : idx][2] * flip;
						
						for(double alpha = 0; alpha < 1.0; alpha += 1.0 / steps)
						{
							if(r == 0 && alpha == 0) continue;

							Vector3 rayOrigin = AlphaBlend( alpha, start, end );

							Vector3d isect;
							bool isIntersect = closestIntersectionPolygon(rayOrigin, rayDirection, boundary, isect);

							if( !isIntersect && isVisualDebug )
							{
								QPainter painter(&debugImage);
								drawArrow( rayOrigin, isect, Qt::blue, painter, 6 );
							}

							sampledBoundary.push_back( isect );
						}
					}
				}

				boundary = sampledBoundary;
			}

			// Experiment: fixed number points + top corner start
			if( !isBuildSkeletonFrames )
			{
				boundary = refineByNumber(boundary, 300);

				size_t closestStart = 0;
				double minDist = DBL_MAX;

				for(size_t i = 0; i < boundary.size(); i++){
					double dist = boundary[i].norm();
					if(dist < minDist){
						minDist = dist;
						closestStart = i;
					}
				}

				std::rotate(boundary.begin(), boundary.begin() + closestStart, boundary.end());
			}

			// Fix orientation if needed
			{
				TPPLPoly poly; poly.Init((long)boundary.size());
				for (int i = 0; i < (int)boundary.size(); i++){
					poly[i].x = boundary[i].x(); poly[i].y = boundary[i].y();
				}
				if (poly.GetOrientation() == TPPL_CW)
					std::reverse(boundary.begin(), boundary.end());
			}

			if( isVisualDebug )
			{
				QPainter painter(&debugImage);
				QPainterPath path;
				for(size_t i = 0; i < boundary.size(); i++){
					Vector3 p = boundary[i];
					if(i == 0) path.moveTo(p.x(), p.y());
					else path.lineTo(p.x(), p.y());
				}
				path.lineTo(boundary.front().x(),boundary.front().y());
				painter.drawPath( path );
			}

			// Record
			NodeProjection nproj;
			nproj.boundary = boundary;
			nproj.skeletonFrames = skeletonFrames;
			projections[n->id] = nproj;
		}

		if(isVisualDebug) showImage(debugImage);
	}

	void updateCamera(double rotationAngle = M_PI * 1.3)
	{
		Eigen::Vector3d center, eye;
		Eigen::Vector3d dir( cos(rotationAngle), sin(rotationAngle), 0.25);
		Eigen::AlignedBox3d bbox = graph->bbox();
		center = bbox.center();
    
		//double radius = bbox.sizes().norm() * 3;
		double radius = 4.5; // Fixed

		eye = center + (dir.normalized() * radius);
		projectionMatrix = perspective<double>(20, 1.0, 0.01, 1000);
		cameraMatrix = lookAt< Eigen::Vector3d >(eye, center, Eigen::Vector3d(0,0,1));
	}

	void drawBlended(ProjectedStructureGraph * pgOther, GraphCorresponder * gcorr, double alpha)
	{
		if(!graph || graph->nodes.empty() || !gcorr || !pgOther) return;

		QColor fillColor(0,0,0);
		QColor borderColor(0,0,0);
		bool isDrawBorder = false;
		bool isTriangulate = false;

		glDisable(GL_LIGHTING);
		glLineWidth( 2.0 );

		glDisable(GL_DEPTH_TEST);

		// 2D mode
		{
			Eigen::Vector4i viewport;
			glGetIntegerv(GL_VIEWPORT, viewport.data());
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			glOrtho(0.0f, viewport[2], viewport[3], 0.0f, 0.0f, 1.0f);
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
		}

		for(auto node : graph->nodes)
		{
			std::vector<QString> correspond = gcorr->correspondingNodesTarget(node->id);
			if(correspond.empty()) continue;
			if(!projections.contains(node->id)) continue;

			for(QString tid : correspond)
			{
				Array1D_Vector3 src = projections[node->id].boundary;
				Array1D_Vector3 tgt = pgOther->projections[tid].boundary;

				Array1D_Vector3 blended;
				for(size_t i = 0; i < src.size(); i++)
				{
					Vector3 p = AlphaBlend( qRanged(0.0, alpha, 1.0) , src[i], tgt[i]);
					blended.push_back(p);
				}

				if( isTriangulate )
				{
					TPPLPoly poly; poly.Init((long)blended.size());
					for(int i = 0; i < (int)blended.size(); i++){
						poly[i].x = blended[i].x();
						poly[i].y = blended[i].y();
					}
					TPPLPartition pp;
					std::list<TPPLPoly> inpolys, outpolys;
					inpolys.push_back(poly);

					pp.Triangulate_MONO(&inpolys, &outpolys);

					glColorQt(QColor(200,0,200));
					glBegin(GL_TRIANGLES);
					for(auto poly : outpolys)
						for(int i = 0; i < poly.GetNumPoints(); i++)
							glVertex2d(poly[i].x, poly[i].y);
					glEnd();
				}
				else
				{
					glClear(GL_STENCIL_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

					glEnable(GL_STENCIL_TEST);
					glColorMask(GL_FALSE,GL_FALSE,GL_FALSE,GL_FALSE);
					glDepthMask(GL_FALSE);
					glEnable(GL_STENCIL_TEST);
					glStencilFunc(GL_ALWAYS,0x1,0x1);
					glStencilOp(GL_KEEP,GL_KEEP,GL_INVERT);
					glBegin(GL_TRIANGLE_FAN);
					for(size_t i = 0; i < blended.size(); i++)
					{
						Vector3 p = blended[i];
						glVector3(p);
					}
					glEnd();


					glColorQt( fillColor );

					glDepthMask(GL_TRUE);
					glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);
					glStencilFunc(GL_EQUAL,0x1,0x1);
					glStencilOp(GL_KEEP,GL_KEEP,GL_KEEP);
					glBegin(GL_TRIANGLE_FAN);
					for(size_t i = 0; i < blended.size(); i++){
						Vector3 p = blended[i];
						glVector3(p);
					}
					glEnd();
			
					glDisable(GL_STENCIL_TEST);
				}

				// Border
				if( isDrawBorder )
				{
					glColorQt( borderColor );
					glBegin(GL_LINE_LOOP);
					for(size_t i = 0; i < blended.size(); i++)
					{
						Vector3 p = blended[i];
						glVector3(p);
					}
					glEnd();
				}

			}
		}
		glEnable(GL_DEPTH_TEST);
	}

	QImage drawBlendedImage(ProjectedStructureGraph * pgOther, GraphCorresponder * gcorr, double alpha)
	{
		QImage img(screenWidth, screenWidth, QImage::Format_RGB888);
		img.fill( qRgba(255,255,255,255) ); // white background
		if(!graph || !pgOther || !pgOther->graph || !gcorr) return img;

		// Draw blended boundaries
		{
			QPainter painter( &img );
			painter.setBrush(QBrush(Qt::black));
			painter.setPen(Qt::NoPen);

			for(auto node : graph->nodes)
			{
				std::vector<QString> correspond = gcorr->correspondingNodesTarget(node->id);
				if(correspond.empty()) continue;
				if(!projections.contains(node->id)) continue;

				for(QString tid : correspond)
				{
					if(!projections.contains(node->id)) continue;
					Array1D_Vector3 src = projections.value(node->id).boundary;
					Array1D_Vector3 tgt = pgOther->projections.value(tid).boundary;

					QPainterPath path;

					for(size_t i = 0; i < src.size(); i++)
					{
						Vector3 p = AlphaBlend( qRanged(0.0, alpha, 1.0) , src[i], tgt[i]);
						if(i == 0) 
							path.moveTo(p.x(), p.y());
						else
							path.lineTo(p.x(), p.y());
					}

					painter.drawPath( path );
				}
			}
		}

		return img;
	}

	QImage drawBoundaryImage()
	{
		QImage img(screenWidth, screenWidth, QImage::Format_RGB888);
		img.fill( qRgba(255,255,255,255) ); // white background
		if(!graph) return img;

		QPainter painter( &img );
		painter.setBrush(QBrush(Qt::black));
		painter.setPen(Qt::NoPen);

		for(auto node : graph->nodes)
		{
			if(!projections.contains(node->id)) continue;
			Array1D_Vector3 src = projections.value(node->id).boundary;
			
			QPainterPath path;

			for(size_t i = 0; i < src.size(); i++)
			{
				Vector3 p = src[i];

				if(i == 0) path.moveTo(p.x(), p.y());
				else path.lineTo(p.x(), p.y());
			}

			painter.drawPath( path );
		}

		return img;
	}
};
