#pragma once
#include <iostream>     // std::cout
#include <QImage>
#include <QPainter>
#include <QPoint>

#include <Eigen/Core>
#include <Eigen/Geometry>

#define RADIANS(deg)    ((deg)/180.0 * M_PI)
#ifndef M_PI_2
#define M_PI    3.14159265358979323846264338328
#define M_PI_2  1.57079632679489661923132169164
#endif
#define cot(x) (tan(M_PI_2 - x))

typedef QPair<Eigen::Vector3d, Eigen::Vector3d> PairPoints;
typedef Eigen::Matrix<double,4,4,Eigen::RowMajor> Matrix4;

//double angle = 0.0;

namespace SoftwareRenderer{

	struct DepthCompare{
		bool operator()(const Eigen::Vector3d & a, const Eigen::Vector3d & b) const{
			return a[2] < b[2];
		}
	};

	inline void horizontalLine(Eigen::MatrixXd & m,  int xpos, int ypos, int x1, double color){
		for(int x = xpos; x <= x1; ++x){
			if(x < 0 || x > m.cols() - 1 || ypos < 0 || ypos > m.rows() - 1) 
				continue;
			m(ypos,x) = color;
		}
	}

	inline void plot4points(Eigen::MatrixXd & buffer, int cx, int cy, int x, int y, double color){
		horizontalLine(buffer, cx - x, cy + y, cx + x, color);
		if (x != 0 && y != 0)
			horizontalLine(buffer, cx - x, cy - y, cx + x, color);
	}

	inline void circle(Eigen::MatrixXd & buffer, int cx, int cy, int radius, double color){
		int error = -radius;
		int x = radius;
		int y = 0;

		while (x >= y){
			int lastY = y;

			error += y;
			++y;
			error += y;

			plot4points(buffer, cx, cy, x, lastY, color);

			if (error >= 0){
				if (x != lastY)
					plot4points(buffer, cx, cy, lastY, x, color);
				error -= x;
				--x;
				error -= x;
			}
		}
	}

	inline void DrawBline(Eigen::MatrixXd & m, Eigen::Vector3d point0, Eigen::Vector3d point1, double color)
	{
		int x0 = (int)point0.x();
		int y0 = (int)point0.y();
		int x1 = (int)point1.x();
		int y1 = (int)point1.y();

		auto dx = std::abs(x1 - x0);
		auto dy = std::abs(y1 - y0);
		auto sx = (x0 < x1) ? 1 : -1;
		auto sy = (y0 < y1) ? 1 : -1;
		auto err = dx - dy;

		while (true) {
			if(x0 < 0 || x0 > m.cols() - 1 || y0 < 0 || y0 > m.rows() - 1) {} else m(y0, x0) = color;

			if ((x0 == x1) && (y0 == y1)) break;
			auto e2 = 2 * err;
			if (e2 > -dy) { err -= dy; x0 += sx; }
			if (e2 < dx) { err += dx; y0 += sy; }
		}
	}

	inline void DrawBlineThick(Eigen::MatrixXd & m, Eigen::Vector3d point0, Eigen::Vector3d point1, double color, int thickness = 3)
	{
		int x0 = (int)point0.x();
		int y0 = (int)point0.y();
		int x1 = (int)point1.x();
		int y1 = (int)point1.y();

		auto dx = std::abs(x1 - x0);
		auto dy = std::abs(y1 - y0);
		auto sx = (x0 < x1) ? 1 : -1;
		auto sy = (y0 < y1) ? 1 : -1;
		auto err = dx - dy;

		while (true) {
			if(x0 < 0 || x0 > m.cols() - 1 || y0 < 0 || y0 > m.rows() - 1) {} 
			else 
			{
				for(int i = -thickness; i <= thickness; i++)
				{	
					for(int j = -thickness; j <= thickness; j++)
					{
						int xx0 = x0 + j; 
						int yy0 = y0 + i;
						if(xx0 < 0 || xx0 > m.cols() - 1 || yy0 < 0 || yy0 > m.rows() - 1){}
						else m(yy0, xx0) = color;
					}
				}
			}

			if ((x0 == x1) && (y0 == y1)) break;
			auto e2 = 2 * err;
			if (e2 > -dy) { err -= dy; x0 += sx; }
			if (e2 < dx) { err += dx; y0 += sy; }
		}
	}

	// Clamping values to keep them between 0 and 1
	inline double Clamp(double value, double min = 0, double max = 1){
		return std::max(min, std::min(value, max));
	}

	// Interpolating the value between 2 vertices 
	// min is the starting point, max the ending point
	// and gradient the % between the 2 points
	inline double Interpolate(double min, double max, double gradient){
		return min + (max - min) * Clamp(gradient);
	}

	// drawing line between 2 points from left to right
	// papb -> pcpd
	// pa, pb, pc, pd must then be sorted before
	inline void ProcessScanLine(Eigen::MatrixXd & m, int y, Vector3 pa, Vector3 pb, Vector3 pc, Vector3 pd, double color)	{
		// Thanks to current Y, we can compute the gradient to compute others values like
		// the starting X (sx) and ending X (ex) to draw between
		// if pa.Y == pb.Y or pc.Y == pd.Y, gradient is forced to 1
		auto gradient1 = pa.y() != pb.y() ? (y - pa.y()) / (pb.y() - pa.y()) : 1;
		auto gradient2 = pc.y() != pd.y() ? (y - pc.y()) / (pd.y() - pc.y()) : 1;

		int sx = (int)Interpolate(pa.x(), pb.x(), gradient1);
		int ex = (int)Interpolate(pc.x(), pd.x(), gradient2);

		// drawing a line from left (sx) to right (ex) 
		for (auto x = sx; x < ex; x++)
		{
			if(x < 0 || x > m.cols() - 1 || y < 0 || y > m.rows() - 1) 
				continue;
			m(y,x) = color;
		}
	}

	inline void drawTriangle(Eigen::MatrixXd & buffer, Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3, double color)
	{
		// Sorting the points in order to always have this order on screen p1, p2 & p3
		// with p1 always up (thus having the Y the lowest possible to be near the top screen)
		// then p2 between p1 & p3
		if (p1.y() > p2.y()){
			auto temp = p2;
			p2 = p1;
			p1 = temp;
		}

		if (p2.y() > p3.y()){
			auto temp = p2;
			p2 = p3;
			p3 = temp;
		}

		if (p1.y() > p2.y()){
			auto temp = p2;
			p2 = p1;
			p1 = temp;
		}

		// inverse slopes
		double dP1P2, dP1P3;

		if (p2.y() - p1.y() > 0) dP1P2 = (p2.x() - p1.x()) / (p2.y() - p1.y());
		else dP1P2 = 0;

		if (p3.y() - p1.y() > 0) dP1P3 = (p3.x() - p1.x()) / (p3.y() - p1.y());
		else dP1P3 = 0;

		if (dP1P2 > dP1P3){
			for (auto y = (int)p1.y(); y <= (int)p3.y(); y++){
				if (y < p2.y())
					ProcessScanLine(buffer, y, p1, p3, p1, p2, color);
				else
					ProcessScanLine(buffer, y, p1, p3, p2, p3, color);
			}
		}
		else{
			for (auto y = (int)p1.y(); y <= (int)p3.y(); y++){
				if (y < p2.y())
					ProcessScanLine(buffer, y, p1, p2, p1, p3, color);
				else
					ProcessScanLine(buffer, y, p2, p3, p1, p3, color);
			}
		}
	}

	inline Eigen::Vector3d TransformCoordinates(const Eigen::Vector3d & vector, const Matrix4 & m) {
		double x = (vector[0] * m(0)) + (vector[1] * m(4)) + (vector[2] * m(8)) + m(12);
		double y = (vector[0] * m(1)) + (vector[1] * m(5)) + (vector[2] * m(9)) + m(13);
		double z = (vector[0] * m(2)) + (vector[1] * m(6)) + (vector[2] * m(10)) + m(14);
		double w = (vector[0] * m(3)) + (vector[1] * m(7)) + (vector[2] * m(11)) + m(15);
		return Eigen::Vector3d(x / w, y / w, z / w);
	};

	inline Matrix4 CreateProjectionMatrix(double fov_degrees, double aspect_ratio, double zNear = 0.1, double zFar = 10.0)
	{
		double yScale = cot( RADIANS(fov_degrees * 0.5) );
		double xScale = yScale/aspect_ratio;
		Eigen::Matrix4d pmat;
		pmat << -xScale, 0,							0,	0,
				0, yScale,							0,	0,
				0,		0,		zFar/(zFar-zNear)	,	1,
				0,		0, -zFar*zNear/(zFar-zNear)	,	0;

		return pmat;
	}

	inline Matrix4 CreateWorldMatrix(double transX = 0, double transY = 0, double transZ = 0)
	{
		Matrix4 wmat = Matrix4::Identity();
		wmat.row(3) = Eigen::Vector4d(transX, transY, transZ, 1);
		return wmat;
	}

	inline Matrix4 CreateViewMatrix( Eigen::Vector3d eye = Eigen::Vector3d(3,-3,3), Eigen::Vector3d target = Eigen::Vector3d(0,0,0), Eigen::Vector3d up = Eigen::Vector3d(0,0.0,1.0)  )
	{
		Matrix4 vmat = Matrix4::Identity();

		Eigen::Vector3d zAxis = (target - eye);			zAxis.normalize();
		Eigen::Vector3d xAxis = (up.cross(zAxis));		xAxis.normalize();
		Eigen::Vector3d yAxis = (zAxis.cross(xAxis));	yAxis.normalize();
		double ex = -(xAxis.dot(eye));
		double ey = -(yAxis.dot(eye));
		double ez = -(zAxis.dot(eye));

		vmat << xAxis[0], xAxis[1], xAxis[2],  ex,
				yAxis[0], yAxis[1], yAxis[2],  ey,
				zAxis[0], zAxis[1], zAxis[2],  ez,
				0		, 0		  , 0		,  1;

		vmat = Matrix4( vmat.transpose() );

		return vmat;
	}

	inline Eigen::Vector3d Project(const Eigen::Vector3d & coord, const Matrix4 & transMat, const Eigen::Vector2d & viewArea)
	{
		double PixelWidth = viewArea[0];
		double PixelHeight = viewArea[1];

		// transforming the coordinates
		Eigen::Vector4d coord4(coord[0], coord[1], coord[2], 1);

		//Eigen::Vector4d point = transMat * coord4;
		Eigen::Vector3d point = TransformCoordinates(coord, transMat);

		// The transformed coordinates will be based on coordinate system
		// starting on the center of the screen. But drawing on screen normally starts
		// from top left. We then need to transform them again to have x:0, y:0 on top left.
		Eigen::Vector2d screen( (point[0] * PixelWidth) + (PixelWidth * 0.5), (-point[1] * PixelHeight) + (PixelHeight * 0.5));
		return Eigen::Vector3d( screen[0], screen[1], -point[2] );
	}

	inline QVector< PairPoints > cube()
	{
		QVector< PairPoints > face;
		QVector< PairPoints > faces;

		double len = 0.5;

		face.push_back(PairPoints(Eigen::Vector3d(  len, -len, len ),Eigen::Vector3d(  len,  len, len )));
		face.push_back(PairPoints(Eigen::Vector3d(  len,  len, len ),Eigen::Vector3d( -len,  len, len )));
		face.push_back(PairPoints(Eigen::Vector3d( -len,  len, len ),Eigen::Vector3d( -len, -len, len )));
		face.push_back(PairPoints(Eigen::Vector3d( -len, -len, len ),Eigen::Vector3d(  len, -len, len )));

		for(int i = 0; i < 4; i++)
		{
			QVector<PairPoints> newFace;

			for(int e = 0; e < 4; e++)
			{
				PairPoints line = face[e];

				line.first = Eigen::AngleAxisd(i * 0.5 * M_PI, Eigen::Vector3d::UnitX()) * line.first;
				line.second = Eigen::AngleAxisd(i * 0.5 * M_PI, Eigen::Vector3d::UnitX()) * line.second;

				newFace.push_back(line);
			}

			foreach(PairPoints line, newFace) faces.push_back(line);
		}

		return faces;
	}

	inline void render( QVector< PairPoints > lines, QImage & img, int width, int height )
	{
		// Rendering device
		img = QImage(width, height, QImage::Format_RGB32);
		QPainter painter(&img);
		painter.setRenderHint(QPainter::Antialiasing);
		painter.setRenderHint(QPainter::HighQualityAntialiasing);
		painter.fillRect(img.rect(), Qt::white);
		painter.setPen(QPen(Qt::black, 1));

		// Camera and projection
		Eigen::Vector2d viewArea( img.width(), img.height() );
		Matrix4 pmat = CreateProjectionMatrix( 80, double(img.width()) / img.height() );
		Matrix4 wmat = CreateWorldMatrix();
		Matrix4 vmat = CreateViewMatrix();
		Matrix4 transformMatrix = wmat * vmat * pmat;

		// World center
		//Eigen::Vector3d p = Project(Eigen::Vector3d(0,0,0), transformMatrix, viewArea);
		//painter.drawEllipse(QPoint( p[0], p[1] ), 3, 3);

		//angle += 0.01;

		foreach(PairPoints line, lines)
		{
			Eigen::Vector3d p0 = Project(line.first, transformMatrix, viewArea);
			Eigen::Vector3d p1 = Project(line.second, transformMatrix, viewArea);

			painter.drawLine(QPoint(p0[0], p0[1]), QPoint(p1[0], p1[1]));
		}
	}

	inline Eigen::MatrixXd render( QVector< Eigen::Vector3d > points, int width = 32, int height = 32, int pointSize = 1, Eigen::Vector3d translate = Eigen::Vector3d(0,0,0) )
	{
		Eigen::MatrixXd img = Eigen::MatrixXd::Zero( height, width );

		// Camera and projection
		Eigen::Vector2d viewArea( width, height );
		Matrix4 pmat = CreateProjectionMatrix( 90, double(width) / height );
		Matrix4 wmat = CreateWorldMatrix( translate[0], translate[1], translate[2] );
		Matrix4 vmat = CreateViewMatrix();
		Matrix4 transformMatrix = wmat * vmat * pmat;

		double minDepth = DBL_MAX;
		double maxDepth = -DBL_MAX;

		QVector<Eigen::Vector3d> allProjected;

		foreach(Eigen::Vector3d point, points){
			Eigen::Vector3d projected = Project(point, transformMatrix, viewArea);

			// off-screen points check
			int x = projected[0];
			int y = projected[1];
			if(x < 0 || x > width - 1 || y < 0 || y > height - 1) 
				continue;

			allProjected.push_back( projected );

			minDepth = qMin( projected[2], minDepth );
			maxDepth = qMax( projected[2], maxDepth );
		}

		qSort(allProjected.begin(), allProjected.end(), DepthCompare() );

		foreach(Eigen::Vector3d point, allProjected)
		{
			int x = point[0];
			int y = point[1];
			double depthVal = (point[2] - minDepth) / (maxDepth - minDepth);

			if(pointSize == 1)
			{
				img(y,x) = depthVal;
			}
			else if(pointSize == 2)
			{
				img(y,x) = depthVal;

				// cross shape
				img(qMax(y-1, 0),x) = depthVal;
				img(qMin(y+1, height-1),x) = depthVal;
				img(y,qMax(x-1,0)) = depthVal;
				img(y,qMin(x+1, width-1)) = depthVal;
			}
			else
				circle(img, x, y, pointSize, depthVal);
		}

		return img;
	}

	inline void render( QVector< Eigen::Vector3d > points, QImage & img, int width = 32, int height = 32, int pointSize = 1, Eigen::Vector3d translate = Eigen::Vector3d(0,0,0) )
	{
		img = QImage(width, height, QImage::Format_RGB32);
		QPainter painter(&img);
		painter.fillRect(img.rect(), Qt::white);
		painter.setPen(Qt::NoPen);

		Eigen::MatrixXd mimg = render(points, width, height, pointSize, translate);

		for(int y = 0; y < mimg.rows(); y++){
			for(int x = 0; x < mimg.cols(); x++){
				double depthVal = mimg(y,x);
				if(depthVal == 0) continue;

				int d = (1.0 - depthVal) * 255;
				QColor c( d,d,d );

				painter.setBrush(c);
				painter.drawEllipse(QPoint(x,y), pointSize, pointSize);
			}
		}
	}

	inline Eigen::MatrixXd vectorToMatrix( const Eigen::VectorXd & v, int width, int height )
	{
		Eigen::MatrixXd M(height, width);
		for(int i = 0; i < v.size(); i++) M(i) = v(i);
		return M;
	}

	inline QImage matrixToImage( const Eigen::MatrixXd & mimg, bool isAlphaBack = true )
	{
		int width = mimg.rows();
		int height = mimg.cols();

		QImage img(width, height, QImage::Format_ARGB32_Premultiplied);

		(isAlphaBack) ? img.fill(QColor(0,0,0,0)) : img.fill(QColor(255,255,255));

		QPainter painter(&img);

		painter.setPen(Qt::NoPen);
		painter.setBrush(Qt::NoBrush);

		for(int y = 0; y < mimg.rows(); y++){
			for(int x = 0; x < mimg.cols(); x++){
				double depthVal = mimg(y,x);

				if(depthVal == 0) continue;

				int d = (1.0 - depthVal) * 255;
				QColor c( d,d,d );

				painter.setPen( c );
				painter.drawPoint(QPoint(x,y));
			}
		}

		return img;
	}

	inline Eigen::MatrixXd renderTriangles2D( QVector< QVector< Eigen::Vector3d > > triangles, int width, int height )
	{
		Eigen::Vector2d viewArea( width, height );
		Eigen::MatrixXd buffer = Eigen::MatrixXd::Zero( height, width );
		for(auto tri : triangles) 
			drawTriangle(buffer, tri[0], tri[1], tri[2], 1.0);
		return buffer;
	}

	inline Eigen::MatrixXd renderTriangles( QVector< QVector< Eigen::Vector3d > > triangles, Matrix4 transformMatrix, int width, int height )
	{
		Eigen::Vector2d viewArea( width, height );
		Eigen::MatrixXd buffer = Eigen::MatrixXd::Zero( height, width );

		for(auto tri : triangles)
		{
			Eigen::Vector3d p0 = Project(tri[0], transformMatrix, viewArea);
			Eigen::Vector3d p1 = Project(tri[1], transformMatrix, viewArea);
			Eigen::Vector3d p2 = Project(tri[2], transformMatrix, viewArea);

			drawTriangle(buffer, p0, p1, p2, 1.0);

			// Wireframe:
			//DrawBline(buffer, p0,p1, 1.0);
			//DrawBline(buffer, p1,p2, 1.0);
			//DrawBline(buffer, p2,p0, 1.0);
		}

		return buffer;
	}

	inline Eigen::MatrixXd renderTriangles( QVector< QVector< Eigen::Vector3d > > triangles, int width, int height, Matrix4 projectionMatrix, Matrix4 viewMatrix )
	{
		Matrix4 wmat = CreateWorldMatrix();
		Matrix4 transformMatrix = wmat * viewMatrix * projectionMatrix;
		return renderTriangles(triangles, projectionMatrix, width, height );
	}

	inline Eigen::MatrixXd render( QVector< QVector< Eigen::Vector3d > > triangles, int width, int height, Matrix4 vmat = CreateViewMatrix() )
	{
		// Camera and projection
		Matrix4 pmat = CreateProjectionMatrix( 45, double(width) / height );
		return renderTriangles(triangles, width, height, pmat, vmat);
	}
}
