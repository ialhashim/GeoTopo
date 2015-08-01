#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#define glVector3( v ) glVertex3d( v.x(), v.y(), v.z() )
#define glNormal3( v ) glNormal3d( v.x(), v.y(), v.z() )
#define glLine(v1,v2) glVector3(v1);glVector3(v2)

template<typename Scalar>
Eigen::Matrix<Scalar,4,4> ortho( Scalar const& left,
                                 Scalar const& right,
                                 Scalar const& bottom,
                                 Scalar const& top,
                                 Scalar const& zNear,
                                 Scalar const& zFar ) {
    Eigen::Matrix<Scalar,4,4> mat = Eigen::Matrix<Scalar,4,4>::Identity();
    mat(0,0) = Scalar(2) / (right - left);
    mat(1,1) = Scalar(2) / (top - bottom);
    mat(2,2) = - Scalar(2) / (zFar - zNear);
    mat(3,0) = - (right + left) / (right - left);
    mat(3,1) = - (top + bottom) / (top - bottom);
    mat(3,2) = - (zFar + zNear) / (zFar - zNear);
    return mat;
}

template<typename Scalar>
Eigen::Matrix<Scalar,4,4> perspective(Scalar fovy, Scalar aspect, Scalar zNear, Scalar zFar){
    Eigen::Transform<Scalar,3,Eigen::Projective> tr;
    tr.matrix().setZero();
    assert(aspect > 0);
    assert(zFar > zNear);
    Scalar radf = M_PI * fovy / 180.0;
    Scalar tan_half_fovy = std::tan(radf / 2.0);
    tr(0,0) = 1.0 / (aspect * tan_half_fovy);
    tr(1,1) = 1.0 / (tan_half_fovy);
    tr(2,2) = - (zFar + zNear) / (zFar - zNear);
    tr(3,2) = - 1.0;
    tr(2,3) = - (2.0 * zFar * zNear) / (zFar - zNear);
    return tr.matrix();
}

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar,4,4> lookAt(Derived const & eye, Derived const & center, Derived const & up){
    typedef Eigen::Matrix<typename Derived::Scalar,4,4> Matrix4;
    typedef Eigen::Matrix<typename Derived::Scalar,3,1> Vector3;
    Vector3 f = (center - eye).normalized();
    Vector3 u = up.normalized();
    Vector3 s = f.cross(u).normalized();
    u = s.cross(f);
    Matrix4 mat = Matrix4::Zero();
    mat(0,0) = s.x();
    mat(0,1) = s.y();
    mat(0,2) = s.z();
    mat(0,3) = -s.dot(eye);
    mat(1,0) = u.x();
    mat(1,1) = u.y();
    mat(1,2) = u.z();
    mat(1,3) = -u.dot(eye);
    mat(2,0) = -f.x();
    mat(2,1) = -f.y();
    mat(2,2) = -f.z();
    mat(2,3) = f.dot(eye);
    mat.row(3) << 0,0,0,1;
    return mat;
}

#include <QMessageBox>
inline void showImages(QVector<QImage> imgs, QStringList labels = QStringList()){ 
	QMessageBox msg;
	QImage img(imgs.size() * imgs.front().width(), imgs.front().height(), QImage::Format_RGBA8888_Premultiplied);
	QPainter painter(&img); QFont font("Monospace",7); font.setStyleHint(QFont::Monospace); painter.setFont(font);
	for(int i = 0; i < imgs.size(); i++){
		painter.drawImage(0,0,imgs[i]); 
		if(!labels.isEmpty()) {
			painter.setPen(QPen(Qt::white)); painter.drawText(QPoint(5,10), labels[i]);
			painter.setPen(QPen(Qt::red)); painter.drawText(QPoint(4,9), labels[i]);
		}
		painter.translate(imgs[i].width(),0); 
	}
	msg.setIconPixmap(QPixmap::fromImage(img)); msg.exec(); 
}
inline void showImage(QImage imgs, QStringList labels = QStringList()){
	showImages(QVector<QImage>() << imgs, labels);
}

// Drawing helpers
inline void drawArrow(Eigen::Vector3d p0, Eigen::Vector3d p1, QColor c, QPainter & painter, int arrowSize = 5){
	QLineF line(QPoint(p0[0], p0[1]), QPoint(p1[0], p1[1]));
	painter.setPen(QPen(c));
	painter.drawLine(line);
	if(line.length() < 1e-3) return;
	static const double Pi = 3.14159265358979323846264338327950288419717, TwoPi = 2.0 * Pi;
	double angle = std::acos(line.dx() / std::max(1e-3,line.length()));
	if (line.dy() >= 0) angle = TwoPi - angle;
	QPointF destArrowP1 = line.pointAt(1) + QPointF(sin(angle - Pi / 3) * arrowSize,cos(angle - Pi / 3) * arrowSize);
	QPointF destArrowP2 = line.pointAt(1) + QPointF(sin(angle - Pi + Pi / 3) * arrowSize,cos(angle - Pi + Pi / 3) * arrowSize);
	painter.setBrush(c);
	painter.drawPolygon(QPolygonF() << line.p2() << destArrowP1 << destArrowP2);
}
