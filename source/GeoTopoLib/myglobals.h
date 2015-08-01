#pragma once

#include <vector>
#include <set>

#define AlphaBlend(alpha, start, end) ( ((1-alpha) * start) + (alpha * end) )

#include <Eigen/Core>
typedef Eigen::Vector3d Vector3Type;

template<typename Vector3d>
static inline Vector3d orthogonalVector(const Vector3d& n) {
	if ((abs(n.y()) >= 0.9 * abs(n.x())) &&
		abs(n.z()) >= 0.9 * abs(n.x())) return Vector3d(0.0, -n.z(), n.y());
	else if ( abs(n.x()) >= 0.9 * abs(n.y()) &&
		abs(n.z()) >= 0.9 * abs(n.y()) ) return Vector3d(-n.z(), 0.0, n.x());
	else return Vector3d(-n.y(), n.x(), 0.0);
}

static inline double deg_to_rad(double deg) {
	double rad = (M_PI * deg) / 180.0;
	return rad;
}

static inline double rad_to_deg(double rad) {
	double deg = 180.0 / M_PI * rad;
	return deg;
}

static double roundDecimal(const double number, const int decimals) {
	int intPart = number;
	double decimalPart = number - intPart;
	double numberDecimals = pow(10, decimals);
	double decimalRound = round(decimalPart * numberDecimals);
	return static_cast<double> (intPart + (decimalRound / numberDecimals));
}

inline QVector<QColor> rndColors2(int count){
	QVector<QColor> colors;
	float currentHue = 0.0;
	for (int i = 0; i < count; i++){
		colors.push_back( QColor::fromHslF(currentHue, 1.0, 0.5) );
		currentHue += 0.618033988749895f;
		currentHue = std::fmod(currentHue, 1.0f);
	}
	return colors;
}

template<typename T, typename Container>
std::vector<T> random_sampling( const Container& original_samples, size_t count ){
	std::vector<T> samples( original_samples.begin(), original_samples.end() );
	if(samples.size() < 1) return samples;
	std::random_device rd;
	std::mt19937 g(rd());
	std::shuffle(samples.begin(), samples.end(), g);
	samples.resize( std::min(std::max(size_t(1), count), size_t(samples.size()-1)) );
	return samples;
}

template<typename Scalar, typename Container>
inline static Eigen::Matrix<Scalar,-1,-1> toEigenMatrix( const Container& vectors ){
	typedef typename Container::value_type VectorType;
	Eigen::Matrix<Scalar,-1,-1> M(vectors.size(), vectors.front().size());
	for(size_t i = 0; i < (size_t)vectors.size(); i++)
		for(size_t j = 0; j < (size_t)vectors.front().size(); j++)
			M(i,j) = vectors[i][j];
	return M;
}

template<typename Scalar, typename Matrix>
inline static std::vector< std::vector<Scalar> > fromEigenMatrix( const Matrix & M ){
	std::vector< std::vector<Scalar> > m;
	m.resize(M.rows(), std::vector<Scalar>(M.cols(), 0));
	for(size_t i = 0; i < m.size(); i++)
		for(size_t j = 0; j < m.front().size(); j++)
			m[i][j] = M(i,j);
	return m;
}

// Input / output Eigen matrix from / to disk
#include <QFileInfo>
#include <QTextStream>
static inline Eigen::MatrixXd matrixFromFile( QString filename, QString splitChar = "," )
{
	Eigen::MatrixXd M;

	// Read data
	QFile file( filename );
	if(!file.open(QFile::ReadOnly | QFile::Text)) return M;
	std::vector< std::vector<double> > mat;
	QTextStream in(&file);
	QStringList lines = in.readAll().split('\n');
	for( auto line : lines ){
		auto row_string = line.split(splitChar, QString::SkipEmptyParts);
		if(row_string.size() < 1) continue;

		std::vector<double> row;
		for(auto token : row_string) 
			row.push_back(token.toDouble());
		mat.push_back( row );
	}

	// Copy values
	M = Eigen::MatrixXd(mat.size(), mat.front().size());
	for(size_t i = 0; i < mat.size(); i++){
		for(size_t j = 0; j < mat.front().size(); j++){
			M(i,j) = mat[i][j];
		}
	}

	return M;
}

static inline void matrixToFile(const Eigen::MatrixXd & M, QString filename){
	QFile file( filename );
	if(!file.open(QFile::WriteOnly | QFile::Text)) return;
	QTextStream out(&file);
	for(unsigned int i = 0; i < M.rows(); i++)
	{
		QStringList row;
		for(unsigned int j = 0; j < M.cols(); j++)
			row << QString::number(M(i,j));
		out << (row.join(",") + "\n");
	}
}

static inline void saveToTextFile( QString filename, QStringList items ){
	QFile file( filename );
	if(!file.open(QFile::WriteOnly | QFile::Text)) return;
	QTextStream out(&file);
	for(auto item : items){
		out << item << "\n";
	}
}

template<typename T>
static inline QStringList vecToString2( T data, int limit = -1 ){
	QStringList l;
	for(auto row : data){
		QStringList line;
		for(auto d : row) line << QString("%1").arg( d );
		l << QString("%1").arg( line.join(", ") );
		if(limit > 0 && l.size() == limit) break;
	}
	return l;
}

// Simplify runtime debugging
#include <QMessageBox>
template<typename DataType>
static inline void debugBox( DataType message ){
	QMessageBox msgBox;
	msgBox.setTextFormat(Qt::RichText);
	msgBox.setText( QString("%1").arg(message) );
	msgBox.setStandardButtons(QMessageBox::Ok);
	msgBox.exec();
}
static inline void debugBoxList( QStringList messages ){
	debugBox( messages.join("<br>") );
}
template<typename Container>
static inline void debugBoxVec( Container data ){
	QStringList l;
	for(auto d : data) l << QString("%1").arg(d);
	debugBoxList(l);
}
template<typename Container2D>
static inline void debugBoxVec2( Container2D data, int limit = -1 ){
	QStringList l;
	for(auto row : data){
		QStringList line;
		for(auto d : row) line << QString("%1").arg( d );
		l << QString("%1").arg( line.join(", ") );
		if(limit > 0 && l.size() == limit) break;
	}
	if(limit > 0 && data.size() - l.size() > 0) l << QString("... (%1) more").arg(data.size() - l.size());
	debugBoxList(l);
}

// Generate subsets
template<typename T>
std::vector< std::vector<T> > sets( const std::vector<T> & input, int min_size = 1, 
								   int max_size = 0, bool isSorted = false ){
	std::vector< std::vector<T> > results;
	if(input.size() == 0) return results;
	int total = pow(2, input.size());
	for(int mask = 0; mask < total; mask++){
		std::vector<T> result;
		int i = (int)input.size() - 1; 
		do{
			if( (mask & (1 << i)) != 0){
				result.push_back(input[i]);
			}
		}while(i--);
		if( result.size() < min_size ) continue;
		if( max_size > 0 && result.size() > max_size ) continue;
		if( isSorted ) std::sort(result.begin(), result.end());
		results.push_back(result);
	}
	return results; 
};

// Cartesian product of vector of vectors
// From http://stackoverflow.com/questions/5279051/how-can-i-create-cartesian-product-of-vector-of-vectors
template<typename Vvi>
void cart_product(Vvi& out, const Vvi& in, int maxCount = -1)
{
	typedef Vvi::value_type Vi;
	struct Digits {
		Vi::const_iterator begin;
		Vi::const_iterator end;
		Vi::const_iterator me;
	};
	typedef std::vector<Digits> Vd;

	Vd vd;

	// Start all of the iterators at the beginning.
	for(Vvi::const_iterator it = in.begin();
		it != in.end();
		++it) {
			Digits d = {(*it).begin(), (*it).end(), (*it).begin()};
			vd.push_back(d);
	}

	while(1) 
	{
		// Construct your first product vector by pulling 
		// out the element of each vector via the iterator.
		Vi result;
		for(Vd::const_iterator it = vd.begin();
			it != vd.end();
			it++) {
				result.push_back(*(it->me));
		}

		out.push_back(result);

		if (maxCount > 0 && out.size() > maxCount) return;

		// Increment the rightmost one, and repeat.

		// When you reach the end, reset that one to the beginning and
		// increment the next-to-last one. You can get the "next-to-last"
		// iterator by pulling it out of the neighboring element in your
		// vector of iterators.
		for(Vd::iterator it = vd.begin(); ; ) {
			// okay, I started at the left instead. sue me
			++(it->me);
			if(it->me == it->end) {
				if(it+1 == vd.end()) {
					// I'm the last digit, and I'm about to roll
					return;
				} else {
					// cascade
					it->me = it->begin;
					++it;
				}
			} else {
				// normal
				break;
			}
		}
	}
}

// Image utility:
#include <QImage>
#include <QPainter>
inline QImage stitchImages(const QImage & a, const QImage & b, bool isVertical = false, int padding = 2, QColor background = Qt::white)
{
	int newWidth = isVertical ? (2 * padding) + std::max(a.width(), b.width()) : (3 * padding) + a.width() + b.width();
	int newHeight = isVertical ? (3 * padding) + a.height() + b.height() : (2 * padding) + std::max(a.height(), b.height());
	QImage img(newWidth, newHeight, QImage::Format_ARGB32_Premultiplied);
	QPainter painter;
	painter.begin(&img);
	painter.fillRect(img.rect(), background);
	if (isVertical)
	{
		painter.drawImage(padding, padding, a);
		painter.drawImage(padding, padding + a.height() + padding, b);
	}
	else
	{
		painter.drawImage(padding, padding, a);
		painter.drawImage(padding + a.width() + padding, padding, b);
	}
	painter.end();
	return img;
}

inline QImage drawText(QString message, QImage a, int x = 14, int y = 14, QColor color = Qt::black)
{
	QPainter painter;
	painter.begin(&a);
	painter.setRenderHint(QPainter::Antialiasing);
	painter.setRenderHint(QPainter::HighQualityAntialiasing);
	painter.setBrush(Qt::black);
	painter.setOpacity(0.2);
	painter.drawText(QPoint(x + 1, y + 1), message);
	painter.setOpacity(1.0);
	painter.setBrush(color);
	painter.drawText(QPoint(x, y), message);
	painter.end();
	return a;
}
