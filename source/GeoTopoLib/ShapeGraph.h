#pragma once
#include "StructureGraph.h"
#include "disjointset.h"

// 64 bit
#pragma warning (disable : 4267)
#pragma warning (disable : 4018)

namespace Structure
{
static QString null_part = "NULLPART";

// Structural relations between parts
struct Relation{
    QStringList parts;
    enum RelationType{ REFLECTIONAL, ROTATIONAL, TRANSLATIONAL, SELF, NULLRELATION } type;
    Vector3 axis, point;
    Array1D_Vector3 deltas;
    PropertyMap property;
    Relation(Vector3 axis = Vector3(0, 0, 0), Vector3 point = Vector3(0, 0, 0), RelationType type = SELF)
        : axis(axis), point(point), type(type) {}
    bool operator==(const Relation & other){ return this->parts == other.parts; }
};

struct Landmark : public Eigen::Vector3d{
    Landmark(unsigned int id = -1, const Eigen::Vector3d vec = Eigen::Vector3d(0, 0, 0)) :
        id(id), Eigen::Vector3d(vec), constraint_id(-1){ u = v = -1; partid = "none"; }
    double u, v;
    QString partid;
    unsigned int id;
    int constraint_id;
    void serialize(QDataStream& os) const {
        os << id;
        os << partid;
        os << u << v;
        os << this->x() << this->y() << this->z();
    }
    void deserialize(QDataStream& is){
        is >> id;
        is >> partid;
        is >> u >> v;
        is >> this->x() >> this->y() >> this->z();
    }
};

typedef QVector<Landmark> Landmarks;

struct ShapeGraph : public Graph{
    ShapeGraph(QString path) : Graph(path), animation_index(0) {}
    ShapeGraph(const ShapeGraph& other) : Graph(other){
        this->landmarks = other.landmarks;
        this->relations = other.relations;

        this->animation = other.animation;
        this->animation_index = other.animation_index;
        this->animation_debug = other.animation_debug;
    }
    ~ShapeGraph(){}

    QVector<Landmarks> landmarks;

    // Part relations and grouping
    QVector<Relation> relations;
    Eigen::AlignedBox3d relationBBox(const Relation & r){
        Eigen::AlignedBox3d box;
        for (auto nid : r.parts) box.extend(robustBBox(nid));
        return box;
    }
    Relation & relationOf(const QString & partID){
        for (auto & r : relations) if (r.parts.contains(partID)) return r;
        return relations.front(); // should not get here
    }
    bool hasRelation(const QString & partID){
        for (auto & r : relations) if (r.parts.contains(partID)) return true;
        return false;
    }

    // Visualize shape changes:
    int animation_index;
    QVector<Array2D_Vector3> animation;
    QVector< QVector<RenderObject::Base*> > animation_debug;
    void saveKeyframe(){ animation_index = animation.size(); animation.push_back(getAllControlPoints()); animation_debug << QVector<RenderObject::Base*>(); }
    void pushKeyframeDebug(RenderObject::Base* d){
        if (!animation_debug.size()) animation_debug << QVector<RenderObject::Base*>();
        animation_debug.back() << d;
    }

    void saveLandmarks(QString filename){
        QFile file(filename); file.open(QIODevice::WriteOnly);
        QDataStream os(&file);
        os << (int)landmarks.size();
        for (auto & landmark : landmarks)
        {
            os << (int)landmark.size();
            for (auto & l : landmark) l.serialize(os);
        }
    }

    void loadLandmarks(QString filename){
        QFile file(filename); file.open(QIODevice::ReadOnly);
        QDataStream is(&file);
        int count;
        is >> count;
        landmarks.resize(count);
        for (int i = 0; i < count; i++)
        {
            int landmarkCount;
            is >> landmarkCount;
            landmarks[i].resize(landmarkCount);
            for (int j = 0; j < landmarkCount; j++)
                landmarks[i][j].deserialize(is);
        }
    }

    static void correspondTwoNodes(QString sid, Structure::Graph * src, QString tid, Structure::Graph * tgt)
    {
        auto sn = src->getNode(sid);
        auto tn = tgt->getNode(tid);

        if (sn->type() == tn->type())
        {
            if (sn->type() == Structure::CURVE)
                ShapeGraph::correspondTwoCurves((Structure::Curve *)sn, (Structure::Curve *)tn, tgt);

            if (sn->type() == Structure::SHEET)
                ShapeGraph::correspondTwoSheets((Structure::Sheet *)sn, (Structure::Sheet *)tn, tgt);
        }
        else
        {
            Structure::Curve * sourceCurve;
            Structure::Graph * target = tgt;

            Structure::Node * snode = (sn->type() == Structure::SHEET) ? sn : tn;
            if (snode != sn) tn = sn;

            auto cpts = snode->controlPoints();
            double resolution = (cpts[0] - cpts[1]).norm() * 0.5;
            sourceCurve = new Structure::Curve(NURBS::NURBSCurved::createCurveFromPoints(snode->discretizedAsCurve(resolution)), "temp");

            ShapeGraph::correspondTwoCurves(sourceCurve, (Structure::Curve *)tn, target);

            delete sourceCurve;
        }
    }

    static void correspondTwoCurves(Structure::Curve *sCurve, Structure::Curve *tCurve, Structure::Graph * tgt)
    {
        std::vector<Vector3> sCtrlPoint = sCurve->controlPoints();
        std::vector<Vector3> tCtrlPoint = tCurve->controlPoints();

        // Euclidean for now, can use Geodesic distance instead if need
        Vector3 scenter = sCurve->center();
        Vector3 sfront = sCtrlPoint.front() - scenter;
        Vector3 tcenter = tCurve->center();
        Vector3 tfront = tCtrlPoint.front() - tcenter;
        Vector3 tback = tCtrlPoint.back() - tcenter;

        float f2f = (sfront - tfront).norm();
        float f2b = (sfront - tback).norm();

        float diff = std::abs(f2f - f2b);
        float threshold = 0.1f;

        if (f2f > f2b && diff > threshold)
        {
            // Flip the target
            std::vector<Scalar> tCtrlWeight = tCurve->controlWeights();
            std::reverse(tCtrlPoint.begin(), tCtrlPoint.end());
            std::reverse(tCtrlWeight.begin(), tCtrlWeight.end());

            NURBS::NURBSCurved newCurve(tCtrlPoint, tCtrlWeight);
            tCurve->curve = newCurve;

            // Update the coordinates of links
            foreach(Structure::Link * l, tgt->getEdges(tCurve->id))
                l->setCoord(tCurve->id, inverseCoords(l->getCoord(tCurve->id)));
        }
    }

    static void correspondTwoSheets(Structure::Sheet *sSheet, Structure::Sheet *tSheet, Structure::Graph * tgt)
    {
        // Old properties
        NURBS::NURBSRectangled &oldRect = tSheet->surface;
        int uDegree = oldRect.GetDegree(0);
        int vDegree = oldRect.GetDegree(1);
        bool uLoop = oldRect.IsLoop(0);
        bool vLoop = oldRect.IsLoop(1);
        bool uOpen = oldRect.IsOpen(0);
        bool vOpen = oldRect.IsOpen(1);
        bool isModified = false;
        bool isUVFlipped = false;

        // Control points and weights
        Array2D_Vector3 sCtrlPoint = sSheet->surface.mCtrlPoint;
        Array2D_Real sCtrlWeight = sSheet->surface.mCtrlWeight;

        Array2D_Vector3 tCtrlPoint = tSheet->surface.mCtrlPoint;
        Array2D_Real tCtrlWeight = tSheet->surface.mCtrlWeight;

        Array2D_Vector3 tCtrlPointNew;
        Array2D_Real tCtrlWeightNew;

        Vector3 scenter = sSheet->center();
        Vector3 tcenter = tSheet->center();

        // Get the extreme points.
        Vector3 s00 = sCtrlPoint.front().front();
        Vector3 s01 = sCtrlPoint.front().back();
        Vector3 s10 = sCtrlPoint.back().front();
        Vector3 sU = s10 - s00;
        Vector3 sV = s01 - s00;

        Vector3 t00 = tCtrlPoint.front().front();
        Vector3 t01 = tCtrlPoint.front().back();
        Vector3 t10 = tCtrlPoint.back().front();
        Vector3 tU = t10 - t00;
        Vector3 tV = t01 - t00;

        // Flip if need
        Vector3 sUV = sU.cross(sV);
        Vector3 tUV = tU.cross(tV);
        if (sUV.dot(tUV) < 0)
        {
            // Reverse the target along u direction
            std::reverse(tCtrlPoint.begin(), tCtrlPoint.end());
            std::reverse(tCtrlWeight.begin(), tCtrlWeight.end());

            // Update tU
            tU = -tU;
            tUV = -tUV;
            isModified = true;

            // Update the coordinates of links
            foreach(Structure::Link * l, tgt->getEdges(tSheet->id)){
                Array1D_Vector4 oldCoord = l->getCoord(tSheet->id), newCoord;
                foreach(Vector4d c, oldCoord) newCoord.push_back(Vector4d(1 - c[0], c[1], c[2], c[3]));
                l->setCoord(tSheet->id, newCoord);
            }
        }

        // Rotate if need
        Scalar cosAngle = sU.normalized().dot(tU.normalized());
        Scalar cos45 = sqrtf(2.0) / 2;

        // Do Nothing
        if (cosAngle > cos45)
        {
            tCtrlPointNew = tCtrlPoint;
            tCtrlWeightNew = tCtrlWeight;
        }
        // Rotate 180 degrees
        else if (cosAngle < -cos45)
        {
            //  --> sV				tU
            // |					|
            // sU             tV <--
            // By flipping along both directions
            std::reverse(tCtrlPoint.begin(), tCtrlPoint.end());
            std::reverse(tCtrlWeight.begin(), tCtrlWeight.end());

            for (int i = 0; i < (int)tCtrlPoint.size(); i++)
            {
                std::reverse(tCtrlPoint[i].begin(), tCtrlPoint[i].end());
                std::reverse(tCtrlWeight[i].begin(), tCtrlWeight[i].end());
            }

            // The new control points and weights
            tCtrlPointNew = tCtrlPoint;
            tCtrlWeightNew = tCtrlWeight;
            isModified = true;

            // Update the coordinates of links
            foreach(Structure::Link * l, tgt->getEdges(tSheet->id)){
                Array1D_Vector4 oldCoord = l->getCoord(tSheet->id), newCoord;
                foreach(Vector4d c, oldCoord) newCoord.push_back(Vector4d(1 - c[0], 1 - c[1], c[2], c[3]));
                l->setCoord(tSheet->id, newCoord);
            }
        }
        // Rotate 90 degrees
        else
        {
            Vector3 stU = sU.cross(tU);
            if (stU.dot(sUV) >= 0)
            {
                //  --> sV		tV
                // |			|
                // sU           --> tU
                // Transpose and reverse along U
                tCtrlPointNew = transpose<Vector3>(tCtrlPoint);
                tCtrlWeightNew = transpose<Scalar>(tCtrlWeight);

                std::reverse(tCtrlPointNew.begin(), tCtrlPointNew.end());
                std::reverse(tCtrlWeightNew.begin(), tCtrlWeightNew.end());

                // Update the coordinates of links
                foreach(Structure::Link * l, tgt->getEdges(tSheet->id)){
                    Array1D_Vector4 oldCoord = l->getCoord(tSheet->id), newCoord;
                    foreach(Vector4d c, oldCoord) newCoord.push_back(Vector4d(1 - c[1], c[0], c[2], c[3]));
                    l->setCoord(tSheet->id, newCoord);
                }
            }
            else
            {
                //  --> sV	tU<--
                // |			 |
                // sU			tV
                // Reverse along U and Transpose
                std::reverse(tCtrlPoint.begin(), tCtrlPoint.end());
                std::reverse(tCtrlWeight.begin(), tCtrlWeight.end());

                tCtrlPointNew = transpose<Vector3>(tCtrlPoint);
                tCtrlWeightNew = transpose<Scalar>(tCtrlWeight);

                // Update the coordinates of links
                foreach(Structure::Link * l, tgt->getEdges(tSheet->id)){
                    Array1D_Vector4 oldCoord = l->getCoord(tSheet->id), newCoord;
                    foreach(Vector4d c, oldCoord) newCoord.push_back(Vector4d(c[1], 1 - c[0], c[2], c[3]));
                    l->setCoord(tSheet->id, newCoord);
                }
            }

            isModified = true;
            isUVFlipped = true;
        }

        // Create a new sheet if need
        if (isModified)
        {
            NURBS::NURBSRectangled newRect;
            if (isUVFlipped)
                newRect = NURBS::NURBSRectangled(tCtrlPointNew, tCtrlWeightNew, vDegree, uDegree, vLoop, uLoop, vOpen, uOpen);
            else
                newRect = NURBS::NURBSRectangled(tCtrlPointNew, tCtrlWeightNew, uDegree, vDegree, uLoop, vLoop, uOpen, vOpen);

            tSheet->surface = newRect;
        }
    }

    static Array2D_Vector4 computeSideCoordinates(int resolution = 10)
    {
        Array2D_Vector4 coords(4);
        for (int i = 0; i < resolution; i++) coords[0].push_back(Eigen::Vector4d(double(i) / (resolution - 1), 0, 0, 0));
        for (int i = 0; i < resolution; i++) coords[1].push_back(Eigen::Vector4d(1, double(i) / (resolution - 1), 0, 0));
        for (int i = 0; i < resolution; i++) coords[2].push_back(Eigen::Vector4d(1 - (double(i) / (resolution - 1)), 1, 0, 0));
        for (int i = 0; i < resolution; i++) coords[3].push_back(Eigen::Vector4d(0, 1 - (double(i) / (resolution - 1)), 0, 0));
        return coords;
    }

    static QString convertCurvesToSheet(Structure::Graph * graph, QStringList & nodeIDs, const Array2D_Vector4 & sideCoordinates)
    {
        // Fit centers into 3D line
        Eigen::Vector3d point, direction;
        Eigen::MatrixXd curveCenters(nodeIDs.size(), 3);
        for (int r = 0; r < (int)nodeIDs.size(); r++)
            curveCenters.row(r) = graph->getNode(nodeIDs[r])->position(Eigen::Vector4d(0.5, 0.5, 0, 0));
        point = Eigen::Vector3d(curveCenters.colwise().mean());
        curveCenters = curveCenters.rowwise() - point.transpose();

        if (nodeIDs.size() == 2)
        {
            direction = (curveCenters.row(1) - curveCenters.row(0)).normalized();
        }
        else
        {
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(curveCenters, Eigen::ComputeThinU | Eigen::ComputeThinV);
            direction = Eigen::Vector3d(svd.matrixV().col(0)).normalized();
        }

        // Sort curves
        std::vector <size_t> sorted;
        QMap<size_t, double> dists;
        for (size_t r = 0; r < nodeIDs.size(); r++) dists[r] = curveCenters.row(r).dot(direction);
        for (auto p : sortQMapByValue(dists)) sorted.push_back(p.second);

        // Build sheet control points
        Array2D_Vector3 cpnts;
        for (size_t i = 0; i < sorted.size(); i++){
            size_t idx = sorted[i];
            auto curNode = graph->getNode(nodeIDs[idx])->getPoints(std::vector<Array1D_Vector4>(1, sideCoordinates[0])).front();
            cpnts.push_back(curNode);
        }

        // Requirement for NURBS is minimum 4 rows
        if (cpnts.size() < 4){
            if (cpnts.size() == 2)
            {
                Array1D_Vector3 m1, m2;
                for (size_t i = 0; i < cpnts.front().size(); i++) m1.push_back(AlphaBlend(1.0 / 3.0, cpnts.front()[i], cpnts.back()[i]));
                for (size_t i = 0; i < cpnts.front().size(); i++) m2.push_back(AlphaBlend(2.0 / 3.0, cpnts.front()[i], cpnts.back()[i]));
                cpnts.insert(cpnts.begin() + 1, m1);
                cpnts.insert(cpnts.begin() + 2, m2);
            }
            else
            {
                Array1D_Vector3 m1, m2;
                for (size_t i = 0; i < cpnts.front().size(); i++) m1.push_back(AlphaBlend(1.0 / 2.0, cpnts[0][i], cpnts[1][i]));
                for (size_t i = 0; i < cpnts.front().size(); i++) m2.push_back(AlphaBlend(1.0 / 2.0, cpnts[1][i], cpnts[2][i]));
                cpnts.insert(cpnts.begin() + 1, m1);
                cpnts.insert(cpnts.begin() + 3, m2);
            }
        }
        NURBS::NURBSRectangled sheet = NURBS::NURBSRectangled::createSheetFromPoints(transpose(cpnts));
        Structure::Sheet * newSheet = new Structure::Sheet(sheet, nodeIDs.join(","));

        if (!graph->getNode(newSheet->id)) graph->addNode(newSheet);

        return newSheet->id;
    }

    void performCuts()
    {
        QVector<QString> toCutIDs;
        for (auto n : nodes)
        {
            // this is restrictive, needs to be more general in the future..
            if (hasDoubleEdges(n->id))
            {
                Structure::Node * otherDouble = nullptr;
                for (auto l : getEdges(n->id)){
                    if (hasDoubleEdges(l->otherNode(n->id)->id)){
                        otherDouble = l->otherNode(n->id);
                        break;
                    }
                }

                if (!otherDouble){
                    toCutIDs << n->id;
                }
                else{
                    if (n->length() > otherDouble->length())
                        toCutIDs << n->id;
                }
            }
        }

        for (auto nid : toCutIDs)
            cutNode(nid, 2);
    }

    void performJoins()
    {
        DisjointSet U(nodes.size());

        // Merge join-able nodes
        for (auto ni : nodes)
        {
            if (ni->type() != Structure::CURVE) continue;

            double theta = M_PI * 0.9;
            double threshold = ni->length() * 0.01;
            double angle_threshold = abs(cos(theta));

            for (auto l : getEdges(ni->id))
            {
                auto nj = l->otherNode(ni->id);
                if (nj->type() != Structure::CURVE) continue;

                auto p = ni->approxProjection(nj->startPoint());
                auto q = ni->approxProjection(nj->endPoint());

                double dist = (p - q).norm();

                auto meetingPoint = [&](Structure::Node * ni, Structure::Node * nj){
                    double dist1 = (ni->approxProjection(nj->startPoint()) - nj->startPoint()).norm();
                    double dist2 = (ni->approxProjection(nj->endPoint()) - nj->endPoint()).norm();
                    if (dist1 < dist2)
                        return ni->approxCoordinates(nj->startPoint())(0);
                    else
                        return ni->approxCoordinates(nj->endPoint())(0);
                };

                auto midTangent = [&](Structure::Node * n, double t, double range){
                    double t0 = std::max(0.0, std::min(1.0, t - range));
                    double t1 = std::max(0.0, std::min(1.0, t + range));
                    return (n->position(Eigen::Vector4d(t1, 0, 0, 0)) - n->position(Eigen::Vector4d(t0, 0, 0, 0))).normalized();
                };

                double range = 0.1;
                auto ti = midTangent(ni, meetingPoint(ni, nj), range);
                auto tj = midTangent(nj, meetingPoint(nj, ni), range);

                double dot_diagonals = abs(ti.dot(tj));

                bool isAngle = angle_threshold < dot_diagonals;

                if (dist < threshold && isAngle)
                {
                    U.Union(nodes.indexOf(ni), nodes.indexOf(nj));
                }
            }
        }

        // Convert numerical indices to alphabetical IDs
        QVector<QStringList> groupedNodes;
        for (auto group : U.Groups())
        {
            QStringList nodeIDs;
            for (auto nidx : group) nodeIDs << nodes[nidx]->id;
            if (nodeIDs.size() < 2) continue;
            groupedNodes << nodeIDs;
        }

        // Apply join
        for (auto nodeIDs : groupedNodes)
            joinNodes(nodeIDs);
    }

};
}

#include <numeric>
template<typename Scalar>
static inline Scalar stdev(const std::vector<Scalar> & v)
{
    Scalar sum = std::accumulate(v.begin(), v.end(), 0.0);
    Scalar mean = sum / v.size();
    std::vector<Scalar> diff(v.size());
    std::transform(v.begin(), v.end(), diff.begin(), std::bind2nd(std::minus<Scalar>(), mean));
    Scalar sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    Scalar stdev = std::sqrt(sq_sum / v.size());

    return stdev;
}

struct NormalAnalysis{
    std::vector<double> x, y, z;
    void addNormal(const Eigen::Vector3d & n){ x.push_back(n.x()); y.push_back(n.y()); z.push_back(n.z()); }
    double standardDeviation(){ return std::max(stdev(x), std::max(stdev(y), stdev(z))); }
};

template<typename Vector3>
std::pair<Vector3, Vector3> best_plane_from_points(const std::vector<Vector3> & c)
{
    // copy coordinates to  matrix in Eigen format
    size_t num_atoms = c.size();
    Eigen::Matrix< typename Vector3::Scalar, Eigen::Dynamic, Eigen::Dynamic > coord(3, num_atoms);
    for (size_t i = 0; i < num_atoms; ++i) coord.col(i) = c[i];

    // calculate centroid
    Vector3 centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

    // subtract centroid
    coord.row(0).array() -= centroid(0); coord.row(1).array() -= centroid(1); coord.row(2).array() -= centroid(2);

    // we only need the left-singular matrix here
    //  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
    auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    Vector3 plane_normal = svd.matrixU().template rightCols<1>();
    return std::make_pair(centroid, plane_normal);
}

template<typename Vector3>
std::pair < Vector3, Vector3 > best_line_from_points(const std::vector<Vector3> & c)
{
    // copy coordinates to  matrix in Eigen format
    size_t num_atoms = c.size();
    Eigen::Matrix< typename Vector3::Scalar, Eigen::Dynamic, Eigen::Dynamic > centers(num_atoms, 3);
    for (size_t i = 0; i < num_atoms; ++i) centers.row(i) = c[i];

    Vector3 origin = centers.colwise().mean();
    Eigen::MatrixXd centered = centers.rowwise() - origin.transpose();
    Eigen::MatrixXd cov = centered.adjoint() * centered;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
    Vector3 axis = eig.eigenvectors().col(2).normalized();

    return std::make_pair(origin, axis);
}
