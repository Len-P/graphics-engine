#ifndef ENGINE_FIGURE3D_H
#define ENGINE_FIGURE3D_H
#define _USE_MATH_DEFINES

#include "../utils/ini_configuration.h"
#include "../utils/easy_image.h"
#include "../utils/vector3d.h"
#include "../LSystem2D/LSystem2D.h"
#include <list>
#include <cmath>



using namespace ini;
using namespace img;
using namespace LSystem2D;
using std::list;
using std::to_string;

namespace Figure3D {

    EasyImage parseIniFigure3D(const Configuration &conf, const bool ZBufferedWireframes = false, const bool ZBuffering = false);

    class Face {
    public:
        vector<int> pointIndexes; // Indexes refer to points in the points vector in the Figure class

        // Face constructor
        explicit Face(vector<int> aPointIndexes);

        // Triangulates a face of any amount of points
        vector<Face> triangulate(const Face &face);
    };

    class Figure {
    public:
        vector<Vector3D> points;
        vector<Face> faces;
        Color color;

        // Figure constructors
        Figure();
        Figure(vector<Vector3D> &aPoints, vector<Face> &aFaces, const Color &aColor);

        // Apply transformation matrix to figure
        void applyTransformation(const Matrix &mat);

        // For figures made out of triangles, divide every triangle into 4 triangles n times
        void triangulateTriangles(const int n);

        // ?============== Static Methods ==============? //
        static Figure createCube(const Color &color);
        static Figure createTetrahedron(const Color &color);
        static Figure createOctahedron(const Color &color);
        static Figure createIcosahedron(const Color &color);
        static Figure createDodecahedron(const Color &color);
        static Figure createSphere(const double r, const int n, const Color &color);
        static Figure createCone(const double h, const int n, const Color &color);
        static Figure createCylinder(const double h, const int n, const Color &color);
        static Figure createTorus(const double r, const double R, const int n, const int m, const Color &color);

        static void recursiveLSystem3D(const string &str, unsigned int iter, unsigned int maxIter, Vector3D &H, Vector3D &L, Vector3D &U, const LParser::LSystem3D &l_system, vector<Vector3D> &points, vector<Figure3D::Face> &faces, Vector3D &startPoint, Vector3D &endPoint, stack<tuple<Vector3D, Vector3D, Vector3D, Vector3D>> &stack, const Color &color);
        static Figure LSystem3DToFigure(const LParser::LSystem3D &l_system, const Color &color);
        // ?============================================? //
    };

    typedef list<Figure> Figures3D;

    // ?============== Transformations ==============? //
    void toPolar(const Vector3D &point, double &r, double &theta, double &phi);

    Matrix scaleFigure(const double scale);
    Matrix rotateX(const double angle);
    Matrix rotateY(const double angle);
    Matrix rotateZ(const double angle);
    Matrix translate(const Vector3D &vector);
    Matrix eyePointTrans(const Vector3D &eyepoint);

    // Apply transformation matrix to list of figures
    void applyTransformation(Figures3D &figs, const Matrix &mat);

    // Project eye transformed Vector3D point to Point2D
    Point2D doProjection(const Vector3D &eyeTransformedPoint, const double d);

    // Project eye transformed list of figures to list of Line2D objects
    Lines2D doProjection(const Figures3D &eyeTransformedFigures);
    // ?==============================================? //

}

#endif //ENGINE_FIGURE3D_H
