#ifndef ENGINE_FIGURE3D_H
#define ENGINE_FIGURE3D_H
#define _USE_MATH_DEFINES

#include "../utils/ini_configuration.h"
#include "../utils/easy_image.h"
#include "../utils/vector3d.h"
#include "../LSystem2D/LSystem2D.h"
#include <list>
#include <cmath>



using namespace std;
using namespace ini;
using namespace img;
using namespace LSystem2D;

namespace Figure3D {

    class Face {
    public:
        vector<int> pointIndexes; // Indexes refer to points in the points vector in the Figure class

        // Face constructor
        explicit Face(vector<int> aPointIndexes);
    };

    class Figure {
    public:
        vector<Vector3D> points;
        vector<Face> faces;
        Color color;

        // Figure constructor
        Figure(vector<Vector3D> &aPoints, vector<Face> &aFaces, const Color &aColor);

        // Apply transformation matrix to figure
        void applyTransformation(const Matrix &mat);
    };

    typedef list<Figure> Figures3D;

    // Parse config ini file
    EasyImage parseIniFigure3D(const Configuration &conf);

    // ============== Transformations ============== //
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
    // ============================================== //

}

#endif //ENGINE_FIGURE3D_H
