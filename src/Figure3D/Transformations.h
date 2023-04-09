#ifndef ENGINE_TRANSFORMATIONS_H
#define ENGINE_TRANSFORMATIONS_H
#define _USE_MATH_DEFINES

#include <list>
#include <vector>
#include <cmath>
#include "../utils/vector3d.h"



using std::list;
using std::vector;

namespace LSystem2D {
    class Point2D;
    class Line2D;
    using Lines2D = vector<Line2D>;
}

namespace Figure3D {
    class Figure;
    typedef list<Figure> Figures3D;
}

class Transformations {
public:
    static void toPolar(const Vector3D &point, double &r, double &theta, double &phi);

    static Matrix scaleFigure(const double scale);
    static Matrix rotateX(const double angle);
    static Matrix rotateY(const double angle);
    static Matrix rotateZ(const double angle);
    static Matrix translate(const Vector3D &vector);
    static Matrix eyePointTrans(const Vector3D &eyepoint);

    // Apply transformation matrix to list of figures
    static void applyTransformation(Figure3D::Figures3D &figs, const Matrix &mat);

    // Project eye transformed Vector3D point to Point2D
    static LSystem2D::Point2D doProjection(const Vector3D &eyeTransformedPoint, const double d);

    // Project eye transformed list of figures to list of Line2D objects
    static LSystem2D::Lines2D doProjection(const Figure3D::Figures3D &eyeTransformedFigures);
};


#endif //ENGINE_TRANSFORMATIONS_H
