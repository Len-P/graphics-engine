#ifndef ENGINE_TRANSFORM_H
#define ENGINE_TRANSFORM_H
#define _USE_MATH_DEFINES

#include "../utils/vector3d.h"
#include "Figure3D.h"
#include <cmath>
#include "../LSystem2D/LSystem2D.h"



using namespace Figure3D;
using namespace LSystem2D;
using Lines2D = vector<Line2D>;

void toPolar(const Vector3D &point, double &r, double &theta, double &phi);

Matrix scaleFigure(const double scale);
Matrix rotateX(const double angle);
Matrix rotateY(const double angle);
Matrix rotateZ(const double angle);
Matrix translate(const Vector3D &vector);
Matrix eyePointTrans(const Vector3D &eyepoint);

void applyTransformation(Figure &fig, const Matrix &mat);
void applyTransformation(Figures3D &figs, const Matrix &mat);

Point2D doProjection(const Vector3D &point, const double d);
Lines2D doProjection(const Figures3D &figs);

#endif //ENGINE_TRANSFORM_H
