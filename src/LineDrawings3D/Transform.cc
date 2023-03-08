#include "Transform.h"



void toPolar(const Vector3D &point, double &r, double &theta, double &phi)
{
    r = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

    if (point.y / point.x < -M_PI/2 || point.y / point.x > M_PI/2)
    {
        cout << "Out of bounds for atan2";
    }
    else
    {
        theta = atan2(point.y, point.x);
    }

    phi = acos(r);
}

Matrix scaleFigure(const double scale)
{
    Matrix matrix;
    matrix(1, 1) = scale;
    matrix(2, 2) = scale;
    matrix(3, 3) = scale;

    return matrix;
}

Matrix rotateX(const double angle)
{
    Matrix matrix;
    matrix(2, 2) = cos(angle);
    matrix(3, 3) = cos(angle);
    matrix(2, 3) = sin(angle);
    matrix(3, 2) = -sin(angle);

    return matrix;
}

Matrix rotateY(const double angle)
{
    Matrix matrix;
    matrix(1, 1) = cos(angle);
    matrix(3, 3) = cos(angle);
    matrix(3, 1) = sin(angle);
    matrix(1, 3) = -sin(angle);

    return matrix;
}

Matrix rotateZ(const double angle)
{
    Matrix matrix;
    matrix(1, 1) = cos(angle);
    matrix(2, 2) = cos(angle);
    matrix(1, 2) = sin(angle);
    matrix(2, 1) = -sin(angle);

    return matrix;
}

Matrix translate(const Vector3D &vector)
{
    Matrix matrix;
    matrix(4, 1) = vector.x;
    matrix(4, 2) = vector.y;
    matrix(4, 3) = vector.z;

    return matrix;
}

Matrix eyePointTrans(const Vector3D &eyepoint)
{
    double r;
    double theta;
    double phi;
    toPolar(eyepoint, r, theta, phi);

    Matrix matrix;
    matrix(1, 1) = -sin(theta);
    matrix(1, 2) = -cos(theta) * cos(phi);
    matrix(1, 3) = cos(theta) * sin(phi);
    matrix(2,1) = cos(theta);
    matrix(2, 2) = -sin(theta) * cos(phi);
    matrix(2, 3) = sin(theta) * sin(phi);
    matrix(3, 2) = sin(phi);
    matrix(3, 3) = cos(phi);
    matrix(4, 3) = -r;

    return matrix;
}

void applyTransformation(Figure &fig, const Matrix &mat)
{
    for (auto &point : fig.points)
    {
        point = point * mat;
    }
}

void applyTransformation(Figures3D &figs, const Matrix &mat)
{
    for (auto &fig : figs)
    {
        applyTransformation(fig, mat);
    }
}

Point2D doProjection(const Vector3D &point, const double d)
{

}

Lines2D doProjection(const Figures3D &figs)
{

}


