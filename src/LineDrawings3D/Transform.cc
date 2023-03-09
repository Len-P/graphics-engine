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

Point2D doProjection(const Vector3D &eyeTransformedPoint, const double d)
{
    return {-d * eyeTransformedPoint.x / eyeTransformedPoint.z, -d * eyeTransformedPoint.y / eyeTransformedPoint.z};
}

Lines2D doProjection(const Figures3D &eyeTransformedFigures)
{
    Lines2D lines;

    for (const auto &fig : eyeTransformedFigures)
    {
        vector<Point2D> points2DVector;

        // Get vector of Point2D objects instead of Vector3D objects
        for (const auto &point: fig.points)
        {
            points2DVector.emplace_back(doProjection(point, 1));
        }

        // Fill list of lines with indexes from the faces
        // Indexes are used to select points from points2DVector to create lines with
        for (const auto &face : fig.faces) {
            // First, create the line between the first and last points
            int numPoints = face.point_indexes.size();

            Point2D startPoint = points2DVector[face.point_indexes[numPoints - 1]];
            Point2D endPoint = points2DVector[face.point_indexes[0]];

            lines.emplace_back(startPoint, endPoint, fig.color);

            // Create all other lines
            if (numPoints > 2)
            {
                for (int i = 1; i < numPoints - 1; i++) {
                    startPoint = endPoint;
                    endPoint = points2DVector[face.point_indexes[i]];
                    lines.emplace_back(startPoint, endPoint, fig.color);
                }
            }
        }
    }

    return lines;
}


