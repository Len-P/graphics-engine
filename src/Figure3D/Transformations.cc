#include "Transformations.h"
#include "../LSystem2D/LSystem2D.h"
#include "../Figure3D/Figure3D.h"



void Transformations::toPolar(const Vector3D &point, double &r, double &theta, double &phi)
{
    r = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    theta = atan2(point.y, point.x);
    phi = acos(point.z / r);
}

Matrix Transformations::scaleFigure(const double scale)
{
    Matrix matrix;
    matrix(1, 1) = scale;
    matrix(2, 2) = scale;
    matrix(3, 3) = scale;

    return matrix;
}

Matrix Transformations::rotateX(const double angle)
{
    Matrix matrix;
    matrix(2, 2) = cos(angle);
    matrix(3, 3) = cos(angle);
    matrix(2, 3) = sin(angle);
    matrix(3, 2) = -sin(angle);

    return matrix;
}

Matrix Transformations::rotateY(const double angle)
{
    Matrix matrix;
    matrix(1, 1) = cos(angle);
    matrix(3, 3) = cos(angle);
    matrix(3, 1) = sin(angle);
    matrix(1, 3) = -sin(angle);

    return matrix;
}

Matrix Transformations::rotateZ(const double angle)
{
    Matrix matrix;
    matrix(1, 1) = cos(angle);
    matrix(2, 2) = cos(angle);
    matrix(1, 2) = sin(angle);
    matrix(2, 1) = -sin(angle);

    return matrix;
}

Matrix Transformations::translate(const Vector3D &vector)
{
    Matrix matrix;
    matrix(4, 1) = vector.x;
    matrix(4, 2) = vector.y;
    matrix(4, 3) = vector.z;

    return matrix;
}

Matrix Transformations::eyePointTrans(const Vector3D &eyepoint)
{
    double r;
    double theta;
    double phi;
    toPolar(eyepoint, r, theta, phi);

    Matrix matrix;
    matrix(1, 1) = -sin(theta);
    matrix(1, 2) = -cos(theta) * cos(phi);
    matrix(1, 3) = cos(theta) * sin(phi);
    matrix(2, 1) = cos(theta);
    matrix(2, 2) = -sin(theta) * cos(phi);
    matrix(2, 3) = sin(theta) * sin(phi);
    matrix(3, 2) = sin(phi);
    matrix(3, 3) = cos(phi);
    matrix(4, 3) = -r;

    return matrix;
}

void Transformations::applyTransformation(Figure3D::Figures3D &figs, const Matrix &mat)
{
    for (auto &fig : figs)
    {
        fig.applyTransformation(mat);
    }
}

Point2D Transformations::doProjection(const Vector3D &eyeTransformedPoint, const double d)
{
    return {-d * eyeTransformedPoint.x / eyeTransformedPoint.z, -d * eyeTransformedPoint.y / eyeTransformedPoint.z, eyeTransformedPoint.z};
}

Lines2D Transformations::doProjection(const Figure3D::Figures3D &eyeTransformedFigures)
{
    Lines2D lines;

    for (const auto &fig : eyeTransformedFigures)
    {
        vector<Point2D> points2DVector;

        // Get vector of Point2D objects instead of Vector3D objects
        for (const auto &point: fig.points)
        {
            points2DVector.emplace_back(Transformations::doProjection(point, 1));
        }

        // Fill list of lines with indexes from the faces
        // Indexes are used to select points from points2DVector to create lines with
        for (const auto &face : fig.faces)
        {
            // First create the line between the first and last points
            int numPoints = face.pointIndexes.size();

            Point2D startPoint = points2DVector[face.pointIndexes[numPoints - 1]];
            Point2D endPoint = points2DVector[face.pointIndexes[0]];

            lines.emplace_back(startPoint, endPoint, fig.color, startPoint.z, endPoint.z);

            // Create all other lines
            if (numPoints > 2)
            {
                for (int i = 1; i < numPoints; i++)
                {
                    startPoint = endPoint;
                    endPoint = points2DVector[face.pointIndexes[i]];
                    lines.emplace_back(startPoint, endPoint, fig.color, startPoint.z, endPoint.z);
                }
            }
        }
    }

    return lines;
}