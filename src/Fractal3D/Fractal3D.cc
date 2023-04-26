#include "Fractal3D.h"
#include "../Figure3D/Figure3D.h"



void Fractal3D::generateFractal(Figure3D::Figure &fig, Figure3D::Figures3D &fractal, const int nrIter, const double scale)
{
    vector<Vector3D> donePoints; // BuckyBalls have duplicate points, which result in incorrect fractals

    for (int i = 0; i < fig.points.size(); i++)
    {
        Vector3D &point = fig.points[i];

        donePoints.emplace_back(point);

        Figure3D::Figure figCopy = fig;

        figCopy.applyTransformation(Transformations::scaleFigure(1 / scale));
        figCopy.applyTransformation(Transformations::translate(point - figCopy.points[i]));

        if (nrIter == 1)
        {
            fractal.emplace_back(figCopy);
        }
        else
        {
            generateFractal(figCopy, fractal, nrIter - 1, scale);
        }

        donePoints.emplace_back(point);
    }
}