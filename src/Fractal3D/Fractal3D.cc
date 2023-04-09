#include "Fractal3D.h"
#include "../Figure3D/Figure3D.h"



void Fractal3D::generateFractal(Figure3D::Figure &fig, Figure3D::Figures3D &fractal, const int nrIter, const double scale)
{
    for (int i = 0; i < fig.points.size(); i++)
    {
        Vector3D &point = fig.points[i];
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
    }
}