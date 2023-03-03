#include "LSystems.h"



//Point constructor
lsys::Point2D::Point2D(double aX, double aY)
{
    x = aX;
    y = aY;
}

//Line constructor
lsys::Line2D::Line2D(Point2D &aP1, Point2D &aP2, const img::Color &aColor)
{
    p1 = aP1;
    p2 = aP2;
    color = aColor;
}