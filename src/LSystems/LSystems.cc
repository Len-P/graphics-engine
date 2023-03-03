#include "LSystems.h"
#include <cmath>


// Parse config ini file
lsys::parseIni::parseIni(const Configuration &conf)
{
    size = conf["General"]["size"].as_int_or_die();

    vector<double> backgroundColorTuple = conf["General"]["backgroundcolor"].as_double_tuple_or_die();
    backgroundColor = Color(lround(backgroundColorTuple[0] * 255), lround(backgroundColorTuple[1] * 255), lround(backgroundColorTuple[2] * 255));

    inputFile = conf["2DLSystem"]["inputfile"].as_string_or_die();

    vector<double> colorTuple = conf["2DLSystem"]["color"].as_double_tuple_or_die();
    color = Color(lround(colorTuple[0] * 255), lround(colorTuple[1] * 255), lround(colorTuple[2] * 255));
}

// Point constructor
lsys::Point2D::Point2D(double aX, double aY)
{
    x = aX;
    y = aY;
}

// Line constructor
lsys::Line2D::Line2D(Point2D &aP1, Point2D &aP2, const img::Color &aColor)
{
    p1 = aP1;
    p2 = aP2;
    color = aColor;
}