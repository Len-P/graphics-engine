#include "LSystem2D.h"
#include <cmath>
#include "../utils/l_parser.h"
#include <fstream>
#include "LSystem2Lines2D.h"
#include "draw2DLines.h"



EasyImage parseIniLSystem2D(const Configuration &conf)
{
    // Parsing ini file
    int size = conf["General"]["size"].as_int_or_die();

    vector<double> backgroundColorTuple = conf["General"]["backgroundcolor"].as_double_tuple_or_die();
    Color backgroundColor = Color(lround(backgroundColorTuple[0] * 255), lround(backgroundColorTuple[1] * 255), lround(backgroundColorTuple[2] * 255));

    string inputFile = conf["2DLSystem"]["inputfile"].as_string_or_die();

    vector<double> colorTuple = conf["2DLSystem"]["color"].as_double_tuple_or_die();
    Color color = Color(lround(colorTuple[0] * 255), lround(colorTuple[1] * 255), lround(colorTuple[2] * 255));

    // Create 2D LSystem from input file
    LParser::LSystem2D l_system;
    ifstream input_stream(inputFile);
    input_stream >> l_system;
    input_stream.close();

    // Get vector with lines from LSystem and then draw lines and return image
    Lines2D lines = LSystem2Lines2D(l_system, color);
    return draw2DLines(lines, size, backgroundColor);
}

// Point constructor
LSystem2D::Point2D::Point2D(double aX, double aY)
{
    x = aX;
    y = aY;
}

// Line constructor
LSystem2D::Line2D::Line2D(Point2D &aP1, Point2D &aP2, const img::Color &aColor)
{
    p1 = aP1;
    p2 = aP2;
    color = aColor;
}