#ifndef ENGINE_LSYSTEM2D_H
#define ENGINE_LSYSTEM2D_H

#include "../utils/ini_configuration.h"
#include "../utils/easy_image.h"
#include "../utils/l_parser.h"
#include <cmath>
#include <fstream>
#include "LSystem2Lines2D.h"
#include "draw2DLines.h"



using namespace std;
using namespace ini;
using namespace img;

namespace LSystem2D {

    class Point2D {
    public:
        double x;
        double y;

        Point2D() = default;
        Point2D(double aX, double aY);
    };

    class Line2D {
    public:
        Point2D p1{};
        Point2D p2{};
        Color color;

        Line2D() = default;
        Line2D(Point2D &aP1, Point2D &aP2, const Color &aColor);
    };

}

// Parse config ini file
EasyImage parseIniLSystem2D(const Configuration &conf);

#endif //ENGINE_LSYSTEM2D_H
