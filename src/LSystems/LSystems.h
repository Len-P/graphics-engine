#ifndef ENGINE_LSYSTEMS_H
#define ENGINE_LSYSTEMS_H

#include "../utils/ini_configuration.h"
#include "../utils/easy_image.h"



using namespace std;
using namespace ini;
using namespace img;

namespace lsys {

    class parseIni {
    public:
        int size;
        Color backgroundColor;
        string inputFile;
        Color color;

        parseIni(const Configuration &conf);
    };

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

#endif //ENGINE_LSYSTEMS_H
