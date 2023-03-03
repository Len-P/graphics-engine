#ifndef ENGINE_LSYSTEMS_H
#define ENGINE_LSYSTEMS_H

#include "../utils/easy_image.h"



using namespace std;

namespace lsys {

    class parseIni {
    public:
        string type;
        int size;

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
        img::Color color;

        Line2D() = default;
        Line2D(Point2D &aP1, Point2D &aP2, const img::Color& aColor);
    };





}

#endif //ENGINE_LSYSTEMS_H
