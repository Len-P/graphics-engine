#ifndef ENGINE_LSYSTEM2D_H
#define ENGINE_LSYSTEM2D_H
#define _USE_MATH_DEFINES

#include "../utils/ini_configuration.h"
#include "../utils/easy_image.h"
#include "../utils/l_parser.h"
#include <cmath>
#include <fstream>
#include <stack>



using namespace std;
using namespace ini;
using namespace img;

namespace LSystem2D {

    // Parse config ini file
    EasyImage parseIniLSystem2D(const Configuration &conf);

    class Point2D {
    public:
        double x;
        double y;

        // Default Point constructor
        Point2D() = default;

        // Point constructor
        Point2D(double aX, double aY);
    };

    class Line2D {
    public:
        Point2D p1{};
        Point2D p2{};
        Color color;

        // Default Line constructor
        Line2D() = default;

        //Line constructor
        Line2D(Point2D &aP1, Point2D &aP2, const Color &aColor);

        // Draws all lines from a list of lines on an image. Returns image. Image parameters are size and background color.
        static EasyImage draw2DLines(vector<Line2D> lines, int size, const Color &backgroundColor);
    };

    using Lines2D = vector<Line2D>;

    // Used in LSystem2Lines2D() for recursively generating the vector of lines that gets returned by LSystem2Lines2D()
    void recursiveLSystem(const string &str, unsigned int iter, unsigned int maxIter, double &currentAngle, const LParser::LSystem2D &l_system, Lines2D &lines, Point2D &startPoint, Point2D &endPoint, stack<tuple<Point2D, double>> &stack, const Color &color);

    // Transforms LSystem to list of lines
    Lines2D LSystem2Lines2D(const LParser::LSystem2D &l_system, const Color &color);

}

#endif //ENGINE_LSYSTEM2D_H
