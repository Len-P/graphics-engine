#include "LSystem2D.h"
#include "../utils/l_parser.h"
#include <stack>



using namespace img;
using namespace LSystem2D;

using Lines2D = vector<Line2D>;

// Used in LSystem2Lines2D() for recursively generating the vector of lines that gets returned by LSystem2Lines2D()
void recursiveLSystem(const string &str, unsigned int iter, unsigned int maxIter, double &currentAngle, const LParser::LSystem2D &l_system, Lines2D &lines, Point2D &startPoint, Point2D &endPoint, stack<tuple<Point2D, double>> &stack, const Color &color);

// Transforms LSystem to list of lines
Lines2D LSystem2Lines2D(const LParser::LSystem2D &l_system, const Color &color);

