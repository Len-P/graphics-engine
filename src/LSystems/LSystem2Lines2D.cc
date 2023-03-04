#define _USE_MATH_DEFINES
#include "LSystem2Lines2D.h"
#include "../utils/l_parser.h"
#include <cmath>
#include <stack>


using namespace std;
using namespace img;
using namespace LSystem2D;

void recursiveLSystem(const string &str, unsigned int iter, unsigned int maxIter, double &currentAngle, const LParser::LSystem2D &l_system, Lines2D &lines, Point2D &startPoint, Point2D &endPoint, stack<tuple<Point2D, double>> &stack, const Color &color);

Lines2D LSystem2Lines2D(const LParser::LSystem2D &l_system, const Color &color)
{
    const string &initiator = l_system.get_initiator();
    double startingAngle = l_system.get_starting_angle() * M_PI/180;
    const unsigned int iterations = l_system.get_nr_iterations();

    Lines2D lines;

    Point2D startPoint = Point2D();
    Point2D endPoint = Point2D();

    stack<tuple<Point2D, double>> stack;

    recursiveLSystem(initiator, 0, iterations, startingAngle, l_system, lines, startPoint, endPoint, stack, color);

    return lines;
}

void recursiveLSystem(const string &str, unsigned int iter, const unsigned int maxIter, double &currentAngle, const LParser::LSystem2D &l_system, Lines2D &lines, Point2D &startPoint, Point2D &endPoint, stack<tuple<Point2D, double>> &stack, const Color &color)
{
    const double angle = l_system.get_angle() * M_PI/180;

    for (char c : str)
    {
        if (l_system.get_alphabet().find(c) == l_system.get_alphabet().end())
        {
            switch(c) {
                case '+':
                    currentAngle += angle;
                    break;
                case '-':
                    currentAngle -= angle;
                    break;
                case '(':
                    stack.emplace(endPoint, currentAngle);
                    break;
                case ')':
                    tuple<Point2D,double> tuple = stack.top();
                    endPoint = get<0>(tuple);
                    currentAngle = get<1>(tuple);
                    stack.pop();
                    break;
            }

        }
        else if (iter == maxIter)
        {
            startPoint = endPoint;

            endPoint.x = startPoint.x + cos(currentAngle);
            endPoint.y = startPoint.y + sin(currentAngle);

            if (l_system.draw(c))
            {
                lines.emplace_back(startPoint, endPoint, color);
            }

        } else
        {
            const string &replacement = l_system.get_replacement(c);
            recursiveLSystem(replacement, iter+1, maxIter, currentAngle, l_system, lines, startPoint, endPoint, stack, color);
        }

    }

}

