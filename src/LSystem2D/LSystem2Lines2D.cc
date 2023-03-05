#define _USE_MATH_DEFINES
#include "LSystem2Lines2D.h"
#include "../utils/l_parser.h"
#include <cmath>
#include <stack>


using namespace std;
using namespace img;
using namespace LSystem2D;

void recursiveLSystem(const string &str, unsigned int iter, const unsigned int maxIter, double &currentAngle, const LParser::LSystem2D &l_system, Lines2D &lines, Point2D &startPoint, Point2D &endPoint, stack<tuple<Point2D, double>> &stack, const Color &color)
{
    const double angle = l_system.get_angle() * M_PI/180;

    for (char c : str)
    {
        // If character is not in alphabet (stop condition 1)
        if (l_system.get_alphabet().find(c) == l_system.get_alphabet().end())
        {
            switch(c) {
                case '+':
                    currentAngle += angle;
                    break;
                case '-':
                    currentAngle -= angle;
                    break;
                case '(':  // Save current point and angle in stack
                    stack.emplace(endPoint, currentAngle);
                    break;
                case ')':  // Teleport back to last point with last angle
                    tuple<Point2D, double> tuple = stack.top();
                    endPoint = get<0>(tuple);
                    currentAngle = get<1>(tuple);
                    stack.pop();
                    break;
            }

        }
        // If max depth has been reached (stop condition 2)
        else if (iter == maxIter)
        {
            startPoint = endPoint;

            endPoint.x = startPoint.x + cos(currentAngle);
            endPoint.y = startPoint.y + sin(currentAngle);

            if (l_system.draw(c))
            {
                lines.emplace_back(startPoint, endPoint, color);
            }

        }
        // Keep going deeper with replacement rules
        else
        {
            const string &replacement = l_system.get_replacement(c);
            recursiveLSystem(replacement, iter+1, maxIter, currentAngle, l_system, lines, startPoint, endPoint, stack, color);
        }

    }

}

Lines2D LSystem2Lines2D(const LParser::LSystem2D &l_system, const Color &color)
{
    // Parse .L2D file
    const string &initiator = l_system.get_initiator();
    double startingAngle = l_system.get_starting_angle() * M_PI/180;
    const unsigned int iterations = l_system.get_nr_iterations();

    // Initialize necessary objects
    Lines2D lines;

    Point2D startPoint = Point2D();
    Point2D endPoint = Point2D();

    stack<tuple<Point2D, double>> stack;

    // Fill lines vector and return it
    recursiveLSystem(initiator, 0, iterations, startingAngle, l_system, lines, startPoint, endPoint, stack, color);
    return lines;
}


