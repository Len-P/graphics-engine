#include "LSystem2D.h"



// ?========================================== Parse Ini ==========================================? //
EasyImage LSystem2D::parseIni(const Configuration &conf)
{
    // Parsing ini file
    int size = conf["General"]["size"].as_int_or_die();

    vector<double> backgroundColorTuple = conf["General"]["backgroundcolor"].as_double_tuple_or_die();
    Color backgroundColor = Color(backgroundColorTuple);

    string inputFile = conf["2DLSystem"]["inputfile"].as_string_or_die();

    vector<double> colorTuple = conf["2DLSystem"]["color"].as_double_tuple_or_die();
    Color color = Color(colorTuple);

    // Create 2D LSystem from input file
    LParser::LSystem2D l_system;
    std::ifstream input_stream(inputFile);
    input_stream >> l_system;
    input_stream.close();

    // Get vector with lines from LSystem and then draw lines and return image
    Lines2D lines = LSystemToLines2D(l_system, color);
    return Line2D::draw2DLines(lines, size, backgroundColor);
}

// ?========================================= Class Constructors =========================================? //
LSystem2D::Point2D::Point2D(double aX, double aY, double aZ)
{
    x = aX;
    y = aY;
    z = aZ;
}

LSystem2D::Line2D::Line2D(Point2D &aP1, Point2D &aP2, const Color &aColor, double aZ0, double aZ1)
{
    p1 = aP1;
    p2 = aP2;
    color = aColor;
    z0 = aZ0;
    z1 = aZ1;
}

// ?=========================================== Static Methods ===========================================? //
EasyImage LSystem2D::Line2D::draw2DLines(vector<Line2D> &lines, const int size, const Color &backgroundColor, const bool ZBuffering)
{
    double x_min = numeric_limits<double>::infinity();
    double y_min = numeric_limits<double>::infinity();
    double x_max = -numeric_limits<double>::infinity();
    double y_max = -numeric_limits<double>::infinity();

    // Iterate over all lines and update min and max values
    for (const auto &line : lines)
    {
        x_min = min(x_min, min(line.p1.x, line.p2.x));
        y_min = min(y_min, min(line.p1.y, line.p2.y));
        x_max = max(x_max, max(line.p1.x, line.p2.x));
        y_max = max(y_max, max(line.p1.y, line.p2.y));
    }

    // Calculate image resolution
    double x_range = x_max - x_min;
    double y_range = y_max - y_min;
    double image_x = size * (x_range / max(x_range, y_range));
    double image_y = size * (y_range / max(x_range, y_range));

    // Calculate scale factor d
    double d = 0.95 * image_x /  x_range;

    // Calculate translation distances dx and dy
    double DC_x = d * (x_min + x_max)/2;
    double DC_y = d * (y_min + y_max)/2;
    double dx = image_x/2 - DC_x;
    double dy = image_y/2 - DC_y;

    // Scale all points with scale factor d, then apply translations dx and dy
    for (Line2D &line : lines)
    {
        line.p1.x *= d;
        line.p1.x += dx;
        line.p1.y *= d;
        line.p1.y += dy;

        line.p2.x *= d;
        line.p2.x += dx;
        line.p2.y *= d;
        line.p2.y += dy;
    }

    image_x = lround(image_x);
    image_y = lround(image_y);

    // Create image object and Z-Buffer
    EasyImage image((int) image_x, (int) image_y, backgroundColor);
    ZBuffer buffer = ZBuffer((int) image_x, (int) image_y);

    // Draw all lines on the image
    for (const auto &line : lines)
    {
        unsigned int x0 = lround(line.p1.x);
        unsigned int y0 = lround(line.p1.y);
        unsigned int x1 = lround(line.p2.x);
        unsigned int y1 = lround(line.p2.y);

        if (ZBuffering)
        {
            image.draw_zbuf_line(buffer, x0, y0, line.z0, x1, y1, line.z1, line.color);
        }
        else
        {
            image.draw_line(x0, y0, x1, y1, line.color);
        }
    }

    return image;
}

void LSystem2D::Line2D::calculateIntermediateX_LandR(const Point2D &P, const Point2D &Q, const int &yI, double &PQxL, double &PQxR)
{
    if ( ( (double) yI - P.y) * ( (double) yI - Q.y) <= 0 && P.y != Q.y )
    {
        double xI = Q.x + (P.x - Q.x) * ( (double) yI - Q.y) / (P.y - Q.y);
        PQxL = min(PQxL, xI);
        PQxR = max(PQxR, xI);
    }
}

// ?=========================================== Functions ===========================================? //
void LSystem2D::recursiveLSystem(const string &str, unsigned int iter, const unsigned int maxIter, double &currentAngle, const LParser::LSystem2D &l_system, Lines2D &lines, LSystem2D::Point2D &startPoint, LSystem2D::Point2D &endPoint, stack<tuple<LSystem2D::Point2D, double>> &stack, const Color &color)
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
                    tuple<LSystem2D::Point2D, double> tuple = stack.top();
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

LSystem2D::Lines2D LSystem2D::LSystemToLines2D(const LParser::LSystem2D &l_system, const Color &color)
{
    // Parse .L2D file
    const string &initiator = l_system.get_initiator();
    double startingAngle = l_system.get_starting_angle() * M_PI/180;
    const unsigned int iterations = l_system.get_nr_iterations();

    // Initialize necessary objects
    Lines2D lines;

    LSystem2D::Point2D startPoint = LSystem2D::Point2D();
    LSystem2D::Point2D endPoint = LSystem2D::Point2D();

    stack<tuple<LSystem2D::Point2D, double>> stack;

    // Fill lines vector and return it
    recursiveLSystem(initiator, 0, iterations, startingAngle, l_system, lines, startPoint, endPoint, stack, color);
    return lines;
}


