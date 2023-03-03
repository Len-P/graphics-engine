#include "draw2DLines.h"
#include "easy_image.h"
#include <list>
#include <cmath>



using namespace std;
using namespace img;
using namespace lsys;

img::EasyImage draw2DLines(Lines2D &lines, const int size, const Color &backgroundColor)
{
    // Calculate extrema
    double x_min = INFINITY;
    double y_min = INFINITY;
    double x_max = -INFINITY;
    double y_max = -INFINITY;

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
    int image_x = lround(size * (x_range / max(x_range, y_range)));
    int image_y = lround(size * (y_range / max(x_range, y_range)));

    // Calculate scale factor d
    double d = 0.95 * (image_x /  x_range);

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

    EasyImage image(image_x, image_y, backgroundColor);

    for (const auto &line : lines)
    {
        unsigned int x0 = lround(line.p1.x);
        unsigned int y0 = lround(line.p1.y);
        unsigned int x1 = lround(line.p2.x);
        unsigned int y1 = lround(line.p2.y);
        image.draw_line(x0, y0, x1, y1, line.color);
    }

    return image;
}
