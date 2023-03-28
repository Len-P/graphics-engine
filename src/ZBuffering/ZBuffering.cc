#include "ZBuffering.h"



EasyImage ZBuffering::parseIni(const Configuration &conf)
{
    // ?============== General ==============? //
    int size = conf["General"]["size"].as_int_or_die();

    vector<double> backgroundColorTuple = conf["General"]["backgroundcolor"].as_double_tuple_or_die();
    Color backgroundColor = Color(lround(backgroundColorTuple[0] * 255), lround(backgroundColorTuple[1] * 255), lround(backgroundColorTuple[2] * 255));

    int nrFigures = conf["General"]["nrFigures"].as_int_or_die();

    vector<double> eyeCoord = conf["General"]["eye"].as_double_tuple_or_die();
    Vector3D eyePoint = Vector3D::point(eyeCoord[0], eyeCoord[1], eyeCoord[2]);

    // ?============== Figures ==============? //
    Figures3D figures;

    for (int i = 0; i < nrFigures; i++)
    {
        string figName = "Figure" + to_string(i);

        Figure figure = Figure::generateFigure(conf, figName, true);

        figures.emplace_back(figure);
    }

    // ?==== Eye Point Transformation and Projection ====? //
    applyTransformation(figures, eyePointTrans(eyePoint));
    Lines2D lines = doProjection(figures);

    // ?============== Draw Image ==============? //
    return draw_zbuf_figures(figures, lines, size, backgroundColor);

}

EasyImage ZBuffering::draw_zbuf_figures(const Figures3D &figures, const Lines2D &lines, const int size, const Color &backgroundColor)
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
    double d = 0.95 * (image_x /  x_range);

    // Calculate translation distances dx and dy
    double DC_x = d * (x_min + x_max)/2;
    double DC_y = d * (y_min + y_max)/2;
    double dx = image_x/2 - DC_x;
    double dy = image_y/2 - DC_y;

    image_x = lround(image_x);
    image_y = lround(image_y);

    // Create image object and Z-Buffer
    EasyImage image((int) image_x, (int) image_y, backgroundColor);
    ZBuffer buffer = ZBuffer((int) image_x, (int) image_y);

    for (const auto &fig : figures)
    {
        for (const auto &face : fig.faces)
        {
            Vector3D A = fig.points[face.pointIndexes[0]];
            Vector3D B = fig.points[face.pointIndexes[1]];
            Vector3D C = fig.points[face.pointIndexes[2]];

            draw_zbuf_triangle(buffer, image, A, B, C, d, dx, dy, fig.color);
        }
    }

    return image;
}

void ZBuffering::draw_zbuf_triangle(ZBuffer &zbuf, EasyImage &image, const Vector3D &A, const Vector3D &B, const Vector3D &C, double d, double dx, double dy, const Color &color)
{
    Point2D A_2D = Point2D(-d * A.x / A.z + dx, -d * A.y / A.z + dy);
    Point2D B_2D = Point2D(-d * B.x / B.z + dx, -d * B.y / B.z + dy);
    Point2D C_2D = Point2D(-d * C.x / C.z + dx, -d * C.y / C.z + dy);
    vector<Point2D> points = {A_2D, B_2D, C_2D};

    double y_min = numeric_limits<double>::infinity();
    double y_max = -numeric_limits<double>::infinity();

    // Iterate over all points and update min and max values
    for (const auto &point : points) // check 0.5?? cursus p.38
    {
        y_min = min(y_min, point.y);
        y_max = max(y_max, point.y);
    }

    int y_minInt = lround(y_min);
    int y_maxInt = lround(y_max);

}