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

        // Fractals or normal functionality
        string figType = conf[figName]["type"].as_string_or_die();
        int nrIter = conf[figName]["nrIterations"].as_int_or_default(0);

        if (nrIter > 0 && figType.substr(0, 7) == "Fractal")
        {
            double fractalScale = conf[figName]["fractalScale"].as_double_or_default(1);
            Fractal3D::generateFractal(figure, figures, nrIter, fractalScale);
        }
        else if (figType == "MengerSponge")
        {
            Figure3D::Figure::createMengerSponge(figure, figures, nrIter, 1);
        }
        else
        {
            figures.emplace_back(figure);
        }
    }

    // ?==== Eye Point Transformation and Projection ====? //
    Transformations::applyTransformation(figures, Transformations::eyePointTrans(eyePoint));
    Lines2D lines = Transformations::doProjection(figures);

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
    double d = 0.95 * image_x /  x_range;

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

    Vector3D G = Vector3D::point((A_2D.x + B_2D.x + C_2D.x)/3, (A_2D.y + B_2D.y + C_2D.y)/3, 1/(3*A.z) + 1/(3*B.z) + 1/(3*C.z) );

    Vector3D u = B - A;
    Vector3D v = C - A;
    Vector3D w = Vector3D::cross(u, v);

    double k = w.x * A.x + w.y * A.y + w.z * A.z;
    double dzdx = w.x / (-d * k);
    double dzdy = w.y / (-d * k);

    double y_min = min(A_2D.y, min(B_2D.y, C_2D.y));
    double y_max = max(A_2D.y, max(B_2D.y, C_2D.y));

    int y_minInt = lround(y_min + 0.5);
    int y_maxInt = lround(y_max - 0.5);

    for (int yI = y_minInt; yI < y_maxInt + 1; yI++)
    {
        double ABxL = numeric_limits<double>::infinity();
        double ACxL = numeric_limits<double>::infinity();
        double BCxL = numeric_limits<double>::infinity();

        double ABxR = -numeric_limits<double>::infinity();
        double ACxR = -numeric_limits<double>::infinity();
        double BCxR = -numeric_limits<double>::infinity();

        calculateIntermediateX_LandR(A_2D, B_2D, yI, ABxL, ABxR);
        calculateIntermediateX_LandR(C_2D, A_2D, yI, ACxL, ACxR);
        calculateIntermediateX_LandR(B_2D, C_2D, yI, BCxL, BCxR);

        int xL = lround(min(ABxL, min(BCxL, ACxL)) + 0.5);
        int xR = lround(max(ABxR, max(BCxR, ACxR)) - 0.5);

        for (int i = xL; i < xR + 1; i ++)
        {
            double z = (1.0001 * G.z) + (i - G.x) * dzdx + (yI - G.y) * dzdy;

            if (z < zbuf[i][yI])
            {
                zbuf[i][yI] = z;
                image(i, yI) = color;
            }
        }
    }
}

void ZBuffering::calculateIntermediateX_LandR(const Point2D &P, const Point2D &Q, const int &yI, double &PQxL, double &PQxR)
{
    if ( ( (double) yI - P.y) * ( (double) yI - Q.y) <= 0 && P.y != Q.y )
    {
        double xI = Q.x + (P.x - Q.x) * ( (double) yI - Q.y) / (P.y - Q.y);
        PQxL = min(PQxL, xI);
        PQxR = max(PQxR, xI);
    }
}
