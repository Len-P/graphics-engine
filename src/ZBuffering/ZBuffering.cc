#include "ZBuffering.h"



EasyImage ZBuffering::parseIni(const Configuration &conf, const bool lighted)
{
    // General
    int size = conf["General"]["size"].as_int_or_die();

    vector<double> backgroundColorTuple = conf["General"]["backgroundcolor"].as_double_tuple_or_die();
    ColorDouble backgroundColor = ColorDouble(backgroundColorTuple);

    vector<double> eyeCoord = conf["General"]["eye"].as_double_tuple_or_die();
    Vector3D eyePoint = Vector3D::point(eyeCoord[0], eyeCoord[1], eyeCoord[2]);
    Matrix eyeMat = Transformations::eyePointTrans(eyePoint);

    // Lights
    Lights3D lights = Light::parseLights(conf, lighted, eyeMat);

    // Figures
    Figures3D figures = Figure::parseFigures(conf, true, lighted);

    // Calculate shadow masks if applicable
    bool shadowing = conf["General"]["shadowEnabled"].as_bool_or_default(false);

    for (auto &light : lights)
    {
        light.calculateShadowMask(figures);
    }

    // Eye Point Transformation and Projection
    Transformations::applyTransformation(figures, eyeMat);
    Lines2D lines = Transformations::doProjection(figures);

    // Draw Image
    return draw_zbuf_figures(figures, lines, size, backgroundColor, lights, eyeMat, shadowing);
}

EasyImage ZBuffering::draw_zbuf_figures(Figures3D &figures, const Lines2D &lines, const int size, const ColorDouble &backgroundColor, Lights3D &lights, Matrix &eyeMat, const bool &shadowing)
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
    EasyImage image((int) image_x, (int) image_y, ColorDouble::getIntColor(backgroundColor));
    ZBuffer buffer = ZBuffer((int) image_x, (int) image_y);

    for (auto &fig : figures)
    {
        for (const auto &face : fig.faces)
        {
            Vector3D A = fig.points[face.pointIndexes[0]];
            Vector3D B = fig.points[face.pointIndexes[1]];
            Vector3D C = fig.points[face.pointIndexes[2]];

            draw_zbuf_triangle(buffer, image, A, B, C, d, dx, dy, fig.ambientReflection, fig.diffuseReflection, fig.specularReflection, fig.reflectionCoefficient, lights, eyeMat, shadowing);
        }
    }

    return image;
}

void ZBuffering::draw_zbuf_triangle(ZBuffer &zbuf, EasyImage &image, const Vector3D &A, const Vector3D &B, const Vector3D &C, double d, double dx, double dy, const ColorDouble &ambientReflection, const ColorDouble &diffuseReflection, const ColorDouble &specularReflection, const double reflectionCoeff, Lights3D &lights, Matrix &eyeMat, const bool &shadowing)
{
    Point2D A_2D = Point2D(-d * A.x / A.z + dx, -d * A.y / A.z + dy);
    Point2D B_2D = Point2D(-d * B.x / B.z + dx, -d * B.y / B.z + dy);
    Point2D C_2D = Point2D(-d * C.x / C.z + dx, -d * C.y / C.z + dy);

    Vector3D G = Vector3D::point((A_2D.x + B_2D.x + C_2D.x)/3, (A_2D.y + B_2D.y + C_2D.y)/3, 1/(3*A.z) + 1/(3*B.z) + 1/(3*C.z) );

    Vector3D u = B - A;
    Vector3D v = C - A;
    Vector3D w = Vector3D::cross(u, v);

    double k = w.x * A.x + w.y * A.y + w.z * A.z;
    if (k > 0) {return;} // Backface culling
    double dzdx = w.x / (-d * k);
    double dzdy = w.y / (-d * k);

    // *********** Ambient and Inf Diffuse Lights ***********

    w.normalise();
    Light total = Light::totalAmbientAndInfDiffuse(lights, w);

    total.ambientLight = ColorDouble::multiply(total.ambientLight, ambientReflection);
    total.diffuseLight = ColorDouble::multiply(total.diffuseLight, diffuseReflection);

    ColorDouble totalColor = ColorDouble::add(total.ambientLight, total.diffuseLight);

    // ******************************************************

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

        LSystem2D::Line2D::calculateIntermediateX_LandR(A_2D, B_2D, yI, ABxL, ABxR);
        LSystem2D::Line2D::calculateIntermediateX_LandR(C_2D, A_2D, yI, ACxL, ACxR);
        LSystem2D::Line2D::calculateIntermediateX_LandR(B_2D, C_2D, yI, BCxL, BCxR);

        int xL = lround(min(ABxL, min(BCxL, ACxL)) + 0.5);
        int xR = lround(max(ABxR, max(BCxR, ACxR)) - 0.5);

        for (int i = xL; i < xR + 1; i ++)
        {
            double z = (1.0001 * G.z) + (i - G.x) * dzdx + (yI - G.y) * dzdy;

            if (shadowing)
            {
                z = G.z + (i - G.x) * dzdx + (yI - G.y) * dzdy;
            }

            if (z < zbuf[i][yI])
            {
                zbuf[i][yI] = z;
                image(i, yI) = Light::totalColor(lights, w, diffuseReflection, specularReflection, reflectionCoeff, totalColor, i, yI, d, dx, dy, 1/z, eyeMat);
            }
        }
    }
}
