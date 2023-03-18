#include "Figure3D.h"



// ========================================== Parse Ini ========================================== //
EasyImage Figure3D::parseIniWireframe(const Configuration &conf)
{
    // ============== General ============== //
    int size = conf["General"]["size"].as_int_or_die();

    vector<double> backgroundColorTuple = conf["General"]["backgroundcolor"].as_double_tuple_or_die();
    Color backgroundColor = Color(lround(backgroundColorTuple[0] * 255), lround(backgroundColorTuple[1] * 255), lround(backgroundColorTuple[2] * 255));

    int nrFigures = conf["General"]["nrFigures"].as_int_or_die();

    vector<double> eyeCoord = conf["General"]["eye"].as_double_tuple_or_die();
    Vector3D eyePoint = Vector3D::point(eyeCoord[0], eyeCoord[1], eyeCoord[2]);

    // ============== Figures ============== //
    Figures3D figures;

    for (int i = 0; i < nrFigures; i++)
    {
        Figure figure = Figure();

        string figName = "Figure" + to_string(i);
        string figType = conf[figName]["type"].as_string_or_die();

        // Transformations
        double rotXAngle = conf[figName]["rotateX"].as_double_or_default(0) * M_PI/180;
        double rotYAngle = conf[figName]["rotateY"].as_double_or_default(0) * M_PI/180;
        double rotZAngle = conf[figName]["rotateZ"].as_double_or_default(0) * M_PI/180;
        double scale = conf[figName]["scale"].as_double_or_default(1);

        vector<double> centerTuple = conf[figName]["center"].as_double_tuple_or_die();
        Vector3D center = Vector3D::point(-centerTuple[0], -centerTuple[1], -centerTuple[2]); // Negative elements to center at (0, 0, 0)
        Matrix transformMatrix = translate(center) * scaleFigure(scale) * rotateX(rotXAngle) * rotateY(rotYAngle) * rotateZ(rotZAngle);

        // Figure color
        vector<double> colorTuple = conf[figName]["color"].as_double_tuple_or_die();
        Color color = Color(lround(colorTuple[0] * 255), lround(colorTuple[1] * 255), lround(colorTuple[2] * 255));

        // Check figType and generate correct figure
        if (figType == "LineDrawing")
        {
            // Lines and points
            int nrPoints = conf[figName]["nrPoints"].as_int_or_die();
            int nrLines = conf[figName]["nrLines"].as_int_or_die();

            vector<Vector3D> points;
            vector<Face> faces;

            for (int n = 0; n < nrPoints; n++)
            {
                vector<double> point = conf[figName]["point" + to_string(n)].as_double_tuple_or_die();
                points.emplace_back(Vector3D::point(point[0], point[1], point[2]));
            }

            for (int n = 0; n < nrLines; n++)
            {
                faces.emplace_back(conf[figName]["line" + to_string(n)].as_int_tuple_or_die());
            }

            figure = Figure(points, faces, color);
        }

        else if (figType == "Cube")
        {
            figure = Figure::createCube(color);

        }

        else if (figType == "Tetrahedron")
        {
            figure = Figure::createTetrahedron(color);
        }

        else if (figType == "Octahedron")
        {
            figure = Figure::createOctahedron(color);
        }

        else if (figType == "Icosahedron")
        {
            figure = Figure::createIcosahedron(color);
        }

        else if (figType == "Dodecahedron")
        {
            figure = Figure::createDodecahedron(color);
        }

        // Finally, transform figure and add it to figures list
        figure.applyTransformation(transformMatrix);
        figures.emplace_back(figure);
    }

    // ============== Eye Point Transformation ============== //
    applyTransformation(figures, eyePointTrans(eyePoint));

    // ============== Eye Point Projection and Drawing Image ============== //
    return LSystem2D::Line2D::draw2DLines(doProjection(figures), size, backgroundColor);
}

// ========================================= Class Constructors ========================================= //
Figure3D::Face::Face(vector<int> aPointIndexes)
{
    pointIndexes = aPointIndexes;
}

Figure3D::Figure::Figure()
{
    points = {};
    faces = {};
    color = Color();
}

Figure3D::Figure::Figure(vector<Vector3D> &aPoints, vector<Face> &aFaces, const Color &aColor)
{
    points = aPoints;
    faces = aFaces;
    color = aColor;
}

// =========================================== Static Methods =========================================== //
Figure3D::Figure Figure3D::Figure::createCube(const Color &color)
{
    Vector3D p0 = Vector3D::point(1, 1, -1);
    Vector3D p1 = Vector3D::point(-1, 1, -1);
    Vector3D p2 = Vector3D::point(-1, -1, -1);
    Vector3D p3 = Vector3D::point(1, -1, -1);

    Vector3D p4 = Vector3D::point(1, 1, 1);
    Vector3D p5 = Vector3D::point(-1, 1, 1);
    Vector3D p6 = Vector3D::point(-1, -1, 1);
    Vector3D p7 = Vector3D::point(1, -1, 1);

    Face bottom = Face({0, 1, 2, 3});
    Face top = Face({4, 5, 6, 7});
    Face right = Face({0, 1, 5, 4});
    Face back = Face({1, 2, 6, 5});
    Face left = Face({2, 3, 7, 6});
    Face front = Face({3, 0, 4, 7});

    vector<Vector3D> points = {p0, p1, p2, p3, p4, p5, p6, p7};
    vector<Face> faces = {front, back, left, right, top, bottom};

    return {points, faces, color};
}

Figure3D::Figure Figure3D::Figure::createTetrahedron(const Color &color)
{
    Vector3D p0 = Vector3D::point(1, -1, -1);
    Vector3D p1 = Vector3D::point(-1, 1, -1);
    Vector3D p2 = Vector3D::point(1, 1, 1);
    Vector3D p3 = Vector3D::point(-1, -1, 1);

    Face f0 = Face({0, 1, 2});
    Face f1 = Face({1, 3, 2});
    Face f2 = Face({0, 3, 1});
    Face f3 = Face({0, 2, 3});

    vector<Vector3D> points = {p0, p1, p2, p3};
    vector<Face> faces = {f0, f1, f2, f3};

    return {points, faces, color};
}

Figure3D::Figure Figure3D::Figure::createOctahedron(const Color &color)
{
    Vector3D p0 = Vector3D::point(1, 0, 0);
    Vector3D p1 = Vector3D::point(0, 1, 0);
    Vector3D p2 = Vector3D::point(-1, 0, 0);
    Vector3D p3 = Vector3D::point(0, -1, 0);
    Vector3D p4 = Vector3D::point(0, 0, -1);
    Vector3D p5 = Vector3D::point(0, 0, 1);

    Face f0 = Face({0, 1, 5});
    Face f1 = Face({1, 2, 5});
    Face f2 = Face({2, 3, 5});
    Face f3 = Face({3, 0, 5});
    Face f4 = Face({1, 0, 4});
    Face f5 = Face({2, 1, 4});
    Face f6 = Face({3, 2, 4});
    Face f7 = Face({0, 3, 4});

    vector<Vector3D> points = {p0, p1, p2, p3, p4, p5};
    vector<Face> faces = {f0, f1, f2, f3, f4, f5, f6, f7};

    return {points, faces, color};
}

Figure3D::Figure Figure3D::Figure::createIcosahedron(const Color &color)
{
    vector<Vector3D> points = {Vector3D::point(0, 0, sqrt(5)/2)}; // Already add first point without logic

    for (int i = 2; i < 12; i++)
    {
        double x;
        double y;
        double z;

        if (i < 7)
        {
            x = cos((i-2) * 2 * M_PI / 5);
            y = sin((i-2) * 2 * M_PI / 5);
            z = 0.5;
        } else
        {
            x = cos(M_PI / 5 + (i-7) * 2 * M_PI / 5);
            y = sin(M_PI / 5 + (i-7) * 2 * M_PI / 5);
            z = -0.5;
        }

        points.emplace_back(Vector3D::point(x, y, z));
    }

    points.emplace_back(Vector3D::point(0, 0, -sqrt(5)/2)); // Add last point without logic

    Face f0 = Face({0, 1, 2});
    Face f1 = Face({0, 2, 3});
    Face f2 = Face({0, 3, 4});
    Face f3 = Face({0, 4, 5});
    Face f4 = Face({0, 5, 1});
    Face f5 = Face({1, 6, 2});
    Face f6 = Face({2, 6, 7});
    Face f7 = Face({2, 7, 3});
    Face f8 = Face({3, 7, 8});
    Face f9 = Face({3, 8, 4});
    Face f10 = Face({4, 8, 9});
    Face f11 = Face({4, 9, 5});
    Face f12 = Face({5, 9, 10});
    Face f13 = Face({5, 10, 1});
    Face f14 = Face({1, 10, 6});
    Face f15 = Face({11, 7, 6});
    Face f16 = Face({11, 8, 7});
    Face f17 = Face({11, 9, 8});
    Face f18 = Face({11, 10, 9});
    Face f19 = Face({11, 6, 10});

    vector<Face> faces = {f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14, f15, f16, f17, f18, f19};

    return {points, faces, color};
}

Figure3D::Figure Figure3D::Figure::createDodecahedron(const Color &color)
{
    // Create new icosahedron
    Figure fig = createIcosahedron(color);

    vector<Vector3D> points;

    // Use icosahedron to calculate point coordinates for dodecahedron
    for (int i = 0; i < 20; i++)
    {
        double x;
        double y;
        double z;

        vector<int> indexes = fig.faces[i].pointIndexes;

        x = ( fig.points[indexes[0]].x + fig.points[indexes[1]].x + fig.points[indexes[2]].x) / 3;
        y = ( fig.points[indexes[0]].y + fig.points[indexes[1]].y + fig.points[indexes[2]].y) / 3;
        z = ( fig.points[indexes[0]].z + fig.points[indexes[1]].z + fig.points[indexes[2]].z) / 3;

        points.emplace_back(Vector3D::point(x, y, z));
    }

    Face f0 = Face({0, 1, 2, 3, 4});
    Face f1 = Face({0, 5, 6, 7, 1});
    Face f2 = Face({1, 7, 8, 9, 2});
    Face f3 = Face({2, 9, 10, 11, 3});
    Face f4 = Face({3, 11, 12, 13, 4});
    Face f5 = Face({4, 13, 14, 5, 0});
    Face f6 = Face({19, 18, 17, 16, 15});
    Face f7 = Face({19, 14, 13, 12, 18});
    Face f8 = Face({18, 12, 11, 10, 17});
    Face f9 = Face({17, 10, 9, 8, 16});
    Face f10 = Face({16, 8, 7, 6, 15});
    Face f11 = Face({15, 6, 5, 14, 19});

    vector<Face> faces = {f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11};

    fig = Figure(points, faces, color);
    return fig;
}

Figure3D::Figure Figure3D::Figure::createSphere(const double r, const int n, const Color &color)
{

}

Figure3D::Figure Figure3D::Figure::createCone(const double h, const int n, const Color &color)
{

}

Figure3D::Figure Figure3D::Figure::createCilinder(const double h, const int n, const Color &color)
{

}

Figure3D::Figure Figure3D::Figure::createTorus(const double r, const double R, const int n, const int m, const Color &color)
{

}


// ========================================== Transformations ========================================== //
void Figure3D::toPolar(const Vector3D &point, double &r, double &theta, double &phi)
{
    r = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    theta = atan2(point.y, point.x);
    phi = acos(point.z / r);
}

Matrix Figure3D::scaleFigure(const double scale)
{
    Matrix matrix;
    matrix(1, 1) = scale;
    matrix(2, 2) = scale;
    matrix(3, 3) = scale;

    return matrix;
}

Matrix Figure3D::rotateX(const double angle)
{
    Matrix matrix;
    matrix(2, 2) = cos(angle);
    matrix(3, 3) = cos(angle);
    matrix(2, 3) = sin(angle);
    matrix(3, 2) = -sin(angle);

    return matrix;
}

Matrix Figure3D::rotateY(const double angle)
{
    Matrix matrix;
    matrix(1, 1) = cos(angle);
    matrix(3, 3) = cos(angle);
    matrix(3, 1) = sin(angle);
    matrix(1, 3) = -sin(angle);

    return matrix;
}

Matrix Figure3D::rotateZ(const double angle)
{
    Matrix matrix;
    matrix(1, 1) = cos(angle);
    matrix(2, 2) = cos(angle);
    matrix(1, 2) = sin(angle);
    matrix(2, 1) = -sin(angle);

    return matrix;
}

Matrix Figure3D::translate(const Vector3D &vector)
{
    Matrix matrix;
    matrix(4, 1) = vector.x;
    matrix(4, 2) = vector.y;
    matrix(4, 3) = vector.z;

    return matrix;
}

Matrix Figure3D::eyePointTrans(const Vector3D &eyepoint)
{
    double r;
    double theta;
    double phi;
    toPolar(eyepoint, r, theta, phi);

    Matrix matrix;
    matrix(1, 1) = -sin(theta);
    matrix(1, 2) = -cos(theta) * cos(phi);
    matrix(1, 3) = cos(theta) * sin(phi);
    matrix(2,1) = cos(theta);
    matrix(2, 2) = -sin(theta) * cos(phi);
    matrix(2, 3) = sin(theta) * sin(phi);
    matrix(3, 2) = sin(phi);
    matrix(3, 3) = cos(phi);
    matrix(4, 3) = -r;

    return matrix;
}

void Figure3D::Figure::applyTransformation(const Matrix &mat)
{
    for (auto &point : points)
    {
        point = point * mat;
    }
}

void Figure3D::applyTransformation(Figures3D &figs, const Matrix &mat)
{
    for (auto &fig : figs)
    {
        fig.applyTransformation(mat);
    }
}

Point2D Figure3D::doProjection(const Vector3D &eyeTransformedPoint, const double d)
{
    return {-d * eyeTransformedPoint.x / eyeTransformedPoint.z, -d * eyeTransformedPoint.y / eyeTransformedPoint.z};
}

Lines2D Figure3D::doProjection(const Figures3D &eyeTransformedFigures)
{
    Lines2D lines;

    for (const auto &fig : eyeTransformedFigures)
    {
        vector<Point2D> points2DVector;

        // Get vector of Point2D objects instead of Vector3D objects
        for (const auto &point: fig.points)
        {
            points2DVector.emplace_back(Figure3D::doProjection(point, 1));
        }

        // Fill list of lines with indexes from the faces
        // Indexes are used to select points from points2DVector to create lines with
        for (const auto &face : fig.faces)
        {
            // First create the line between the first and last points
            int numPoints = face.pointIndexes.size();

            Point2D startPoint = points2DVector[face.pointIndexes[numPoints - 1]];
            Point2D endPoint = points2DVector[face.pointIndexes[0]];

            lines.emplace_back(startPoint, endPoint, fig.color);

            // Create all other lines
            if (numPoints > 2)
            {
                for (int i = 1; i < numPoints; i++)
                {
                    startPoint = endPoint;
                    endPoint = points2DVector[face.pointIndexes[i]];
                    lines.emplace_back(startPoint, endPoint, fig.color);
                }
            }
        }
    }

    return lines;
}

