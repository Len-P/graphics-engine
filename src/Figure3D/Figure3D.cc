#include "Figure3D.h"



// ?========================================== Parse Ini ==========================================? //
EasyImage Figure3D::parseIni(const Configuration &conf, const bool ZBuffering)
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

        Figure figure = Figure::generateFigure(conf, figName, false);

        figures.emplace_back(figure);
    }

    // ?=========== Eye Point Transformation ===========? //
    applyTransformation(figures, eyePointTrans(eyePoint));

    // ?==== Eye Point Projection and Drawing Image ====? //
    Lines2D lines = doProjection(figures);
    return LSystem2D::Line2D::draw2DLines(lines, size, backgroundColor, ZBuffering);
}

// ?========================================= Class Constructors =========================================? //
Figure3D::Face::Face(vector<int> aPointIndexes)
{
    pointIndexes = std::move(aPointIndexes);
}

vector<Figure3D::Face> Figure3D::Face::triangulate(const Figure3D::Face &face)
{
    vector<Figure3D::Face> faces;

    for (int i = 1; i < face.pointIndexes.size() - 1; i++)
    {
        Figure3D::Face triangle = Figure3D::Face({face.pointIndexes[0], face.pointIndexes[i], face.pointIndexes[i+1]});
        faces.emplace_back(triangle);
    }

    return faces;
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

// ?=========================================== Class Methods ===========================================? //
void Figure3D::Figure::applyTransformation(const Matrix &mat)
{
    for (auto &point : points)
    {
        point = point * mat;
    }
}

void Figure3D::Figure::triangulateTriangles(const int n)
{
    for (int k = 0; k < n; k++)
    {
        int nrFaces = faces.size();

        // Old face needs to get replaces by 4 faces that have the correct point indexes
        // Initial 3 points stay, 3 points get added
        for (int i = 0; i < nrFaces; i++)
        {
            int nrPoints = points.size();
            vector<int> &pointIndexes = faces[i].pointIndexes;

            Vector3D &A = points[pointIndexes[0]];
            Vector3D &B = points[pointIndexes[1]];
            Vector3D &C = points[pointIndexes[2]];
            Vector3D D = (A + B) / 2; // nrPoints
            Vector3D E = (A + C) / 2; // nrPoints + 1
            Vector3D F = (B + C) / 2; // nrPoints + 2

            points.emplace_back(D);
            points.emplace_back(E);
            points.emplace_back(F);

            Face ADE = Face({pointIndexes[0], nrPoints, nrPoints + 1});
            Face DBF = Face({nrPoints, pointIndexes[1], nrPoints + 2});
            Face EFC = Face({nrPoints + 1, nrPoints + 2, pointIndexes[2]});
            Face DFE = Face({nrPoints, nrPoints + 2, nrPoints + 1});

            // Set old face equal to one of the faces (to preserve face indexes), then add the rest of the faces at the end
            faces.at(i) = ADE;
            faces.emplace_back(DBF);
            faces.emplace_back(EFC);
            faces.emplace_back(DFE);
        }
    }
}

void Figure3D::Figure::triangulate()
{
    vector<Face> triangles;

    for (const auto &face : faces)
    {
        vector<Face> faceTriangles = Face::triangulate(face);

        for (const auto &triangle : faceTriangles)
        {
            triangles.emplace_back(triangle);
        }
    }

    faces = triangles;
}

// ?=========================================== Static Methods ===========================================? //
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
    Figure icosa = createIcosahedron(color);
    icosa.triangulateTriangles(n);

    for (auto &point : icosa.points)
    {
        point.normalise();
        point *= r;
    }

    return icosa;
}

Figure3D::Figure Figure3D::Figure::createCone(const double h, const int n, const Color &color)
{
    vector<Vector3D> points;
    vector<Face> faces;

    // Bottom points
    for (int i = 0; i < n; i++)
    {
        points.emplace_back(Vector3D::point(cos(2 * i * M_PI / n), sin(2 * i * M_PI / n), 0));
    }
    // Top point
    points.emplace_back(Vector3D::point(0, 0, h));

    // Side faces
    for (int i = 0; i < n; i++)
    {
        vector<int> indexes = {i, (i+1)%n, n};
        faces.emplace_back(indexes);
    }

    // Bottom face
    vector<int> bottomFaceIndexes;
    for(int i = n-1; i >= 0; i--)
    {
        bottomFaceIndexes.emplace_back(i);
    }
    faces.emplace_back(bottomFaceIndexes);

    return {points, faces, color};
}

Figure3D::Figure Figure3D::Figure::createCylinder(const double h, const int n, const Color &color)
{
    vector<Vector3D> points;
    vector<Face> faces;

    // Separate for loops to preserve indexes
    // Bottom points
    for (int i = 0; i < n; i++)
    {
        points.emplace_back(Vector3D::point(cos(2 * i * M_PI / n), sin(2 * i * M_PI / n), 0));
    }

    // Top points
    for (int i = 0; i < n; i++)
    {
        points.emplace_back(Vector3D::point(cos(2 * i * M_PI / n), sin(2 * i * M_PI / n), h));
    }

    // Side faces
    for (int i = 0; i < n; i++)
    {
        vector<int> indexes = {i, (i+1)%n, (i+1)%n + n, i+n};
        faces.emplace_back(indexes);
    }

    // Bottom and top faces
    vector<int> bottomFaceIndexes;
    vector<int> topFaceIndexes;
    for(int i = n-1; i >= 0; i--)
    {
        bottomFaceIndexes.emplace_back(i);
        topFaceIndexes.emplace_back(i + n);
    }
    faces.emplace_back(bottomFaceIndexes);
    faces.emplace_back(topFaceIndexes);

    return {points, faces, color};
}

Figure3D::Figure Figure3D::Figure::createTorus(const double r, const double R, const int n, const int m, const Color &color)
{
    vector<Vector3D> points;
    vector<Face> faces;

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
        {
            double u = 2 * i * M_PI / n;
            double v = 2 * j * M_PI / m;

            double x = (R + r * cos(v)) * cos (u);
            double y = (R + r * cos(v)) * sin (u);
            double z = r * sin(v);

            points.emplace_back(Vector3D::point(x, y, z));
        }
    }

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
        {
            // To get element i,j from the flattened points vector, use: index = i * m + j
            vector<int> indexes = {i * m + j, (i+1)%n * m + j, (i+1)%n * m + (j+1)%m, i * m + (j+1)%m};
            faces.emplace_back(indexes);
        }
    }

    return {points, faces, color};
}

void Figure3D::Figure::recursiveLSystem3D(const string &str, unsigned int iter, unsigned int maxIter, Vector3D &H, Vector3D &L, Vector3D &U, const LParser::LSystem3D &l_system, vector<Vector3D> &points, vector<Figure3D::Face> &faces, Vector3D &startPoint, Vector3D &endPoint, stack<tuple<Vector3D, Vector3D, Vector3D, Vector3D>> &stack, const Color &color)
{
    const double angle = l_system.get_angle() * M_PI/180;

    for (char c : str)
    {
        // If character is not in alphabet (stop condition 1)
        if (l_system.get_alphabet().find(c) == l_system.get_alphabet().end())
        {
            Vector3D tempH = H;
            Vector3D tempL = L;
            Vector3D tempU = U;

            switch(c) {
                case '+':
                    H = tempH * cos(angle) + tempL * sin(angle);
                    L = -tempH * sin(angle) + tempL * cos(angle);
                    break;
                case '-':
                    H = tempH * cos(-angle) + tempL * sin(-angle);
                    L = -tempH * sin(-angle) + tempL * cos(-angle);
                    break;
                case '&':
                    H = tempH * cos(angle) - tempU * sin(angle);
                    U = tempU * cos(angle) + tempH * sin(angle);
                    break;
                case '^':
                    H = tempH * cos(-angle) - tempU * sin(-angle);
                    U = tempU * cos(-angle) + tempH * sin(-angle);
                    break;
                case '/':
                    L = tempL * cos(angle) + tempU * sin(angle);
                    U = -tempL * sin(angle) + tempU * cos(angle);
                    break;
                case '\\':
                    L = tempL * cos(-angle) + tempU * sin(-angle);
                    U = -tempL * sin(-angle) + tempU * cos(-angle);
                    break;
                case '(':  // Save current point and HLU in stack
                    stack.emplace(endPoint, H, L, U);
                    break;
                case ')':  // Teleport back to last point with last HLU
                    tuple<Vector3D, Vector3D, Vector3D, Vector3D> tuple = stack.top();
                    endPoint = get<0>(tuple);
                    H = get<1>(tuple);
                    L = get<2>(tuple);
                    U = get<3>(tuple);
                    stack.pop();
                    break;
            }

        }

        // If max depth has been reached (stop condition 2)
        else if (iter == maxIter)
        {
            startPoint = endPoint;

            endPoint = startPoint + H;

            points.emplace_back(startPoint);
            points.emplace_back(endPoint);

            if (l_system.draw(c))
            {
                int size = points.size();
                faces.emplace_back(Face({size - 2, size - 1}));
            }

        }

        // Keep going deeper with replacement rules
        else
        {
            const string &replacement = l_system.get_replacement(c);
            recursiveLSystem3D(replacement, iter + 1, maxIter, H, L, U, l_system, points, faces, startPoint, endPoint, stack, color);
        }

    }

}

Figure3D::Figure Figure3D::Figure::LSystem3DToFigure(const LParser::LSystem3D &l_system, const Color &color)
{
    // Parse .L3D file
    const string &initiator = l_system.get_initiator();
    const unsigned int iterations = l_system.get_nr_iterations();

    // Initialize necessary objects
    Vector3D H = Vector3D::vector(1, 0, 0);
    Vector3D L = Vector3D::vector(0, 1, 0);
    Vector3D U = Vector3D::vector(0, 0, 1);

    Vector3D startPoint = Vector3D::point(0, 0, 0);
    Vector3D endPoint = Vector3D::point(0, 0, 0);

    stack<tuple<Vector3D, Vector3D, Vector3D, Vector3D>> stack;

    vector<Vector3D> points;
    vector<Face> faces;

    recursiveLSystem3D(initiator, 0, iterations, H, L, U, l_system, points, faces, startPoint, endPoint, stack, color);

    return {points, faces, color};
}

Figure3D::Figure Figure3D::Figure::generateFigure(const Configuration &conf, const string &figName, const bool &triangulate)
{
    Figure figure;
    string figType = conf[figName]["type"].as_string_or_die();

    // Transformations
    double rotXAngle = conf[figName]["rotateX"].as_double_or_default(0) * M_PI/180;
    double rotYAngle = conf[figName]["rotateY"].as_double_or_default(0) * M_PI/180;
    double rotZAngle = conf[figName]["rotateZ"].as_double_or_default(0) * M_PI/180;
    double scale = conf[figName]["scale"].as_double_or_default(1);

    vector<double> centerTuple = conf[figName]["center"].as_double_tuple_or_die();
    Vector3D center = Vector3D::point(centerTuple[0], centerTuple[1], centerTuple[2]);
    Matrix transformMatrix =  scaleFigure(scale) * rotateX(rotXAngle) * rotateY(rotYAngle) * rotateZ(rotZAngle) * translate(center);

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

    else if (figType == "Sphere")
    {
        double r = conf[figName]["r"].as_double_or_default(1);
        int n = conf[figName]["n"].as_int_or_die();
        figure = Figure::createSphere(r, n, color);
    }

    else if (figType == "Cone")
    {
        double h = conf[figName]["height"].as_double_or_die();
        int n = conf[figName]["n"].as_int_or_die();
        figure = Figure::createCone(h, n, color);
    }

    else if (figType == "Cylinder")
    {
        double h = conf[figName]["height"].as_double_or_die();
        int n = conf[figName]["n"].as_int_or_die();
        figure = Figure::createCylinder(h, n, color);
    }

    else if (figType == "Torus")
    {
        double r = conf[figName]["r"].as_double_or_die();
        double R = conf[figName]["R"].as_double_or_die();
        int n = conf[figName]["n"].as_int_or_die();
        int m = conf[figName]["m"].as_int_or_die();
        figure = Figure::createTorus(r, R, n, m, color);
    }

    else if (figType == "3DLSystem")
    {
        string inputFile = conf[figName]["inputfile"].as_string_or_die();

        // Create 3D LSystem from input file
        LParser::LSystem3D l_system;
        std::ifstream input_stream(inputFile);
        input_stream >> l_system;
        input_stream.close();

        figure = Figure::LSystem3DToFigure(l_system, color);
    }

    figure.applyTransformation(transformMatrix);

    if (triangulate)
    {
        figure.triangulate();
    }

    return figure;
}

// ?========================================== Transformations ==========================================? //
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
    matrix(2, 1) = cos(theta);
    matrix(2, 2) = -sin(theta) * cos(phi);
    matrix(2, 3) = sin(theta) * sin(phi);
    matrix(3, 2) = sin(phi);
    matrix(3, 3) = cos(phi);
    matrix(4, 3) = -r;

    return matrix;
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
    return {-d * eyeTransformedPoint.x / eyeTransformedPoint.z, -d * eyeTransformedPoint.y / eyeTransformedPoint.z, eyeTransformedPoint.z};
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

            lines.emplace_back(startPoint, endPoint, fig.color, startPoint.z, endPoint.z);

            // Create all other lines
            if (numPoints > 2)
            {
                for (int i = 1; i < numPoints; i++)
                {
                    startPoint = endPoint;
                    endPoint = points2DVector[face.pointIndexes[i]];
                    lines.emplace_back(startPoint, endPoint, fig.color, startPoint.z, endPoint.z);
                }
            }
        }
    }

    return lines;
}

