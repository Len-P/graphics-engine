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

void Figure3D::Figure::splitLine3(Vector3D &A, Vector3D &B)
{
    Vector3D AB = B - A;

    // Points inbetween each line that split the line in 3
    Vector3D pointAB1 = A + AB/3;
    Vector3D pointAB2 = A + 2*AB/3;

    points.emplace_back(pointAB1);
    points.emplace_back(pointAB2);
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

Figure3D::Figure Figure3D::Figure::createBuckyBall(const Color &color)
{
    Figure icosa = createIcosahedron(color);

    int oldPointSize = icosa.points.size();

    vector<Face> newFaces;

    // Contains the 12 original icosa points which get deleted after the for loop
    vector<Vector3D> deletedPointsVec = icosa.points;

    for (const auto &face : icosa.faces)
    {
        int pointsSize = icosa.points.size() - oldPointSize;

        vector<int> pointIndexes = face.pointIndexes;
        Vector3D A = icosa.points[pointIndexes[0]];
        Vector3D B = icosa.points[pointIndexes[1]];
        Vector3D C = icosa.points[pointIndexes[2]];

        icosa.splitLine3(A, B);
        icosa.splitLine3(B, C);
        icosa.splitLine3(C, A);

        // Create hexagon in center
        newFaces.emplace_back(Face({pointsSize, pointsSize + 1, pointsSize + 2, pointsSize + 3, pointsSize + 4, pointsSize + 5}));
    }

    icosa.points.erase(icosa.points.begin(), icosa.points.begin() + oldPointSize);

    //! /$$$$$$$                                        /$$             /$$                                   /$$               /$$$           /$$          /$$$
    //!| $$__  $$                                      | $$            | $$                                  | $$              /$$_/          | $$         |_  $$
    //!| $$  \ $$  /$$$$$$        /$$$$$$$   /$$$$$$  /$$$$$$         /$$$$$$    /$$$$$$  /$$   /$$  /$$$$$$$| $$$$$$$        /$$/    /$$$$$$ | $$  /$$$$$$$ \  $$
    //!| $$  | $$ /$$__  $$      | $$__  $$ /$$__  $$|_  $$_/        |_  $$_/   /$$__  $$| $$  | $$ /$$_____/| $$__  $$      | $$    /$$__  $$| $$ /$$_____/  | $$
    //!| $$  | $$| $$  \ $$      | $$  \ $$| $$  \ $$  | $$            | $$    | $$  \ $$| $$  | $$| $$      | $$  \ $$      | $$   | $$  \ $$| $$|  $$$$$$   | $$
    //!| $$  | $$| $$  | $$      | $$  | $$| $$  | $$  | $$ /$$        | $$ /$$| $$  | $$| $$  | $$| $$      | $$  | $$      |  $$  | $$  | $$| $$ \____  $$  /$$/
    //!| $$$$$$$/|  $$$$$$/      | $$  | $$|  $$$$$$/  |  $$$$/        |  $$$$/|  $$$$$$/|  $$$$$$/|  $$$$$$$| $$  | $$       \  $$$| $$$$$$$/| $$ /$$$$$$$//$$$/
    //!|_______/  \______/       |__/  |__/ \______/    \___/           \___/   \______/  \______/  \_______/|__/  |__/        \___/| $$____/ |__/|_______/|___/

    // Pool contains all unique hexagon points with index
    map<Vector3D, int> pool;
    for (int i = 0; i < icosa.points.size(); i++)
    {
        pool[icosa.points[i]] = i;
    }

    // For each deleted point, create a pentagon
    for (const auto &point : deletedPointsVec)
    {
        // Sort tempvec by distance to deletedPoint, then take first 5 points: these are the pentagon points in random order
        vector<pair<Vector3D, int>> tempVec (pool.begin(), pool.end());
        sort(tempVec.begin(), tempVec.end(), distanceToPointComparator(point));
        vector<pair<Vector3D, int>> fivePoints(tempVec.begin(), tempVec.begin() + 5);

        // Delete these points from the pool for more efficient computation later
        for (const auto &pointToDelete : fivePoints)
        {
            pool.erase(pointToDelete.first);
        }

        // Pick random point as first point for the face, in this case the first point in the vector
        // Then erase it from the fivePoints list and sort by distance to this random point: this results in the 2 nearest points being first in the vector
        pair<Vector3D, int> firstPoint = fivePoints[0];
        fivePoints.erase(fivePoints.begin());
        sort(fivePoints.begin(), fivePoints.end(), distanceToPointComparator(firstPoint.first));

        // Calculate normal vector on plane created by deletedPoint, firstPoint and potential second point
        Vector3D vec1 = firstPoint.first - point;
        Vector3D vec2 = fivePoints[0].first - point;
        Vector3D cross = Vector3D::cross(vec1, vec2);
        double dot = Vector3D::dot(cross, point); // Dot between normal vector and vector of deletedPoint (vector from origin to deletedPoint)

        pair<Vector3D, int> secondPoint;

        // If dot product is positive, this is the correct next point (counter-clockwise), otherwise take the other nearest point
        if (dot > 0)
        {
            secondPoint = fivePoints[0];
            fivePoints.erase(fivePoints.begin());
        }
        else
        {
            secondPoint = fivePoints[1];
            fivePoints.erase(fivePoints.begin() + 1);
        }

        vector<int> pointIndexes = {firstPoint.second, secondPoint.second};

        // Keep taking the next nearest point to order the face indexes counter-clockwise
        while (!fivePoints.empty())
        {
            // secondPoint used as point iterator
            sort(fivePoints.begin(), fivePoints.end(), distanceToPointComparator(secondPoint.first));
            secondPoint = fivePoints[0];
            pointIndexes.emplace_back(secondPoint.second);
            fivePoints.erase(fivePoints.begin());
        }

        newFaces.emplace_back(pointIndexes);
    }

    icosa.faces = newFaces;
    return icosa;
}

void Figure3D::Figure::createMengerSponge(Figure3D::Figure &fig, Figure3D::Figures3D &fractal, const int nrIter, const int iterCounter)
{
    if (nrIter == 0)
    {
        fractal.emplace_back(fig);
    }
    else
    {
        for (int i = 0; i < fig.points.size(); i++)
        {
            Vector3D &point = fig.points[i];
            Figure3D::Figure figCopy = fig;
            double scale = 1.0 / 3.0;
            double transScale = pow(scale, iterCounter);

            figCopy.applyTransformation(Transformations::scaleFigure(scale));
            figCopy.applyTransformation(Transformations::translate(point - figCopy.points[i]));

            Figure3D::Figure figCopyOffset0;
            Figure3D::Figure figCopyOffset1;
            Figure3D::Figure figCopyOffset2;
            if (i == 0)
            {
                figCopyOffset0 = figCopy;
                figCopyOffset0.applyTransformation(Transformations::translate(Vector3D::vector(0, 0, 2 * transScale)));

                figCopyOffset1 = figCopy;
                figCopyOffset1.applyTransformation(Transformations::translate(Vector3D::vector(0, -2 * transScale, 0)));

                figCopyOffset2 = figCopy;
                figCopyOffset2.applyTransformation(Transformations::translate(Vector3D::vector(-2 * transScale, 0, 0)));
            }

            Figure3D::Figure figCopyOffset3;
            Figure3D::Figure figCopyOffset4;
            Figure3D::Figure figCopyOffset5;
            if (i == 2)
            {
                figCopyOffset3 = figCopy;
                figCopyOffset3.applyTransformation(Transformations::translate(Vector3D::vector(0, 0, 2 * transScale)));

                figCopyOffset4 = figCopy;
                figCopyOffset4.applyTransformation(Transformations::translate(Vector3D::vector(0, 2 * transScale, 0)));

                figCopyOffset5 = figCopy;
                figCopyOffset5.applyTransformation(Transformations::translate(Vector3D::vector(2 * transScale, 0, 0)));
            }

            Figure3D::Figure figCopyOffset6;
            Figure3D::Figure figCopyOffset7;
            Figure3D::Figure figCopyOffset8;
            if (i == 5)
            {
                figCopyOffset6 = figCopy;
                figCopyOffset6.applyTransformation(Transformations::translate(Vector3D::vector(0, 0, -2 * transScale)));

                figCopyOffset7 = figCopy;
                figCopyOffset7.applyTransformation(Transformations::translate(Vector3D::vector(0, -2 * transScale, 0)));

                figCopyOffset8 = figCopy;
                figCopyOffset8.applyTransformation(Transformations::translate(Vector3D::vector(2 * transScale, 0, 0)));
            }

            Figure3D::Figure figCopyOffset9;
            Figure3D::Figure figCopyOffset10;
            Figure3D::Figure figCopyOffset11;
            if (i == 7)
            {
                figCopyOffset9 = figCopy;
                figCopyOffset9.applyTransformation(Transformations::translate(Vector3D::vector(0, 0, -2 * transScale)));

                figCopyOffset10 = figCopy;
                figCopyOffset10.applyTransformation(Transformations::translate(Vector3D::vector(0, 2 * transScale, 0)));

                figCopyOffset11 = figCopy;
                figCopyOffset11.applyTransformation(
                        Transformations::translate(Vector3D::vector(-2 * transScale, 0, 0)));
            }

            if (nrIter == 1)
            {
                fractal.emplace_back(figCopy);

                if (i == 0)
                {
                    fractal.emplace_back(figCopyOffset0);
                    fractal.emplace_back(figCopyOffset1);
                    fractal.emplace_back(figCopyOffset2);
                }
                else if (i == 2)
                {
                    fractal.emplace_back(figCopyOffset3);
                    fractal.emplace_back(figCopyOffset4);
                    fractal.emplace_back(figCopyOffset5);
                }
                else if (i == 5)
                {
                    fractal.emplace_back(figCopyOffset6);
                    fractal.emplace_back(figCopyOffset7);
                    fractal.emplace_back(figCopyOffset8);
                }
                else if (i == 7)
                {
                    fractal.emplace_back(figCopyOffset9);
                    fractal.emplace_back(figCopyOffset10);
                    fractal.emplace_back(figCopyOffset11);
                }
            }
            else
            {
                createMengerSponge(figCopy, fractal, nrIter - 1, iterCounter + 1);

                if (i == 0)
                {
                    createMengerSponge(figCopyOffset0, fractal, nrIter - 1, iterCounter + 1);
                    createMengerSponge(figCopyOffset1, fractal, nrIter - 1, iterCounter + 1);
                    createMengerSponge(figCopyOffset2, fractal, nrIter - 1, iterCounter + 1);
                }
                else if (i == 2)
                {
                    createMengerSponge(figCopyOffset3, fractal, nrIter - 1, iterCounter + 1);
                    createMengerSponge(figCopyOffset4, fractal, nrIter - 1, iterCounter + 1);
                    createMengerSponge(figCopyOffset5, fractal, nrIter - 1, iterCounter + 1);
                }
                else if (i == 5)
                {
                    createMengerSponge(figCopyOffset6, fractal, nrIter - 1, iterCounter + 1);
                    createMengerSponge(figCopyOffset7, fractal, nrIter - 1, iterCounter + 1);
                    createMengerSponge(figCopyOffset8, fractal, nrIter - 1, iterCounter + 1);
                }
                else if (i == 7)
                {
                    createMengerSponge(figCopyOffset9, fractal, nrIter - 1, iterCounter + 1);
                    createMengerSponge(figCopyOffset10, fractal, nrIter - 1, iterCounter + 1);
                    createMengerSponge(figCopyOffset11, fractal, nrIter - 1, iterCounter + 1);
                }
            }
        }
    }
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
    Matrix transformMatrix =  Transformations::scaleFigure(scale) * Transformations::rotateX(rotXAngle) * Transformations::rotateY(rotYAngle) * Transformations::rotateZ(rotZAngle) * Transformations::translate(center);

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

    else if (figType == "Cube" || figType == "FractalCube" || figType == "MengerSponge")
    {
        figure = Figure::createCube(color);
    }

    else if (figType == "Tetrahedron" || figType == "FractalTetrahedron")
    {
        figure = Figure::createTetrahedron(color);
    }

    else if (figType == "Octahedron" || figType == "FractalOctahedron")
    {
        figure = Figure::createOctahedron(color);
    }

    else if (figType == "Icosahedron" || figType == "FractalIcosahedron")
    {
        figure = Figure::createIcosahedron(color);
    }

    else if (figType == "Dodecahedron" || figType == "FractalDodecahedron")
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

    else if (figType == "BuckyBall" || figType == "FractalBuckyBall")
    {
        figure = Figure::createBuckyBall(color);
    }

    figure.applyTransformation(transformMatrix);

    if (triangulate)
    {
        figure.triangulate();
    }

    return figure;
}


