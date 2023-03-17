#include "Figure3D.h"



// ========================================== Classes ========================================== //
Figure3D::Face::Face(vector<int> aPointIndexes)
{
    pointIndexes = aPointIndexes;
}

Figure3D::Figure::Figure(vector<Vector3D> &aPoints, vector<Face> &aFaces, const Color &aColor)
{
    points = aPoints;
    faces = aFaces;
    color = aColor;
}

// ========================================== Parse Ini ========================================== //
EasyImage Figure3D::parseIniFigure3D(const Configuration &conf)
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
        string figName = "Figure" + to_string(i);

        // LineDrawing
        if (conf[figName]["type"].as_string_or_die() == "LineDrawing")
        {
            // Transformations
            double rotXAngle = conf[figName]["rotateX"].as_double_or_default(0) * M_PI/180;
            double rotYAngle = conf[figName]["rotateY"].as_double_or_default(0) * M_PI/180;
            double rotZAngle = conf[figName]["rotateZ"].as_double_or_default(0) * M_PI/180;
            double scale = conf[figName]["scale"].as_double_or_default(1);

            vector<double> centerTuple = conf[figName]["center"].as_double_tuple_or_die();
            Vector3D center = Vector3D::point(-centerTuple[0], -centerTuple[1], -centerTuple[2]); // Negative elements to center at (0, 0, 0)

            // Figure color
            vector<double> colorTuple = conf[figName]["color"].as_double_tuple_or_die();
            Color color = Color(lround(colorTuple[0] * 255), lround(colorTuple[1] * 255), lround(colorTuple[2] * 255));

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

            // Create and add transformed figure to figures list
            Figure figure = Figure(points, faces, color);

            Matrix transformMatrix = translate(center) * scaleFigure(scale) * rotateX(rotXAngle) * rotateY(rotYAngle) * rotateZ(rotZAngle);
            figure.applyTransformation(transformMatrix);

            figures.emplace_back(figure);
        }
    }

    // ============== Eye Point Transformation ============== //
    applyTransformation(figures, eyePointTrans(eyePoint));

    // ============== Eye Point Projection and Drawing Image ============== //
    return LSystem2D::draw2DLines(doProjection(figures), size, backgroundColor);
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
                for (int i = 1; i < numPoints - 1; i++)
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

