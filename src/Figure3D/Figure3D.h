#ifndef ENGINE_FIGURE3D_H
#define ENGINE_FIGURE3D_H
#define _USE_MATH_DEFINES

#include "../utils/ini_configuration.h"
#include "../utils/easy_image.h"
#include "../utils/vector3d.h"
#include "../LSystem2D/LSystem2D.h"
#include "../ZBuffering/ZBuffer.h"
#include "../Fractal3D/Fractal3D.h"
#include <list>
#include <cmath>
#include <set>
#include <map>
#include <algorithm>



using namespace ini;
using namespace img;
using namespace LSystem2D;
using std::list;
using std::set;
using std::map;
using std::pair;
using std::sort;
using std::to_string;
using std::make_tuple;

namespace Figure3D
{

    EasyImage parseIni(const Configuration &conf, const bool ZBuffering = false);

    class Face
    {
        public:
            vector<int> pointIndexes; // Indexes refer to points in the points vector in the Figure class

            // Face constructor
            explicit Face(vector<int> aPointIndexes);

            // Triangulates a face of any amount of points
            static vector<Face> triangulate(const Face &face);
    };

    class Figure;
    typedef list<Figure> Figures3D;
    typedef tuple<Color, Color, Color, double> reflectionCoeffs;

    class Figure
    {
        public:
            vector<Vector3D> points;
            vector<Face> faces;
            Color ambientReflection;
            Color diffuseReflection;
            Color specularReflection;
            double reflectionCoefficient;

            // Figure constructors
            Figure();
            Figure(vector<Vector3D> &aPoints, vector<Face> &aFaces, Color &aAmbientReflection); // Figure with 1 color
            Figure(vector<Vector3D> &aPoints, vector<Face> &aFaces, Color &aAmbientReflection,Color &aDiffuseReflection, Color &aSpecularReflection, double aReflectionCoefficient);
            Figure(vector<Vector3D> &aPoints, vector<Face> &aFaces, reflectionCoeffs &tuple);

            // Apply transformation matrix to figure
            void applyTransformation(const Matrix &mat);

            // For figures made out of triangles, divide every triangle into 4 triangles n times
            void triangulateTriangles(const int n);

            // Turn all faces into triangles
            void triangulate();

            // Split a line in 3 using 2 points, and add those 2 points to the points vector of the figure
            void splitLine3(Vector3D &A, Vector3D &B);

            // ?============== Static Methods ==============? //
            static Figures3D parseFigures(const Configuration &conf, const bool &triangulate, const bool &lighted);

            static Figure createCube(reflectionCoeffs &colorCoeffs);
            static Figure createTetrahedron(reflectionCoeffs &colorCoeffs);
            static Figure createOctahedron(reflectionCoeffs &colorCoeffs);
            static Figure createIcosahedron(reflectionCoeffs &colorCoeffs);
            static Figure createDodecahedron(reflectionCoeffs &colorCoeffs);
            static Figure createSphere(const double r, const int n, reflectionCoeffs &colorCoeffs);
            static Figure createCone(const double h, const int n, reflectionCoeffs &colorCoeffs);
            static Figure createCylinder(const double h, const int n, reflectionCoeffs &colorCoeffs);
            static Figure createTorus(const double r, const double R, const int n, const int m, reflectionCoeffs &colorCoeffs);
            static Figure createBuckyBall(reflectionCoeffs &colorCoeffs);
            static void createMengerSponge(Figure &fig, Figures3D &fractal, const int nrIter, const int iterCounter);

            static void recursiveLSystem3D(const string &str, unsigned int iter, unsigned int maxIter, Vector3D &H, Vector3D &L, Vector3D &U, const LParser::LSystem3D &l_system, vector<Vector3D> &points, vector<Figure3D::Face> &faces, Vector3D &startPoint, Vector3D &endPoint, stack<tuple<Vector3D, Vector3D, Vector3D, Vector3D>> &stack);
            static Figure LSystem3DToFigure(const LParser::LSystem3D &l_system, reflectionCoeffs &colorCoeffs);

            // Generate a figure based on name. Triangulate all faces if desired.
            static Figure generateFigure(const Configuration &conf, const string &figName, const bool &triangulate, const bool &lighted = false);
            // ?============================================? //
    };

    class distanceToPointComparator
    {
        public:
            Vector3D point;

            explicit distanceToPointComparator(const Vector3D &p)
            {
                this->point = p;
            }

            inline bool operator() (const pair<Vector3D, int> &vec1, const pair<Vector3D, int> &vec2) const
            {
                double dx = vec1.first.x - point.x;
                double dy = vec1.first.y - point.y;
                double dz = vec1.first.z - point.z;
                double d1 = sqrt(dx*dx + dy*dy + dz*dz);

                dx = vec2.first.x - point.x;
                dy = vec2.first.y - point.y;
                dz = vec2.first.z - point.z;
                double d2 = sqrt(dx*dx + dy*dy + dz*dz);

                return (d1 < d2);
            }
    };

}

#endif //ENGINE_FIGURE3D_H
