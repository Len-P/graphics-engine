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



using namespace ini;
using namespace img;
using namespace LSystem2D;
using std::list;
using std::to_string;

namespace Figure3D {

    EasyImage parseIni(const Configuration &conf, const bool ZBuffering = false);

    class Face {
    public:
        vector<int> pointIndexes; // Indexes refer to points in the points vector in the Figure class

        // Face constructor
        explicit Face(vector<int> aPointIndexes);

        // Triangulates a face of any amount of points
        static vector<Face> triangulate(const Face &face);
    };

    class Figure;
    typedef list<Figure> Figures3D;

    class Figure {
    public:
        vector<Vector3D> points;
        vector<Face> faces;
        Color color;

        // Figure constructors
        Figure();
        Figure(vector<Vector3D> &aPoints, vector<Face> &aFaces, const Color &aColor);

        // Apply transformation matrix to figure
        void applyTransformation(const Matrix &mat);

        // For figures made out of triangles, divide every triangle into 4 triangles n times
        void triangulateTriangles(const int n);

        // Turn all faces into triangles
        void triangulate();

        // Split a line in 3 using 2 points, and add those 2 points to the points vector of the figure
        void splitLine3(Vector3D &A, Vector3D &B);

        // ?============== Static Methods ==============? //
        static Figure createCube(const Color &color);
        static Figure createTetrahedron(const Color &color);
        static Figure createOctahedron(const Color &color);
        static Figure createIcosahedron(const Color &color);
        static Figure createDodecahedron(const Color &color);
        static Figure createSphere(const double r, const int n, const Color &color);
        static Figure createCone(const double h, const int n, const Color &color);
        static Figure createCylinder(const double h, const int n, const Color &color);
        static Figure createTorus(const double r, const double R, const int n, const int m, const Color &color);
        static Figure createBuckyBall(const Color &color);
        static void createMengerSponge(Figure &fig, Figures3D &fractal, const int nrIter, const int iterCounter);

        static void recursiveLSystem3D(const string &str, unsigned int iter, unsigned int maxIter, Vector3D &H, Vector3D &L, Vector3D &U, const LParser::LSystem3D &l_system, vector<Vector3D> &points, vector<Figure3D::Face> &faces, Vector3D &startPoint, Vector3D &endPoint, stack<tuple<Vector3D, Vector3D, Vector3D, Vector3D>> &stack, const Color &color);
        static Figure LSystem3DToFigure(const LParser::LSystem3D &l_system, const Color &color);

        // Generate a figure based on name. Triangulate all faces if desired.
        static Figure generateFigure(const Configuration &conf, const string &figName, const bool &triangulate);
        // ?============================================? //
    };

}

#endif //ENGINE_FIGURE3D_H
