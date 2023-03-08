#ifndef ENGINE_FIGURE3D_H
#define ENGINE_FIGURE3D_H

#include "../utils/ini_configuration.h"
#include "../utils/easy_image.h"
#include "../utils/vector3d.h"
#include <list>



using namespace std;
using namespace ini;
using namespace img;

namespace Figure3D {

    class Face
    {
    public:
        vector<int> point_indexes; // Indexes refer to points in the points vector in the Figure class
    };

    class Figure
    {
    public:
        vector<Vector3D> points;
        vector<Face> faces;
        Color color;

        Figure(vector<Vector3D> &aPoints, vector<Face> &aFaces, const Color &aColor);
    };

    typedef list<Figure> Figures3D;

}
#endif //ENGINE_FIGURE3D_H
