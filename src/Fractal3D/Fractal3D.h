#ifndef ENGINE_FRACTAL3D_H
#define ENGINE_FRACTAL3D_H

#include <list>
#include "../utils/vector3d.h"
#include "../Figure3D/Transformations.h"



using std::list;

namespace Figure3D {
    class Figure;
    typedef list<Figure> Figures3D;
}

class Fractal3D {
public:
    static void generateFractal(Figure3D::Figure &fig, Figure3D::Figures3D &fractal, const int nrIter, const double scale);
};


#endif //ENGINE_FRACTAL3D_H
