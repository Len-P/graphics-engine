#ifndef ENGINE_ZBUFFERING_H
#define ENGINE_ZBUFFERING_H

#include "../Figure3D/Figure3D.h"


using namespace Figure3D;

class ZBuffering {
public:
    static EasyImage parseIni(const Configuration &conf);

private:
    static EasyImage draw_zbuf_figures(const Figures3D &figures, const Lines2D &lines, const int size, const Color &backgroundColor);

    static void draw_zbuf_triangle(ZBuffer &zbuf, EasyImage &image, const Vector3D &A, const Vector3D &B, const Vector3D &C, double d, double dx, double dy, const Color &color);

    static void calculateIntermediateX_LandR(const Point2D &P, const Point2D &Q, const int &yI, double &PQxL, double &PQxR);
};


#endif //ENGINE_ZBUFFERING_H
