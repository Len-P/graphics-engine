#ifndef ENGINE_ZBUFFERING_H
#define ENGINE_ZBUFFERING_H

#include "../Figure3D/Figure3D.h"
#include "../Lighting/Lighting.h"
#include "../utils/ColorDouble.h"



using namespace Figure3D;
using namespace Lighting;

class ZBuffering
{
    public:
        static EasyImage parseIni(const Configuration &conf, const bool lighted = false);

    private:
        static EasyImage draw_zbuf_figures(Figures3D &figures, const Lines2D &lines, const int size, const ColorDouble &backgroundColor, Lights3D &lights, Matrix &eyeMat);

        static void draw_zbuf_triangle(ZBuffer &zbuf, EasyImage &image, const Vector3D &A, const Vector3D &B, const Vector3D &C, double d, double dx, double dy, const ColorDouble &ambientReflection, const ColorDouble &diffuseReflection, const ColorDouble &specularReflection, const double reflectionCoeff, Lights3D &lights, Matrix &eyeMat);
};


#endif //ENGINE_ZBUFFERING_H
