#ifndef ENGINE_LIGHTING_H
#define ENGINE_LIGHTING_H
#define _USE_MATH_DEFINES

#include "../utils/ini_configuration.h"
#include "../utils/easy_image.h"
#include "../utils/ColorDouble.h"
#include "../utils/vector3d.h"
#include "../Figure3D/Figure3D.h"
#include "../ZBuffering/ZBuffer.h"
#include "../Figure3D/Transformations.h"
#include <list>
#include <cmath>



using namespace ini;
using namespace img;
using namespace Figure3D;
using std::list;
using std::string;
using std::to_string;
using std::pow;
using std::floor;
using std::ceil;
using std::abs;

namespace Lighting
{

    class Light;
    typedef list<Light> Lights3D;

    class Light
    {
        public:
            ColorDouble ambientLight;
            ColorDouble diffuseLight;
            ColorDouble specularLight;

            bool inf = true; // False for point lights
            Vector3D ldVector; // The direction in which the light travels

            Vector3D location;

            bool spot = false; // True for spotlights
            double spotAngle{}; // In radians

            bool diffuse = false;
            bool specular = false;

            bool shadows = false;
            ZBuffer shadowMask = ZBuffer(1, 1);
            int maskSize{};
            Matrix eyeMat;

            // Values for looking up a point in the shadowMask
            double d{};
            double dx{};
            double dy{};

            Light(); // White ambient light, nothing else
            Light(ColorDouble &aAmbientLight, ColorDouble &aDiffuseLight, ColorDouble &aSpecularLight, const Vector3D &aLdVector = Vector3D::vector(0, 0, 0));
            Light(vector<double> &aAmbientLight, vector<double> &aDiffuseLight, vector<double> &aSpecularLight, const Vector3D &aLdVector = Vector3D::vector(0, 0, 0));

            virtual ~Light() = default;

            static Lights3D parseLights(const Configuration &conf, const bool &lighted, const Matrix &eyeMat);

            static Light totalAmbientAndInfDiffuse(Lights3D &lights, const Vector3D &normal);

            static Color totalColor(Lights3D &lights, const Vector3D &normal, const ColorDouble &diffuseReflection, const ColorDouble &specularReflection, const double &reflectionCoeff, const ColorDouble &color, const int &x, const int &y, double &d, double &dx, double &dy, double Pz, Matrix &eyeMat);

            void calculateShadowMask(Figures3D figures);
    };

}

#endif //ENGINE_LIGHTING_H
