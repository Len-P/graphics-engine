#ifndef ENGINE_LIGHTING_H
#define ENGINE_LIGHTING_H
#define _USE_MATH_DEFINES

#include "../utils/ini_configuration.h"
#include "../utils/easy_image.h"
#include "../utils/vector3d.h"
#include <list>
#include <cmath>



using namespace ini;
using namespace img;
using std::list;
using std::string;
using std::to_string;
using std::pow;

namespace Lighting
{

    class Light;
    typedef list<Light> Lights3D;

    class Light
    {
        public:
            Color ambientLight;
            Color diffuseLight;
            Color specularLight;

            bool inf = true; // False for point lights
            Vector3D ldVector; // The direction in which the light travels

            Vector3D location;

            bool spot = false; // True for spotlights
            double spotAngle{}; // In radians

            bool diffuse = false;
            bool specular = false;

            Light(); // White ambient light, nothing else
            Light(Color &aAmbientLight, Color &aDiffuseLight, Color &aSpecularLight, const Vector3D &aLdVector = Vector3D::vector(0, 0, 0));
            Light(vector<double> &aAmbientLight, vector<double> &aDiffuseLight, vector<double> &aSpecularLight, const Vector3D &aLdVector = Vector3D::vector(0, 0, 0));

            virtual ~Light() = default;

            static Lights3D parseLights(const Configuration &conf, const bool &lighted, const Matrix &eyeMat);

            static Light totalAmbientAndInfDiffuse(Lights3D &lights, const Vector3D &normal);

            static Color totalColor(Lights3D &lights, const Vector3D &normal, const Color &diffuseReflection, const Color &specularReflection, const double &reflectionCoeff, const Color &color, const int &x, const int &y, double &d, double &dx, double &dy, double Pz);
    };

}

#endif //ENGINE_LIGHTING_H
