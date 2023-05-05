#ifndef ENGINE_LIGHTING_H
#define ENGINE_LIGHTING_H

#include "../utils/easy_image.h"
#include "../utils/vector3d.h"
#include <list>



using namespace img;
using std::list;

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

            bool inf = true;
            Vector3D ldVector; // The direction in which the light travels

            Vector3D location;
            double spotAngle{};

            Light(); // White ambient light, nothing else
            Light(Color &aAmbientLight, Color &aDiffuseLight, Color &aSpecularLight, const Vector3D &aLdVector = Vector3D::vector(0, 0, 0));
            Light(vector<double> &aAmbientLight, vector<double> &aDiffuseLight, vector<double> &aSpecularLight, const Vector3D &aLdVector = Vector3D::vector(0, 0, 0));

            virtual ~Light() = default;

            static Light totalLight(Lights3D lights, Vector3D &normal);
    };

}

#endif //ENGINE_LIGHTING_H
