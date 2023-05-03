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

            Light(); // White ambient light, nothing else
            Light(Color &aAmbientLight, Color &aDiffuseLight, Color &aSpecularLight);

            static Light totalAmbient(Lights3D &lights);
    };

    class InfLight: public Light
    {
        public:
            // The direction in which the light travels
            Vector3D ldVector;
    };

    class PointLight: public Light
    {
        public:
            Vector3D location;
            double spotAngle;
    };

}

#endif //ENGINE_LIGHTING_H
