#include "Lighting.h"



namespace Lighting
{

    Light::Light()
    {
        ambientLight = Color(1, 1, 1);
        diffuseLight = Color();
        specularLight = Color();
    }

    Light::Light(Color &aAmbientLight, Color &aDiffuseLight, Color &aSpecularLight)
    {
        ambientLight = aAmbientLight;
        diffuseLight = aDiffuseLight;
        specularLight = aSpecularLight;
    }

    Light Light::totalAmbient(Lights3D &lights)
    {
        Light ambient = lights.front();

        // Start loop at 2nd element
        for (auto it = next(lights.begin()); it != lights.end(); ++it)
        {
            ambient.ambientLight.add(it->ambientLight);
        }

        return ambient;
    }

}