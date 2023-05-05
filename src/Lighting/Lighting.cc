#include "Lighting.h"



namespace Lighting
{

    Light::Light()
    {
        ambientLight = Color(255, 255, 255);
        diffuseLight = Color();
        specularLight = Color();
        ldVector = Vector3D::vector(0, 0, 0);
    }

    Light::Light(Color &aAmbientLight, Color &aDiffuseLight, Color &aSpecularLight, const Vector3D &aLdVector)
    {
        ambientLight = aAmbientLight;
        diffuseLight = aDiffuseLight;
        specularLight = aSpecularLight;
        ldVector = aLdVector;
    }

    Light::Light(vector<double> &aAmbientLight, vector<double> &aDiffuseLight, vector<double> &aSpecularLight, const Vector3D &aLdVector)
    {
        ambientLight = Color(aAmbientLight);
        diffuseLight = Color(aDiffuseLight);
        specularLight = Color(aSpecularLight);
        ldVector = aLdVector;
    }

    Light Light::totalLight(Lights3D lights, Vector3D &normal)
    {
        Color ambient = Color();
        Color diffuse = Color();
        Color specular = Color();

        for (auto &light : lights)
        {
            ambient = Color::add(light.ambientLight, ambient);

            if (light.inf)
            {
                double cos = Vector3D::dot(-light.ldVector, normal);
                cos = (cos < 0) ? 0 : cos;

                Color lightDif = Color::multiply(light.diffuseLight, cos);
                diffuse = Color::add(lightDif, diffuse);
            }
        }

        return {ambient, diffuse, specular};
    }

}