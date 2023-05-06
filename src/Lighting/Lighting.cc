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

    Lights3D Light::parseLights(const Configuration &conf, const bool &lighted, const Matrix &eyeMat)
    {
        Lights3D lights;

        if (lighted)
        {
            int nrLights = conf["General"]["nrLights"].as_int_or_die();

            for (int i = 0; i < nrLights; i++)
            {
                string lightName = "Light" + to_string(i);

                vector<double> ambientLight = conf[lightName]["ambientLight"].as_double_tuple_or_default({0, 0, 0});
                vector<double> diffuseLight = {0, 0, 0};
                vector<double> specularLight = {0, 0, 0};

                bool diffuse = conf[lightName]["diffuseLight"].as_double_tuple_if_exists(diffuseLight);
                bool specular = conf[lightName]["specularLight"].as_double_tuple_if_exists(specularLight);

                Light light = Light(ambientLight, diffuseLight, specularLight);

                if (diffuse || specular)
                {
                    light.diffuse = diffuse;
                    light.specular = specular;

                    if (conf[lightName]["infinity"].as_bool_or_die())
                    {
                        // Infinite diffuse light
                        vector<double> ldTuple = conf[lightName]["direction"].as_double_tuple_or_die();
                        Vector3D ldVector = Vector3D::vector(ldTuple[0], ldTuple[1], ldTuple[2]) * eyeMat;
                        ldVector.normalise();
                        light.ldVector = ldVector;

                        lights.emplace_back(light);
                    }
                    else
                    {
                        light.inf = false;
                        vector<double> locationTuple = conf[lightName]["location"].as_double_tuple_or_die();
                        Vector3D location = Vector3D::point(locationTuple[0], locationTuple[1], locationTuple[2]) * eyeMat;
                        light.location = location;

                        double spotAngle;
                        if (conf[lightName]["spotAngle"].as_double_if_exists(spotAngle))
                        {
                            // Spotlight
                            light.spotAngle = spotAngle * M_PI/180;
                            light.spot = true;

                            lights.emplace_back(light);
                        }
                        else
                        {
                            // Point light
                            lights.emplace_back(light);
                        }
                    }
                }
                else
                {
                    lights.emplace_back(ambientLight, diffuseLight, specularLight);
                }
            }
        }
        else
        {
            // Add white ambient light
            // This is needed for old functionality where figures only have 1 (ambient) color and no lights in the .ini
            lights.emplace_back();
        }

        return lights;
    }

    Light Light::totalAmbientAndInfDiffuse(Lights3D &lights, const Vector3D &normal)
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

    Color Light::totalColor(Lights3D &lights, const Vector3D &normal, const Color &diffuseReflection, const Color &specularReflection, const double &reflectionCoeff, const Color &color, const int &x, const int &y, double &d, double &dx, double &dy, double Pz)
    {
        Color total = color;
        Color diffuse = Color();
        Color specular = Color();

        // P in eye coordinates
        double Px = (dx - x) * Pz / d;
        double Py = (dy - y) * Pz / d;
        Vector3D P = Vector3D::point(Px, Py, Pz);

        for (auto &light : lights)
        {
            if (!light.inf)
            {
                Vector3D l = light.location - P;
                l.normalise();

                double cos = Vector3D::dot(l, normal);
                cos = (cos < 0) ? 0 : cos;

                if (light.spot)
                {
                    // Spotlight
                    double cosl = std::cos(light.spotAngle);

                    if (cos > cosl)
                    {
                        double factor = (cos - cosl) / (1 - cosl);
                        Color lightSpot = Color::multiply(light.diffuseLight, factor);
                        diffuse = Color::add(lightSpot, diffuse);
                    }
                }
                else
                {
                    // Point light
                    if (light.diffuse)
                    {
                        Color lightPoint = Color::multiply(light.diffuseLight, cos);
                        diffuse = Color::add(lightPoint, diffuse);
                    }

                    if (light.specular)
                    {
                        Vector3D r = 2 * normal * cos - l;
                        r.normalise();

                        Vector3D camera = -P;
                        camera.normalise();

                        double cosB = Vector3D::dot(r, camera); // -P = camera
                        cosB = (cosB < 0) ? 0 : cosB;

                        double specularFactor = pow(cosB, reflectionCoeff);

                        Color lightPoint = Color::multiply(light.specularLight, specularFactor);
                        specular = Color::add(lightPoint, specular);
                    }
                }
            }
            else if (light.inf && !light.spot && light.specular)
            {
                double cos = Vector3D::dot(-light.ldVector, normal);
                cos = (cos < 0) ? 0 : cos;

                Vector3D r = 2 * normal * cos + light.ldVector;
                r.normalise();

                Vector3D camera = -P;
                camera.normalise();

                double cosB = Vector3D::dot(r, camera); // -P = camera
                cosB = (cosB < 0) ? 0 : cosB;

                double specularFactor = pow(cosB, reflectionCoeff);

                Color lightPoint = Color::multiply(light.specularLight, specularFactor);
                specular = Color::add(lightPoint, specular);
            }
        }

        diffuse = Color::multiply(diffuse, diffuseReflection);
        specular = Color::multiply(specular, specularReflection);

        total = Color::add(total, diffuse);
        total = Color::add(total, specular);

        return total;
    }

}