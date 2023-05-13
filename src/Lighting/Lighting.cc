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
            bool shadowEnabled = false;
            int shadowMaskSize;

            if (conf["General"]["shadowMask"].as_int_if_exists(shadowMaskSize))
            {
                shadowEnabled = true;
            }

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
                        Vector3D location = Vector3D::point(locationTuple[0], locationTuple[1], locationTuple[2]);
                        light.location = location * eyeMat;

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
                            if (shadowEnabled)
                            {
                                light.shadows = true;
                                light.maskSize = shadowMaskSize;
                                light.eyeMat = Transformations::eyePointTrans(location);
                            }

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

    Color Light::totalColor(Lights3D &lights, const Vector3D &normal, const Color &diffuseReflection, const Color &specularReflection, const double &reflectionCoeff, const Color &color, const int &x, const int &y, double &d, double &dx, double &dy, double Pz, Matrix &eyeMat)
    {
        Color total = color;
        Color diffuse = Color();
        Color specular = Color();

        // P in eye coordinates
        double Px = (dx - x) * Pz / d;
        double Py = (dy - y) * Pz / d;
        Vector3D P = Vector3D::point(Px, Py, Pz);

        for (const auto &light : lights)
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
                    bool lit = true;

                    if (light.shadows)
                    {
                        Vector3D P_world = P * Matrix::inv(eyeMat);
                        Vector3D P_light = P_world * light.eyeMat;

                        double x_light2D = -P_light.x * light.d / P_light.z + light.dx;
                        double y_light2D = -P_light.y * light.d / P_light.z + light.dy;

                        // Interpolate using 4 nearest points in shadow mask
                        int xF = (int) floor(x_light2D);
                        int xC = (int) ceil(x_light2D);
                        int yF = (int) floor(y_light2D);
                        int yC = (int) ceil(y_light2D);

                        double alphaX = x_light2D - xF;
                        double alphaY = y_light2D - yF;

                        double ZinvA = light.shadowMask[xF][yC];
                        double ZinvB = light.shadowMask[xC][yC];
                        double ZinvC = light.shadowMask[xF][yF];
                        double ZinvD = light.shadowMask[xC][yF];

                        double ZinvE = (1 - alphaX) * ZinvA + alphaX * ZinvB;
                        double ZinvF = (1 - alphaX) * ZinvC + alphaX * ZinvD;

                        double Zinv = alphaY * ZinvE + (1 - alphaY) * ZinvF;

                        // If difference between ZBuffer and shadow mask is large enough, the light is being obstructed
                        if (abs(Zinv - 1/P_light.z) > 0.0001)
                        {
                            lit = false;
                        }
                    }

                    if (light.diffuse && lit)
                    {
                        Color lightPoint = Color::multiply(light.diffuseLight, cos);
                        diffuse = Color::add(lightPoint, diffuse);
                    }

                    if (light.specular && lit)
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

    void Light::calculateShadowMask(Figures3D figures)
    {
        if (this->shadows)
        {
            // Eye Point Transformation and Projection
            Transformations::applyTransformation(figures, this->eyeMat);
            Lines2D lines = Transformations::doProjection(figures);

            double x_mi = numeric_limits<double>::infinity();
            double y_mi = numeric_limits<double>::infinity();
            double x_ma = -numeric_limits<double>::infinity();
            double y_ma = -numeric_limits<double>::infinity();

            // Iterate over all lines and update min and max values
            for (const auto &line : lines)
            {
                x_mi = min(x_mi, min(line.p1.x, line.p2.x));
                y_mi = min(y_mi, min(line.p1.y, line.p2.y));
                x_ma = max(x_ma, max(line.p1.x, line.p2.x));
                y_ma = max(y_ma, max(line.p1.y, line.p2.y));
            }

            // Calculate mask resolution
            double x_range = x_ma - x_mi;
            double y_range = y_ma - y_mi;
            double image_x = this->maskSize * (x_range / max(x_range, y_range));
            double image_y = this->maskSize * (y_range / max(x_range, y_range));

            // Calculate scale factor d
            double d = 0.95 * image_x /  x_range;

            // Calculate translation distances dx and dy
            double DC_x = d * (x_mi + x_ma)/2;
            double DC_y = d * (y_mi + y_ma)/2;
            double dx = image_x/2 - DC_x;
            double dy = image_y/2 - DC_y;

            image_x = lround(image_x);
            image_y = lround(image_y);

            this->shadowMask = ZBuffer((int) image_x, (int) image_y);
            this->d = d;
            this->dx = dx;
            this->dy = dy;

            for (auto &fig : figures)
            {
                for (const auto &face : fig.faces)
                {
                    Vector3D A = fig.points[face.pointIndexes[0]];
                    Vector3D B = fig.points[face.pointIndexes[1]];
                    Vector3D C = fig.points[face.pointIndexes[2]];

                    Point2D A_2D = Point2D(-d * A.x / A.z + dx, -d * A.y / A.z + dy);
                    Point2D B_2D = Point2D(-d * B.x / B.z + dx, -d * B.y / B.z + dy);
                    Point2D C_2D = Point2D(-d * C.x / C.z + dx, -d * C.y / C.z + dy);

                    Vector3D G = Vector3D::point((A_2D.x + B_2D.x + C_2D.x)/3, (A_2D.y + B_2D.y + C_2D.y)/3, 1/(3*A.z) + 1/(3*B.z) + 1/(3*C.z) );

                    Vector3D u = B - A;
                    Vector3D v = C - A;
                    Vector3D w = Vector3D::cross(u, v);

                    double k = w.x * A.x + w.y * A.y + w.z * A.z;
                    double dzdx = w.x / (-d * k);
                    double dzdy = w.y / (-d * k);

                    double y_min = min(A_2D.y, min(B_2D.y, C_2D.y));
                    double y_max = max(A_2D.y, max(B_2D.y, C_2D.y));

                    int y_minInt = lround(y_min + 0.5);
                    int y_maxInt = lround(y_max - 0.5);

                    for (int yI = y_minInt; yI < y_maxInt + 1; yI++)
                    {
                        double ABxL = numeric_limits<double>::infinity();
                        double ACxL = numeric_limits<double>::infinity();
                        double BCxL = numeric_limits<double>::infinity();

                        double ABxR = -numeric_limits<double>::infinity();
                        double ACxR = -numeric_limits<double>::infinity();
                        double BCxR = -numeric_limits<double>::infinity();

                        LSystem2D::Line2D::calculateIntermediateX_LandR(A_2D, B_2D, yI, ABxL, ABxR);
                        LSystem2D::Line2D::calculateIntermediateX_LandR(C_2D, A_2D, yI, ACxL, ACxR);
                        LSystem2D::Line2D::calculateIntermediateX_LandR(B_2D, C_2D, yI, BCxL, BCxR);

                        int xL = lround(min(ABxL, min(BCxL, ACxL)) + 0.5);
                        int xR = lround(max(ABxR, max(BCxR, ACxR)) - 0.5);

                        for (int i = xL; i < xR + 1; i ++)
                        {
                            double z = (1.0001 * G.z) + (i - G.x) * dzdx + (yI - G.y) * dzdy;

                            if (z < this->shadowMask[i][yI])
                            {
                                this->shadowMask[i][yI] = z;
                            }
                        }
                    }
                }
            }
        }
    }

}