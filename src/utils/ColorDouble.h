#ifndef ENGINE_COLORDOUBLE_H
#define ENGINE_COLORDOUBLE_H

#include "easy_image.h"
#include <vector>
#include <cmath>


using namespace img;
using std::vector;
using std::round;

class ColorDouble
{
public:
    // Use values between 0 and 1
    double r = 0;
    double g = 0;
    double b = 0;

    ColorDouble() = default;
    ColorDouble(double r, double b, double g);
    explicit ColorDouble(vector<double> &colorTuple);

    // Element-wise addition of 2 colors (none of the components can go above 255)
    void add(const ColorDouble &color);
    static ColorDouble add(const ColorDouble &color1, const ColorDouble &color2);

    // Element-wise multiplication of 2 colors
    void multiply(const ColorDouble &color);
    static ColorDouble multiply(const ColorDouble &color1, const ColorDouble &color2);

    // Multiply each element by a factor
    void multiply(const double &factor);
    static ColorDouble multiply(const ColorDouble &color, const double &factor);

    // Convert to 0-255 color channels
    static Color getIntColor(const ColorDouble &color);

};


#endif //ENGINE_COLORDOUBLE_H
