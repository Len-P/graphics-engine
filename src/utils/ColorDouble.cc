#include "ColorDouble.h"



ColorDouble::ColorDouble(double r, double b, double g)
{
    this->r = r;
    this->b = b;
    this->g = g;
}

ColorDouble ColorDouble::add(const ColorDouble &color1, const ColorDouble &color2)
{
    return {color1.r + color2.r, color1.b + color2.b, color1.g + color2.g};
}

ColorDouble ColorDouble::multiply(const ColorDouble &color1, const ColorDouble &color2)
{
    return {color1.r * color2.r, color1.b * color2.b, color1.g * color2.g};
}

ColorDouble ColorDouble::multiply(const ColorDouble &color, const double &factor)
{
    return {color.r * factor, color.b * factor, color.g * factor};
}
