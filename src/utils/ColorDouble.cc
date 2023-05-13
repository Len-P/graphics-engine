#include "ColorDouble.h"



ColorDouble::ColorDouble(double r, double g, double b)
{
    this->r = r;
    this->g = g;
    this->b = b;
}

ColorDouble::ColorDouble(vector<double> &colorTuple)
{
    this->r = colorTuple[0];
    this->g = colorTuple[1];
    this->b = colorTuple[2];
}

//void ColorDouble::add(const ColorDouble &color)
//{
//    r = r + color.r;
//    g = g + color.g;
//    b = b + color.b;
//}

ColorDouble ColorDouble::add(const ColorDouble &color1, const ColorDouble &color2)
{
    return {color1.r + color2.r, color1.g + color2.g, color1.b + color2.b};
}

ColorDouble ColorDouble::multiply(const ColorDouble &color1, const ColorDouble &color2)
{
    return {color1.r * color2.r, color1.g * color2.g, color1.b * color2.b};
}

ColorDouble ColorDouble::multiply(const ColorDouble &color, const double &factor)
{
    return {color.r * factor, color.g * factor, color.b * factor};
}

Color ColorDouble::getIntColor(const ColorDouble &color)
{
    double r = (color.r > 1) ? 1 : color.r;
    double g = (color.g > 1) ? 1 : color.g;
    double b = (color.b > 1) ? 1 : color.b;

    return Color(lround(r * 255), lround(g * 255), lround(b * 255));
}
