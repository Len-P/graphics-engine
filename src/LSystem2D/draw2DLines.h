#include "../utils/easy_image.h"
#include "LSystem2D.h"



using namespace img;
using namespace LSystem2D;

using Lines2D = vector<Line2D>;

// Draws all lines from a list of lines on an image. Returns image. Image parameters are size and background color.
EasyImage draw2DLines(Lines2D &lines, int size, const Color &backgroundColor);
