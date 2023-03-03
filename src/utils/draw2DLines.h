#include "easy_image.h"
#include <list>
#include "../LSystems/LSystems.h"



using namespace img;
using namespace lsys;

using Lines2D = vector<Line2D>;

// Draws lines on an image from a list of lines. Returns image. Image parameters are size and background color.
EasyImage draw2DLines(Lines2D &lines, int size, const Color &backgroundColor);
