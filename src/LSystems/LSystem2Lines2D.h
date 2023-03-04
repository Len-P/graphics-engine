#include "LSystem2D.h"
#include "../utils/l_parser.h"



using namespace img;
using namespace LSystem2D;

using Lines2D = vector<Line2D>;

// Transforms LSystem to list of lines
Lines2D LSystem2Lines2D(const LParser::LSystem2D &l_system, const Color &color);
