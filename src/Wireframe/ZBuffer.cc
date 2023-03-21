#include "ZBuffer.h"



ZBuffer::ZBuffer(const int width, const int height) : vector<vector<double>>(height, vector<double>(width, numeric_limits<double>::infinity())) {}

