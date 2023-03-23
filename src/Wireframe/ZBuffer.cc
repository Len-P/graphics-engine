#include "ZBuffer.h"



ZBuffer::ZBuffer(const int width, const int height) : vector<vector<double>>(width, vector<double>(height, numeric_limits<double>::infinity())) {}

