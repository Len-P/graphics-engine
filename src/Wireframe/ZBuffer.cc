#include "ZBuffer.h"



ZBuffer::ZBuffer(const int width, const int height) : vector<vector<double>>(width, vector<double>(height, std::numeric_limits<double>::infinity())) {}

