#ifndef ENGINE_ZBUFFER_H
#define ENGINE_ZBUFFER_H

#include <vector>
#include <limits>



using std::vector;

class ZBuffer: public vector<vector<double>>
{
    public:
        // Constructor creates Z-Buffer of correct size and initializes all fields to +infinity
        ZBuffer(const int width, const int height);
};

#endif //ENGINE_ZBUFFER_H
