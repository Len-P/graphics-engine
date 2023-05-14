#ifndef ENGINE_TEXTURE_H
#define ENGINE_TEXTURE_H

#include "../utils/ini_configuration.h"
#include "../utils/easy_image.h"
#include <string>
#include <iostream>
#include <fstream>



using namespace ini;
using namespace img;
using std::string;

class Texture
{
public:
    EasyImage textureImg;

    Texture();

    explicit Texture(string &str);

};


#endif //ENGINE_TEXTURE_H
