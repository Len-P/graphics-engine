#include "Texture.h"

Texture::Texture()
{
    textureImg = EasyImage();
}

Texture::Texture(string &str)
{
    std::ifstream fin("texture.bmp");
    fin >> textureImg;
    fin.close();
}
