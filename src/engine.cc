#include "utils/easy_image.h"
#include "utils/ini_configuration.h"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include "LSystem2D/LSystem2D.h"
#include "Figure3D/Figure3D.h"
#include "ZBuffering/ZBuffering.h"



using namespace ini;
using namespace img;

EasyImage generate_image(const Configuration &conf)
{
    string type = conf["General"]["type"].as_string_or_die();

    if (type == "2DLSystem")
    {
        return LSystem2D::parseIni(conf);
    }
    else if (type == "Wireframe")
    {
        return Figure3D::parseIni(conf, false);
    }
    else if (type == "ZBufferedWireframe")
    {
        return Figure3D::parseIni(conf, true);
    }
    else if (type == "ZBuffering")
    {
        return ZBuffering::parseIni(conf, false);
    }
    else if (type == "LightedZBuffering")
    {
        return ZBuffering::parseIni(conf, true);
    }
    else
    {
        std::cerr << "Error: Type wasn't recognized." << std::endl;
        return {};
    }
}

int main(int argc, char const* argv[])
{
        int retVal = 0;
        try
        {
                std::vector<std::string> args = std::vector<std::string>(argv+1, argv+argc);
                if (args.empty()) {
                        std::ifstream fileIn("filelist");
                        std::string filelistName;
                        while (std::getline(fileIn, filelistName)) {
                                args.push_back(filelistName);
                        }
                }
                for(std::string fileName : args)
                {
                        ini::Configuration conf;
                        try
                        {
                                std::ifstream fin(fileName);
                                if (fin.peek() == std::istream::traits_type::eof()) {
                                    std::cout << "Ini file appears empty. Does '" <<
                                    fileName << "' exist?" << std::endl;
                                    continue;
                                }
                                fin >> conf;
                                fin.close();
                        }
                        catch(ini::ParseException& ex)
                        {
                                std::cerr << "Error parsing file: " << fileName << ": " << ex.what() << std::endl;
                                retVal = 1;
                                continue;
                        }

                        img::EasyImage image = generate_image(conf);
                        if(image.get_height() > 0 && image.get_width() > 0)
                        {
                                std::string::size_type pos = fileName.rfind('.');
                                if(pos == std::string::npos)
                                {
                                        //filename does not contain a '.' --> append a '.bmp' suffix
                                        fileName += ".bmp";
                                }
                                else
                                {
                                        fileName = fileName.substr(0,pos) + ".bmp";
                                }
                                try
                                {
                                        std::ofstream f_out(fileName.c_str(),std::ios::trunc | std::ios::out | std::ios::binary);
                                        f_out << image;

                                }
                                catch(std::exception& ex)
                                {
                                        std::cerr << "Failed to write image to file: " << ex.what() << std::endl;
                                        retVal = 1;
                                }
                        }
                        else
                        {
                                std::cout << "Could not generate image for " << fileName << std::endl;
                        }
                }
        }
        catch(const std::bad_alloc &exception)
        {
    		//When you run out of memory this exception is thrown. When this happens the return value of the program MUST be '100'.
    		//Basically this return value tells our automated test scripts to run your engine on a pc with more memory.
    		//(Unless of course you are already consuming the maximum allowed amount of memory)
    		//If your engine does NOT adhere to this requirement you risk losing points because then our scripts will
		//mark the test as failed while in reality it just needed a bit more memory
                std::cerr << "Error: insufficient memory" << std::endl;
                retVal = 100;
        }
        return retVal;
}
