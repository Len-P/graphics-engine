cmake_minimum_required(VERSION 3.5)

project(engine)

############################################################
# Set compiler flags
############################################################
# If supported by your compiler, you can add the -Wall, -Wextra, –fstack-protector-all and -g3 flags here.
set(OWN_GXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${OWN_GXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OWN_GXX_FLAGS}")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${OWN_GXX_FLAGS}")

############################################################
# List all sources
############################################################
set(engine_sources
		src/utils/easy_image.cc
		src/engine.cc
		src/utils/ini_configuration.cc
		src/utils/l_parser.cc src/utils/l_parser.h
		src/utils/vector3d.cc src/utils/vector3d.h
		src/utils/ColorDouble.cc src/utils/ColorDouble.h
		src/LSystem2D/LSystem2D.cc src/LSystem2D/LSystem2D.h
        src/Figure3D/Figure3D.cc src/Figure3D/Figure3D.h
		src/ZBuffering/ZBuffer.cc src/ZBuffering/ZBuffer.h
		src/ZBuffering/ZBuffering.cc src/ZBuffering/ZBuffering.h
		src/Fractal3D/Fractal3D.cc src/Fractal3D/Fractal3D.h
		src/Figure3D/Transformations.cc src/Figure3D/Transformations.h
		src/Lighting/Lighting.cc src/Lighting/Lighting.h)

############################################################
# Create an executable
############################################################
set(exe_name "engine")
add_executable( ${exe_name} ${engine_sources} )
install( TARGETS ${exe_name} DESTINATION ${PROJECT_SOURCE_DIR}/ )
