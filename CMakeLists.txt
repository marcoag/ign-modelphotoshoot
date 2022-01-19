cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(ignition-cmake2 REQUIRED)

project(ModelPhotoShoot)

find_package(ignition-rendering6 REQUIRED OPTIONAL_COMPONENTS ogre ogre2)
find_package(ignition-sensors6 REQUIRED COMPONENTS rendering camera)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

if (TARGET ignition-rendering6::ogre)
   add_definitions(-DWITH_OGRE)
endif()
if (TARGET ignition-rendering6::ogre2)
   add_definitions(-DWITH_OGRE2)
endif()

find_package(ignition-gazebo6 REQUIRED)
add_library(ModelPhotoShoot SHARED ModelPhotoShoot.cc)
set_property(TARGET ModelPhotoShoot PROPERTY CXX_STANDARD 17)
target_link_libraries(ModelPhotoShoot
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo6::ignition-gazebo6
  PRIVATE ignition-sensors6::camera
  PRIVATE ignition-rendering6::ignition-rendering6
  )
